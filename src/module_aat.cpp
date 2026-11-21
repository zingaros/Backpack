#if defined(AAT_BACKPACK)
#include "common.h"
#include "module_aat.h"
#include "aat_navigation.h"
#include "aat_projection.h"
#include "aat_servo.h"
#include "logging.h"
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING) || defined(AAT_I2C_TRACE)
#include "aat_hmc5883l.h"
#endif

#include <math.h>
#include <Arduino.h>
#if defined(AAT_PERF_DEBUG)
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
static portMUX_TYPE aatPerfMux = portMUX_INITIALIZER_UNLOCKED;
#endif
#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#define DELAY_IDLE          (20U)   // sleep used when not tracking
#define DELAY_FIRST_UPDATE  (5000U) // absolute delay before first servo update
#define FONT_W              (6)     // Actually 5x7 + 1 pixel space
#define FONT_H              (8)
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
static constexpr uint32_t SERVO_TEST_SWEEP_TIMEOUT_MS = 90000U;
static constexpr uint16_t SERVO_TEST_SWEEP_PULSE_US = 1450U;
#endif
#if defined(AAT_SERVO_TEST)
static constexpr uint32_t SERVO_TEST_REFERENCE_DEBOUNCE_MS = 30U;
static constexpr uint32_t SERVO_TEST_COMPASS_RETRY_MS = 1000U;
#endif

#if defined(AAT_SERVO_TEST)
static AatHmc5883l servoTestCompass(Wire);
#endif
#if defined(AAT_I2C_TRACE)
static void pollCompassTrace(uint32_t now)
{
    static AatHmc5883l sensor(Wire);
    static uint32_t lastPollMs = 0;
    static uint32_t cycle = 0;
    static bool configured = false;
#if defined(AAT_I2C_TRACE_SINGLE_SHOT)
    static uint32_t lastTriggerMs = 0;
#endif
    if (now - lastPollMs < 100U || cycle >= 1000U)
        return;
    lastPollMs = now;
    uint8_t identity[3] = {};
    const bool detected = sensor.readIdentity(identity) && sensor.hasHmcIdentity(identity);
    if (detected && !configured)
    {
#if defined(AAT_I2C_TRACE_SINGLE_SHOT)
        configured = sensor.configureSingleShot(0x70);
        lastTriggerMs = now;
#else
        configured = sensor.configure();
#endif
#if defined(AAT_I2C_TRACE_DEFER_STATUS)
    if (configured)
        return;
#endif
    }
    AatHmc5883l::RawSample sample = {};
    uint8_t status = 0;
    const bool available = configured && sensor.readRaw(&sample, &status);
    uint8_t settings[3] = {};
    const bool settingsValid = configured && sensor.readConfiguration(settings);
    Serial.printf("I2C_CYCLE n=%lu ms=%lu clock=%lu detected=%u configured=%u sample=%u status=%02x after=%02x after_valid=%u config_valid=%u config=%02x%02x%02x xyz=%d,%d,%d\n",
        static_cast<unsigned long>(++cycle), static_cast<unsigned long>(now),
        static_cast<unsigned long>(Wire.getClock()), detected, configured, available,
        status, sample.statusAfterRead, sample.statusAfterReadValid, settingsValid,
        settings[0], settings[1], settings[2], sample.x, sample.y, sample.z);
#if defined(AAT_I2C_TRACE_SINGLE_SHOT)
    if (configured && (available || now - lastTriggerMs >= 1000U))
    {
        const bool triggered = sensor.triggerSingleShot();
#if defined(AAT_I2C_TRACE_DEFER_STATUS)
        Serial.printf("I2C_TRIGGER ok=%u status_deferred=1\n", triggered);
#else
        uint8_t triggerStatus = 0;
        const bool statusValid = sensor.readStatus(&triggerStatus);
        Serial.printf("I2C_TRIGGER ok=%u status_valid=%u status=%02x\n",
            triggered, statusValid, triggerStatus);
#endif
        lastTriggerMs = now;
    }
#endif
}
#endif
#if defined(AAT_COMPASS_TRACKING)
static AatHmc5883l compassPanSensor(Wire);
static constexpr uint32_t COMPASS_PAN_POLL_INTERVAL_MS = 50U;
static constexpr uint32_t COMPASS_PAN_REFERENCE_DEBOUNCE_MS = 30U;
static constexpr uint32_t COMPASS_PAN_RETRY_MS = 1000U;
static constexpr uint32_t COMPASS_PAN_CONFIG_REFRESH_MS = 1000U;
static constexpr uint32_t COMPASS_PAN_SAMPLE_TIMEOUT_MS = 500U;
#if defined(AAT_COMPASS_TRACKING_CONFIG_A)
static constexpr uint8_t COMPASS_PAN_CONFIG_A = AAT_COMPASS_TRACKING_CONFIG_A;
#else
static constexpr uint8_t COMPASS_PAN_CONFIG_A = 0x68;
#endif

static bool calculateCompassPanHeadingDegrees(int16_t rawX, int16_t rawY,
    int32_t *headingDegrees)
{
    if (headingDegrees == nullptr)
        return false;
    *headingDegrees = 0;
    const aat_pan_compass_calibration_t saved = config.GetAatPanCompassCalibration();
    const AatCompass::Calibration calibration = {
        static_cast<float>(saved.offsetX),
        static_cast<float>(saved.offsetY),
        saved.scaleXPermille / 1000.0f,
        saved.scaleYPermille / 1000.0f,
        saved.headingOffsetDegrees,
        saved.valid == 1
    };
    int32_t headingMilliDegrees = 0;
    if (!AatCompass::calculateHeadingMilliDegrees(rawX, rawY,
        calibration, &headingMilliDegrees, config.IsAatPanCompassReversed()))
        return false;
    *headingDegrees = AatCompass::normalizeHeading((headingMilliDegrees + 500) / 1000);
    return true;
}

static bool saveCompassPanNorthAlignment(int16_t rawX, int16_t rawY,
    int32_t referenceBearingDegrees, bool deriveOffset)
{
    aat_pan_compass_calibration_t saved = config.GetAatPanCompassCalibration();
    if (saved.valid != 1)
        return false;

    if (deriveOffset)
    {
        const AatCompass::Calibration sensorCalibration = {
            static_cast<float>(saved.offsetX),
            static_cast<float>(saved.offsetY),
            saved.scaleXPermille / 1000.0f,
            saved.scaleYPermille / 1000.0f,
            0,
            true
        };
        int32_t sensorHeadingMilliDegrees = 0;
        if (!AatCompass::calculateHeadingMilliDegrees(rawX, rawY,
            sensorCalibration, &sensorHeadingMilliDegrees, config.IsAatPanCompassReversed())
            || !AatCompass::calculateCompassHeadingOffsetDegrees(
                sensorHeadingMilliDegrees, referenceBearingDegrees,
                &saved.headingOffsetDegrees))
            return false;
    }
    else
        saved.headingOffsetDegrees = 0;

    if (!config.SetAatPanCompassCalibration(saved)
        || !config.SetAatPanCompassNorthAligned(true))
        return false;
    config.Commit();
    return true;
}
#endif

VbatSampler::VbatSampler() :
    _lastUpdate(0), _value(0), _sum(0), _cnt(0)
{
}

void VbatSampler::update(uint32_t now)
{
    if (_lastUpdate != 0 && now - _lastUpdate < VBAT_UPDATE_INTERVAL)
        return;
    _lastUpdate = now;

    uint32_t adc = analogRead(A0);
    _sum += adc;
    ++_cnt;

    if (_cnt >= VBAT_UPDATE_COUNT)
    {
        adc = _sum / _cnt;
        // For negative offsets, anything between abs(OFFSET) and 0 is considered 0
        if ((config.GetVbatOffset() < 0 && (int32_t)adc <= -config.GetVbatOffset())
            || config.GetVbatScale() == 0)
            _value = 0;
        else
            _value = ((int32_t)adc - config.GetVbatOffset()) * 100 / config.GetVbatScale();

        _sum = 0;
        _cnt = 0;
    }
}

AatModule::AatModule(Stream &port) :
    CrsfModuleBase(port), _gpsLast{0}, _home{0},
    _gpsAvgUpdateIntervalMs(0), _lastServoUpdateMs(0), _targetDistance(0),
    _targetAzim(0), _targetElev(0), _azimMsPerDegree(0),
    _servoPos{0}, _lastAzimFlipMs(0)
#if defined(PIN_SERVO_AZIM)
    , _servo_Azim()
#endif
#if defined(PIN_SERVO_ELEV)
    , _servo_Elev()
#endif
#if defined(PIN_OLED_SDA)
    , _display(SCREEN_WIDTH, SCREEN_HEIGHT), _lastDisplayActiveMs(0)
#endif
{
    // Init is called manually
}

#if defined(AAT_COMPASS_TRACKING)
void AatModule::updateCompassPan(uint32_t now)
{
#if defined(AAT_COMPASS_TRACKING_TRACE)
    const uint32_t updateStartedAtMs = millis();
    uint8_t setupRateTraceState = 0;
#endif
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    if (_compassPanSetupRequested.exchange(false))
    {
        _compassPanSetupMode.store(true);
        _servoTestRequestedPulseUs.store(neutralUs);
        _servoTestLastCommandMs.store(now);
        _servoTestCalibratedRateWindow.reset();
        _servoTestCompassCalibratedRateValid.store(false);
        _servoTestPanSafety.invalidate();
        if (_compassPanAppliedPulseUs != neutralUs)
        {
            _servo_Azim.writeMicroseconds(neutralUs);
            _compassPanAppliedPulseUs = neutralUs;
        }
#if defined(PIN_OLED_SDA) && !defined(AAT_SERVO_TEST_DISABLE_OLED)
        displayCompassPanSetup();
#endif
    }
    bool compassSampleReady = false;
    const bool rawReferencePressed = digitalRead(PIN_AAT_REFERENCE_BUTTON) == LOW;
    if (rawReferencePressed != _compassPanRawReferencePressed)
    {
        _compassPanRawReferencePressed = rawReferencePressed;
        _compassPanReferenceDebounceMs = now;
    }
    else if (now - _compassPanReferenceDebounceMs >= COMPASS_PAN_REFERENCE_DEBOUNCE_MS)
    {
        _compassPanReferencePressed = rawReferencePressed;
    }

            _servoTestReferencePressed.store(_compassPanReferencePressed);
    if (now - _compassPanLastPollMs >= COMPASS_PAN_POLL_INTERVAL_MS)
    {
        _compassPanLastPollMs = now;
        _servoTestCompassPollCount.fetch_add(1);
        if (!_compassPanConfigured)
        {
            uint8_t identity[3] = {};
            if (compassPanSensor.readIdentity(identity) && compassPanSensor.hasHmcIdentity(identity))
            {
                _compassPanConfigured = compassPanSensor.configureSingleShot(COMPASS_PAN_CONFIG_A);
                if (_compassPanConfigured)
                {
                    _compassPanLastTriggerMs = now;
                    uint8_t settings[3] = {};
                    _compassPanConfigReadValid = compassPanSensor.readConfiguration(settings);
                    _compassPanConfigReadAtMs = now;
                    if (_compassPanConfigReadValid)
                        _compassPanConfigA = settings[0];
                    _servoTestCompassConfigReadValid.store(_compassPanConfigReadValid);
                    _servoTestCompassConfigReadAtMs.store(now);
                    if (_compassPanConfigReadValid)
                    {
                        _servoTestCompassConfigA.store(settings[0]);
                        _servoTestCompassConfigB.store(settings[1]);
                        _servoTestCompassConfigMode.store(settings[2]);
                    }
                    uint8_t triggerStatus = 0;
                    _compassPanStatusAfterTriggerValid = compassPanSensor.readStatus(&triggerStatus);
                    if (_compassPanStatusAfterTriggerValid)
                        _compassPanStatusAfterTrigger = triggerStatus;
                    _servoTestCompassStatusAfterTriggerValid.store(_compassPanStatusAfterTriggerValid);
                    _servoTestCompassStatusAfterTrigger.store(_compassPanStatusAfterTrigger);
                    _servoTestCompassStatusAfterTriggerAtMs.store(now);
                }
            }
            _compassPanDetected = _compassPanConfigured;
            _servoTestCompassDetected.store(_compassPanDetected);
        }
        else
        {
            AatHmc5883l::RawSample sample = {};
            if (compassPanSensor.readRaw(&sample))
            {
                compassSampleReady = true;
                _compassPanRawX = sample.x;
                _compassPanRawY = sample.y;
                _servoTestCompassRawZ.store(sample.z);
#if defined(AAT_COMPASS_TRACKING_TRACE)
                _compassPanRawZ = sample.z;
                _compassPanSampleStatus = sample.status;
                _compassPanSampleStatusAfterRead = sample.statusAfterRead;
                _compassPanSampleStatusAfterReadValid = sample.statusAfterReadValid;
#endif
                _servoTestCompassStatus.store(sample.status);
                _servoTestCompassStatusAfterRead.store(sample.statusAfterRead);
                _servoTestCompassStatusAfterReadValid.store(sample.statusAfterReadValid);
                _servoTestCompassRawX.store(sample.x);
                _servoTestCompassRawY.store(sample.y);
                _servoTestCompassHeadingValid.store(sample.statusAfterReadValid);
                _servoTestCompassSampleAtMs.store(now);
                _servoTestCompassSampleCount.fetch_add(1);
                const float rawHeadingDegrees = AatCompass::headingFromForwardRightXY(
                    sample.x, sample.y);
                const float normalizedRawHeading = rawHeadingDegrees < 0.0f
                    ? rawHeadingDegrees + 360.0f : rawHeadingDegrees;
                _servoTestCompassHeadingMilliDegrees.store(
                    static_cast<int32_t>(lroundf(normalizedRawHeading * 1000.0f)));
                const aat_pan_compass_calibration_t saved = config.GetAatPanCompassCalibration();
                const AatCompass::Calibration calibration = {
                    static_cast<float>(saved.offsetX),
                    static_cast<float>(saved.offsetY),
                    saved.scaleXPermille / 1000.0f,
                    saved.scaleYPermille / 1000.0f,
                    saved.headingOffsetDegrees,
                    saved.valid == 1
                };
                int32_t headingMilliDegrees = 0;
                const bool headingValid = AatCompass::calculateHeadingMilliDegrees(
                    sample.x, sample.y, calibration, &headingMilliDegrees, config.IsAatPanCompassReversed());
                _servoTestCompassCalibratedHeadingValid.store(headingValid);
                if (headingValid)
                    _servoTestCompassCalibratedHeadingDegrees.store(
                        AatCompass::normalizeHeading((headingMilliDegrees + 500) / 1000));
                if (!compassPanSensor.triggerSingleShot())
                {
#if defined(AAT_COMPASS_TRACKING_TRACE)
                    setupRateTraceState = 9;
#endif
                    _compassPanConfigured = false;
                    _compassPanDetected = false;
                    _compassPanHeadingValid = false;
                    _compassPanStatusAfterTriggerValid = false;
                    _servoTestCompassDetected.store(false);
                    _servoTestCompassStatusAfterTriggerValid.store(false);
                    _servoTestCalibratedRateWindow.reset();
                    _servoTestCompassCalibratedRateValid.store(false);
                }
                else
                {
                    _compassPanLastTriggerMs = now;
                    uint8_t triggerStatus = 0;
                    _compassPanStatusAfterTriggerValid = compassPanSensor.readStatus(&triggerStatus);
                    if (_compassPanStatusAfterTriggerValid)
                        _compassPanStatusAfterTrigger = triggerStatus;
                    _servoTestCompassStatusAfterTriggerValid.store(_compassPanStatusAfterTriggerValid);
                    _servoTestCompassStatusAfterTrigger.store(_compassPanStatusAfterTrigger);
                    _servoTestCompassStatusAfterTriggerAtMs.store(now);
                    const bool triggerReleased = AatCompass::isCompassSampleReleaseConfirmed(
                        _compassPanStatusAfterTriggerValid, _compassPanStatusAfterTrigger);
                    const bool configMatches = AatServo::isCompassConfigurationValid(
                        _compassPanConfigReadValid, _compassPanConfigA, COMPASS_PAN_CONFIG_A);
#if defined(AAT_COMPASS_TRACKING_TRACE)
                    if (!headingValid)
                        setupRateTraceState = 2;
                    else if (!sample.statusAfterReadValid)
                        setupRateTraceState = 3;
                    else if (!triggerReleased)
                        setupRateTraceState = 4;
                    else if (!configMatches)
                        setupRateTraceState = 5;
#endif
                    if (headingValid && sample.statusAfterReadValid && triggerReleased
                        && configMatches)
                    {
                        _compassPanHeadingDegrees = AatCompass::normalizeHeading(
                            (headingMilliDegrees + 500) / 1000);
                        _compassPanSampleAtMs = now;
                        _compassPanHeadingValid = true;
                        int32_t rateMilliDegreesPerSecond = 0;
                        const AatCompass::RateWindowUpdate rateUpdate =
                            _servoTestCalibratedRateWindow.update(now, headingMilliDegrees,
                                &rateMilliDegreesPerSecond);
#if defined(AAT_COMPASS_TRACKING_TRACE)
                        setupRateTraceState = 6 + static_cast<uint8_t>(rateUpdate);
#endif
                        if (rateUpdate == AatCompass::RateWindowUpdate::Updated)
                        {
                            _servoTestCompassCalibratedRateMilliDegreesPerSecond.store(
                                rateMilliDegreesPerSecond);
                            _servoTestCompassCalibratedRateValid.store(true);
                            _servoTestCompassCalibratedRateSequence.fetch_add(1);
                        }
                        else if (rateUpdate == AatCompass::RateWindowUpdate::Reset)
                            _servoTestCompassCalibratedRateValid.store(false);
                    }
                    else
                    {
                        _compassPanHeadingValid = false;
                        _servoTestCalibratedRateWindow.reset();
                        _servoTestCompassCalibratedRateValid.store(false);
                    }
                }
            }
            else if (now - _compassPanLastTriggerMs >= COMPASS_PAN_RETRY_MS)
            {
                if (!compassPanSensor.triggerSingleShot())
                {
                    _compassPanConfigured = false;
                    _compassPanDetected = false;
                    _compassPanHeadingValid = false;
                    _compassPanStatusAfterTriggerValid = false;
                    _servoTestCompassDetected.store(false);
                    _servoTestCompassStatusAfterTriggerValid.store(false);
                }
                else
                {
                    _compassPanLastTriggerMs = now;
                    uint8_t triggerStatus = 0;
                    _compassPanStatusAfterTriggerValid = compassPanSensor.readStatus(&triggerStatus);
                    if (_compassPanStatusAfterTriggerValid)
                        _compassPanStatusAfterTrigger = triggerStatus;
                    _servoTestCompassStatusAfterTriggerValid.store(_compassPanStatusAfterTriggerValid);
                    _servoTestCompassStatusAfterTrigger.store(_compassPanStatusAfterTrigger);
                    _servoTestCompassStatusAfterTriggerAtMs.store(now);
                }
            }
        }
    }

    if (_compassPanConfigured
        && now - _compassPanConfigReadAtMs >= COMPASS_PAN_CONFIG_REFRESH_MS)
    {
        uint8_t settings[3] = {};
        _compassPanConfigReadValid = compassPanSensor.readConfiguration(settings);
        _compassPanConfigReadAtMs = now;
        _servoTestCompassConfigReadValid.store(_compassPanConfigReadValid);
        _servoTestCompassConfigReadAtMs.store(now);
        if (_compassPanConfigReadValid)
        {
            _compassPanConfigA = settings[0];
            _servoTestCompassConfigA.store(settings[0]);
            _servoTestCompassConfigB.store(settings[1]);
            _servoTestCompassConfigMode.store(settings[2]);
        }
    }

    now = millis();
    const AatServo::CompassPanOperatingState operatingState =
        AatServo::getCompassPanOperatingState(_compassPanModeActive,
            _compassPanSetupMode.load(),
            config.GetAatPanCompassCalibration().valid == 1,
            config.GetAatPanCompassNorthAligned(),
            _compassPanStartupAlignmentComplete);
    if (operatingState == AatServo::CompassPanOperatingState::Setup)
    {
        _compassPanTrackingMedian.reset();
        _servoTestReferencePressed.store(_compassPanReferencePressed);
        uint8_t sweepState = _servoTestSweepState.load();
        if (compassSampleReady
            && sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping))
        {
            const uint32_t sampleAtMs = _servoTestCompassSampleAtMs.load();
            const uint32_t sweepStartMs = _servoTestSweepStartMs.load();
            if (_servoTestCompassSweep.addSample(_compassPanRawX, _compassPanRawY,
                _servoTestCompassRawZ.load(), sampleAtMs - sweepStartMs))
                _servoTestSweepSamples.fetch_add(1);
            else
            {
                sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
                _servoTestSweepState.store(sweepState);
                _servoTestSweepFaultReason.store(
                    static_cast<uint8_t>(ServoTestSweepFaultReason::SampleBufferFull));
                _servoTestRequestedPulseUs.store(neutralUs);
                _servoTestLastCommandMs.store(now);
            }
        }
        if (AatServo::shouldStartSweepOnReferencePress(
            sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed),
            _compassPanReferencePressed))
        {
            _servoTestSweepStartMs.store(now);
            sweepState = static_cast<uint8_t>(ServoTestSweepState::Sweeping);
            _servoTestSweepState.store(sweepState);
        }
        if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
            || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping))
        {
            const uint32_t elapsedMs = now - _servoTestSweepStartMs.load();
            const uint32_t sampleAgeMs = now - _servoTestCompassSampleAtMs.load();
            if (AatServo::isSweepTimedOut(
                sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping),
                elapsedMs, SERVO_TEST_SWEEP_TIMEOUT_MS))
            {
                _servoTestSweepFaultReason.store(
                    static_cast<uint8_t>(ServoTestSweepFaultReason::Timeout));
                sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
            }
            else if (_servoTestCompassSampleCount.load() == 0)
            {
                _servoTestSweepFaultReason.store(
                    static_cast<uint8_t>(ServoTestSweepFaultReason::NoCompassSamples));
                sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
            }
            else if (sampleAgeMs > 500U)
            {
                _servoTestSweepFaultReason.store(
                    static_cast<uint8_t>(ServoTestSweepFaultReason::StaleCompass));
                sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
            }
            else if (AatServo::shouldCompleteSweepOnReferenceRelease(
                sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping),
                _compassPanReferencePressed))
            {
                sweepState = static_cast<uint8_t>(ServoTestSweepState::Complete);
            }

            if (sweepState != static_cast<uint8_t>(ServoTestSweepState::Armed)
                && sweepState != static_cast<uint8_t>(ServoTestSweepState::Sweeping))
            {
                if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Complete))
                {
                    AatCompassCalibration::Result result = {};
                    const bool fitValid = _servoTestCompassSweep.fit(&result, true);
                    _servoTestCompassCalibrationResult = result;
                    _servoTestFitAttempted.store(true);
                    if (fitValid)
                    {
                        _servoTestCalibrationReady.store(true);
                        if (_servoTestSiteSweep.load())
                        {
                            aat_pan_compass_calibration_t saved = config.GetAatPanCompassCalibration();
                            if (AatCompassCalibration::refreshLocalOffset(saved, result)
                                && config.SetAatPanCompassCalibration(saved))
                            {
                                config.Commit();
                                _servoTestCalibrationSaved.store(true);
                            }
                            else
                                sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
                        }
                    }
                    else
                        sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
                }
                _servoTestSweepState.store(sweepState);
                _servoTestRequestedPulseUs.store(neutralUs);
                _servoTestLastCommandMs.store(now);
            }
        }

        sweepState = _servoTestSweepState.load();
        const bool sweepActive = sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
            || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping);
        const bool sweepPulseActive = sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping);
        const uint32_t sampleAgeMs = now - _servoTestCompassSampleAtMs.load();
        const bool calibratedCompassFresh = _servoTestCompassDetected.load()
            && _servoTestCompassCalibratedHeadingValid.load()
            && _servoTestCompassSampleCount.load() > 0
            && sampleAgeMs <= 500U
            && AatServo::isCompassConfigurationValid(_compassPanConfigReadValid,
                _compassPanConfigA, COMPASS_PAN_CONFIG_A)
            && AatCompass::isCompassSampleReleaseConfirmed(
                _compassPanStatusAfterTriggerValid, _compassPanStatusAfterTrigger);
        const uint32_t sampleSequence = _servoTestCompassSampleCount.load();
        const bool pulseCalibrationStopRequested =
            _servoTestPulseCalibrationStopRequested.exchange(false);
        if (pulseCalibrationStopRequested)
        {
            _servoTestPulseCalibrationStartRequested.store(false);
            _servoTestPulseCalibration.cancel();
        }
        const bool pulseCalibrationStartRequested =
            _servoTestPulseCalibrationStartRequested.exchange(false);
        if (pulseCalibrationStartRequested)
            _servoTestPulseCalibration.start(now, neutralUs,
                config.GetAatPanLeftMaxOffset(), config.GetAatPanRightMaxOffset(),
                sampleSequence);
        const bool pulseCalibrationWasActive = _servoTestPulseCalibration.active();
        uint16_t calibrationPulseUs = neutralUs;
        if (pulseCalibrationWasActive)
        {
            AatServo::PanCommand currentPan = {};
            const int32_t heading = _servoTestCompassCalibratedHeadingDegrees.load();
            const bool safetyReady = !sweepActive && calibratedCompassFresh
                && config.GetAatPanCompassCalibration().valid == 1
                && config.GetAatPanCompassNorthAligned()
                && !_compassPanReferencePressed && !rawReferencePressed
                && _servoTestPanSafety.calculateCommand(
                    calibratedCompassFresh, heading, heading, &currentPan);
            calibrationPulseUs = _servoTestPulseCalibration.update(now, safetyReady,
                currentPan.currentPanDegrees, _compassPanAppliedPulseUs,
                calibratedCompassFresh, sampleSequence);
            _servoTestRequestedPulseUs.store(neutralUs);
            _servoTestLastCommandMs.store(now);
        }
        const bool pulseCalibrationActiveNow = _servoTestPulseCalibration.active();
        const bool pulseCalibrationComplete = pulseCalibrationWasActive
            && _servoTestPulseCalibration.state() == AatServo::PulseCalibration::State::Complete;
        if (sweepActive || (config.IsAatPanCompassReversed() && !config.GetAatPanCompassNorthAligned()))
        {
            _servoTestPanSafety.invalidate();
            _servoTestManualPulseAllowed.store(false);
        }
        else if (!pulseCalibrationActiveNow)
        {
            _servoTestPanSafety.updateReference(_compassPanReferencePressed,
                _compassPanAppliedPulseUs == neutralUs, calibratedCompassFresh,
                _servoTestCompassCalibratedHeadingDegrees.load());
            AatServo::PanCommand currentPan = {};
            const int32_t heading = _servoTestCompassCalibratedHeadingDegrees.load();
            const bool withinTravel = _servoTestPanSafety.calculateCommand(
                calibratedCompassFresh, heading, heading, &currentPan);
            const bool pulseAllowed = withinTravel
                && currentPan.currentPanDegrees >= -AatServoTest::MANUAL_PULSE_MAX_PAN_DEGREES
                && currentPan.currentPanDegrees <= AatServoTest::MANUAL_PULSE_MAX_PAN_DEGREES
                && !_compassPanReferencePressed;
            _servoTestManualPulseAllowed.store(pulseAllowed);
            if (!pulseAllowed)
            {
                _servoTestRequestedPulseUs.store(neutralUs);
                _servoTestLastCommandMs.store(now);
            }
        }

        _servoTestPanSafetyState.store(static_cast<uint8_t>(_servoTestPanSafety.state()));
        const uint16_t pulseRequestUs = sweepActive
            ? AatServo::getCompassSetupSweepPulse(sweepPulseActive,
                SERVO_TEST_SWEEP_PULSE_US, neutralUs)
            : pulseCalibrationActiveNow
                ? calibrationPulseUs
                : _servoTestRequestedPulseUs.load();
        const uint16_t requestedPulseUs = AatServo::getServoTestPulse(sweepPulseActive,
            pulseRequestUs, neutralUs, now, _servoTestLastCommandMs.load(),
            AatServoTest::MANUAL_PULSE_TIMEOUT_MS);
        if (requestedPulseUs != _compassPanAppliedPulseUs)
        {
            _servo_Azim.writeMicroseconds(requestedPulseUs);
            _compassPanAppliedPulseUs = requestedPulseUs;
            _servoTestCalibratedRateWindow.reset();
            _servoTestCompassCalibratedRateValid.store(false);
        }
        if (!sweepActive && requestedPulseUs == neutralUs)
            AatServo::clearServoTestPulseRequestIfUnchanged(
                _servoTestRequestedPulseUs, pulseRequestUs, neutralUs);

        _servoTestAppliedPulseUs.store(_compassPanAppliedPulseUs);
        if (pulseCalibrationComplete)
        {
            if (config.SetAatPanPulseCalibration(
                _servoTestPulseCalibration.leftStartUs(), _servoTestPulseCalibration.rightStartUs()))
                config.Commit();
            else
                _servoTestPulseCalibration.fail(AatServo::PulseCalibration::Fault::Configuration);
        }
        _servoTestPulseCalibrationState.store(static_cast<uint8_t>(_servoTestPulseCalibration.state()));
        _servoTestPulseCalibrationCandidateUs.store(_servoTestPulseCalibration.candidateUs());
        _servoTestPulseCalibrationSamples.store(_servoTestPulseCalibration.samples());
        _servoTestPulseCalibrationDisplacementMdeg.store(
            _servoTestPulseCalibration.displacementMilliDegrees());
        _servoTestPulseCalibrationFault.store(static_cast<uint8_t>(_servoTestPulseCalibration.fault()));
        _servoTestPulseCalibrationSettling.store(_servoTestPulseCalibration.settling());
        if ((pulseCalibrationWasActive || pulseCalibrationStopRequested
                || pulseCalibrationStartRequested)
            && !_servoTestPulseCalibration.active()
            && !_servoTestPulseCalibrationStartRequested.load()
            && _compassPanAppliedPulseUs == neutralUs)
            _servoTestPulseCalibrationBusy.store(false);
        _compassPanTelemetryModeActive.store(true);
        _compassPanTelemetrySetupActive.store(true);
        _compassPanTelemetrySafetyState.store(static_cast<uint8_t>(_servoTestPanSafety.state()));
        _compassPanTelemetryBlockReason.store(
            static_cast<uint8_t>(AatServo::ContinuousPanBlockReason::Setup));
        _compassPanTelemetryReferencePressed.store(_compassPanReferencePressed);
        _compassPanTelemetryCompassDetected.store(_compassPanDetected);
        _compassPanTelemetrySampleValid.store(false);
        _compassPanTelemetryCompassFresh.store(false);
        _compassPanTelemetryConfigMatchesRequested.store(AatServo::isCompassConfigurationValid(
            _compassPanConfigReadValid, _compassPanConfigA, COMPASS_PAN_CONFIG_A));
        _compassPanTelemetryConfigA.store(_compassPanConfigA);
        _compassPanTelemetryTriggerStatusValid.store(_compassPanStatusAfterTriggerValid);
        _compassPanTelemetryTriggerLockClear.store(AatCompass::isCompassSampleReleaseConfirmed(
            _compassPanStatusAfterTriggerValid, _compassPanStatusAfterTrigger));
        _compassPanTelemetryNorthAligned.store(config.GetAatPanCompassNorthAligned());
        _compassPanTelemetryStartupComplete.store(false);
        _compassPanTelemetryGpsFresh.store(false);
        _compassPanTelemetryPanCommandValid.store(false);
        _compassPanTelemetryRequestedPulseUs.store(requestedPulseUs);
        _compassPanTelemetryAppliedPulseUs.store(_compassPanAppliedPulseUs);
#if defined(AAT_COMPASS_TRACKING_TRACE)
        static uint32_t lastSetupTraceMs = 0;
        if (now - lastSetupTraceMs >= 100U)
        {
            lastSetupTraceMs = now;
            const uint32_t traceAtMs = millis();
            const uint32_t commandAgeMs = traceAtMs - _servoTestLastCommandMs.load();
            const uint32_t sampleAgeMs = traceAtMs - _servoTestCompassSampleAtMs.load();
            const bool configMatches = AatServo::isCompassConfigurationValid(
                _compassPanConfigReadValid, _compassPanConfigA, COMPASS_PAN_CONFIG_A);
            const bool triggerLockClear = AatCompass::isCompassSampleReleaseConfirmed(
                _compassPanStatusAfterTriggerValid, _compassPanStatusAfterTrigger);
            Serial.printf("PAN_SETUP ms=%lu work_ms=%lu command_age_ms=%lu sample_age_ms=%lu sweep=%u pulse_allowed=%u rate_valid=%u rate_mdeg_s=%ld rate_sequence=%lu rate_state=%u trigger_valid=%u trigger_lock_clear=%u config_match=%u reference_pressed=%u requested_us=%u effective_us=%u applied_us=%u library_us=%d\n",
                static_cast<unsigned long>(traceAtMs),
                static_cast<unsigned long>(traceAtMs - updateStartedAtMs),
                static_cast<unsigned long>(commandAgeMs),
                static_cast<unsigned long>(sampleAgeMs),
                static_cast<unsigned>(sweepState),
                _servoTestManualPulseAllowed.load(),
                _servoTestCompassCalibratedRateValid.load(),
                static_cast<long>(_servoTestCompassCalibratedRateMilliDegreesPerSecond.load()),
                static_cast<unsigned long>(_servoTestCompassCalibratedRateSequence.load()),
                setupRateTraceState,
                _compassPanStatusAfterTriggerValid, triggerLockClear, configMatches,
                _compassPanReferencePressed, pulseRequestUs, requestedPulseUs,
                _compassPanAppliedPulseUs, _servo_Azim.readMicroseconds());
        }
#endif
        return;
    }

    const bool compassConfigMatchesRequested = AatServo::isCompassConfigurationValid(
        _compassPanConfigReadValid, _compassPanConfigA, COMPASS_PAN_CONFIG_A);
    const bool compassTriggerLockClear = AatCompass::isCompassSampleReleaseConfirmed(
        _compassPanStatusAfterTriggerValid, _compassPanStatusAfterTrigger);
    const bool compassSampleFresh = AatServo::areCompassPanSampleInputsValid(
        _compassPanDetected, _compassPanHeadingValid,
        AatServo::isPanFeedbackFresh(_compassPanSampleAtMs, now,
            COMPASS_PAN_SAMPLE_TIMEOUT_MS),
        config.GetAatPanCompassCalibration().valid == 1,
        compassConfigMatchesRequested, _compassPanStatusAfterTriggerValid,
        compassTriggerLockClear);
    const bool actuatorNeutral = _compassPanAppliedPulseUs == neutralUs;
    if (_compassPanSafety.state() == AatServo::PanSafetyState::AwaitButtonRelease
        && !_compassPanReferencePressed)
        _compassPanSafety.updateReference(false, actuatorNeutral,
            compassSampleFresh, _compassPanHeadingDegrees);

    const bool alignmentButtonPressed = _compassPanReferencePressed
        && _compassPanSafety.state() != AatServo::PanSafetyState::AwaitButtonRelease;
    const bool northAligned = config.GetAatPanCompassNorthAligned();
    const AatServo::CompassStartupAlignmentAction alignmentAction =
        AatServo::getCompassStartupAlignmentAction(
            _compassPanStartupAlignmentComplete, northAligned, compassSampleFresh,
            alignmentButtonPressed);
    if (alignmentAction == AatServo::CompassStartupAlignmentAction::CaptureNorthAndHome
        && saveCompassPanNorthAlignment(_compassPanRawX, _compassPanRawY, 0, true))
    {
        _compassPanStartupAlignmentComplete = calculateCompassPanHeadingDegrees(
            _compassPanRawX, _compassPanRawY, &_compassPanHeadingDegrees);
        _compassPanSampleAtMs = now;
    }
    else if (alignmentAction == AatServo::CompassStartupAlignmentAction::CaptureHome)
    {
        _compassPanStartupAlignmentComplete = calculateCompassPanHeadingDegrees(
            _compassPanRawX, _compassPanRawY, &_compassPanHeadingDegrees);
        _compassPanSampleAtMs = now;
    }

    bool compassFreshAndValid = compassSampleFresh
        && _compassPanStartupAlignmentComplete
        && config.GetAatPanCompassNorthAligned();
    _compassPanSafety.updateReference(_compassPanReferencePressed,
        actuatorNeutral, compassFreshAndValid, _compassPanHeadingDegrees);

#if defined(PIN_OLED_SDA)
    if (!_compassPanStartupAlignmentComplete)
    {
        const uint8_t promptState = !compassSampleFresh ? 0xfe
            : northAligned ? 0xfc : 0xfb;
        if (promptState != _compassPanLastPromptState)
        {
            displayCompassPanStartup(compassSampleFresh, northAligned, false);
            _compassPanLastPromptState = promptState;
        }
    }
#endif

    now = millis();
    if (compassFreshAndValid && !AatServo::isPanFeedbackFresh(
        _compassPanSampleAtMs, now, COMPASS_PAN_SAMPLE_TIMEOUT_MS))
    {
        compassFreshAndValid = false;
        _compassPanSafety.invalidate();
    }
    bool trackingHeadingReady = false;
    if (compassFreshAndValid
        && _compassPanSafety.state() == AatServo::PanSafetyState::Referenced)
    {
        // Home-relative pan avoids the north wrap; safety continues to use raw heading.
        const int32_t rawPan = AatServo::shortestPathDelta(
            _compassPanSafety.reference().homeHeadingDegrees, _compassPanHeadingDegrees);
        trackingHeadingReady = _compassPanTrackingMedian.update(_compassPanSampleAtMs, rawPan);
    }
    else
        _compassPanTrackingMedian.reset();
    uint16_t requestedPulseUs = neutralUs;
    const bool gpsFresh = AatServo::isContinuousPanGpsFresh(isHomeSet(),
        _gpsAvgUpdateIntervalMs, now - _gpsLast.lastUpdateMs);
    const AatServo::ContinuousPanBlockReason blockReason =
        AatServo::getContinuousPanBlockReason(false,
            _compassPanStartupAlignmentComplete, compassFreshAndValid, gpsFresh,
            _compassPanSafety.state());
    int32_t targetHeading = 0;
    AatServo::PanCommand pulseCommand = {};
    AatServo::PanCommand guardedCommand = {};
    bool panCommandValid = false;
    if (blockReason == AatServo::ContinuousPanBlockReason::None
        && AatServo::areContinuousPanCommandInputsValid(gpsFresh, compassFreshAndValid,
            _compassPanSafety.state() == AatServo::PanSafetyState::Referenced))
    {
        targetHeading = calcProjectedAzim(now);
        if (_compassPanSafety.calculateCommand(compassFreshAndValid,
            _compassPanHeadingDegrees, targetHeading, &guardedCommand))
        {
            panCommandValid = trackingHeadingReady && config.GetAatPanPulseCalibrationValid()
                && AatServo::calculateContinuousPanPulseProfile(
                _compassPanSafety.reference(),
                compassFreshAndValid,
                _compassPanSafety.reference().homeHeadingDegrees
                    + _compassPanTrackingMedian.panDegrees(), targetHeading,
                neutralUs, config.GetAatPanLeftStartPulse(),
                config.GetAatPanLeftMaxPulse(),
                config.GetAatPanRightStartPulse(),
                config.GetAatPanRightMaxPulse(),
                config.GetAatPanDeadbandDegrees(), config.GetAatServoSmooth(),
                &requestedPulseUs, &pulseCommand);
            if (!panCommandValid)
                requestedPulseUs = neutralUs;
            else
                // Filter lag must not drive against the raw error or through its deadband.
                requestedPulseUs = AatServo::constrainFilteredPanPulse(requestedPulseUs,
                    neutralUs, guardedCommand.errorDegrees, config.GetAatPanDeadbandDegrees());
        }
    }

    if (requestedPulseUs != neutralUs)
    {
        if (requestedPulseUs != _compassPanAppliedPulseUs)
        {
            _servo_Azim.writeMicroseconds(requestedPulseUs);
            _compassPanAppliedPulseUs = requestedPulseUs;
        }
    }
    else if (_compassPanAppliedPulseUs != neutralUs)
    {
        _servo_Azim.writeMicroseconds(neutralUs);
        _compassPanAppliedPulseUs = neutralUs;
    }

    _compassPanTelemetryModeActive.store(_compassPanModeActive);
    _compassPanTelemetrySetupActive.store(_compassPanSetupMode.load());
    _compassPanTelemetrySafetyState.store(static_cast<uint8_t>(_compassPanSafety.state()));
    _compassPanTelemetryBlockReason.store(static_cast<uint8_t>(blockReason));
    _compassPanTelemetryReferencePressed.store(_compassPanReferencePressed);
    _compassPanTelemetryCompassDetected.store(_compassPanDetected);
    _compassPanTelemetrySampleValid.store(compassSampleFresh);
    _compassPanTelemetryCompassFresh.store(compassFreshAndValid);
    _compassPanTelemetryConfigMatchesRequested.store(compassConfigMatchesRequested);
    _compassPanTelemetryConfigA.store(_compassPanConfigA);
    _compassPanTelemetryTriggerStatusValid.store(_compassPanStatusAfterTriggerValid);
    _compassPanTelemetryTriggerLockClear.store(compassTriggerLockClear);
    _compassPanTelemetryHeadingDegrees.store(_compassPanHeadingDegrees);
    _compassPanTelemetrySampleAtMs.store(_compassPanSampleAtMs);
    _compassPanTelemetryNorthAligned.store(config.GetAatPanCompassNorthAligned());
    _compassPanTelemetryStartupComplete.store(_compassPanStartupAlignmentComplete);
    _compassPanTelemetryGpsFresh.store(gpsFresh);
    _compassPanTelemetryTargetHeadingDegrees.store(targetHeading);
    _compassPanTelemetryPanCommandValid.store(panCommandValid);
    _compassPanTelemetryCurrentPanDegrees.store(panCommandValid ? guardedCommand.currentPanDegrees : 0);
    _compassPanTelemetryTargetPanDegrees.store(panCommandValid ? guardedCommand.targetPanDegrees : 0);
    _compassPanTelemetryErrorDegrees.store(panCommandValid ? guardedCommand.errorDegrees : 0);
    _compassPanTelemetryRequestedPulseUs.store(requestedPulseUs);
    _compassPanTelemetryAppliedPulseUs.store(_compassPanAppliedPulseUs);
#if defined(AAT_COMPASS_TRACKING_TRACE)
    static uint32_t lastTraceMs = 0;
    if (now - lastTraceMs >= 100U)
    {
        lastTraceMs = now;
        Serial.printf("PAN ms=%lu work_ms=%lu sample_age_ms=%lu gps_age_ms=%lu state=%u detected=%u heading_valid=%u sample_gate=%u compass_fresh=%u startup_complete=%u north_aligned=%u gps_fresh=%u reference_pressed=%u raw_x=%d raw_y=%d raw_z=%d status=%02x post_status=%02x post_valid=%u config_valid=%u config_a=%02x config_match=%u trigger_valid=%u trigger_status=%02x trigger_lock_clear=%u heading=%ld target=%ld current_pan=%ld target_pan=%ld error=%ld filter_ready=%u control_pan=%ld control_error=%ld command_valid=%u requested_us=%u applied_us=%u library_us=%d deadband_deg=%u\n",
            static_cast<unsigned long>(now),
            static_cast<unsigned long>(millis() - updateStartedAtMs),
            static_cast<unsigned long>(now - _compassPanSampleAtMs),
            static_cast<unsigned long>(now - _gpsLast.lastUpdateMs),
            static_cast<unsigned>(_compassPanSafety.state()),
            _compassPanDetected, _compassPanHeadingValid, compassSampleFresh,
            compassFreshAndValid, _compassPanStartupAlignmentComplete,
            config.GetAatPanCompassNorthAligned(), gpsFresh, _compassPanReferencePressed,
            _compassPanRawX, _compassPanRawY, _compassPanRawZ,
            _compassPanSampleStatus, _compassPanSampleStatusAfterRead,
            _compassPanSampleStatusAfterReadValid, _compassPanConfigReadValid,
            _compassPanConfigA, compassConfigMatchesRequested,
            _compassPanStatusAfterTriggerValid, _compassPanStatusAfterTrigger,
            compassTriggerLockClear,
            static_cast<long>(_compassPanHeadingDegrees), static_cast<long>(targetHeading),
            static_cast<long>(panCommandValid ? guardedCommand.currentPanDegrees : 0),
            static_cast<long>(panCommandValid ? guardedCommand.targetPanDegrees : 0),
            static_cast<long>(panCommandValid ? guardedCommand.errorDegrees : 0),
            trackingHeadingReady,
            static_cast<long>(pulseCommand.currentPanDegrees),
            static_cast<long>(pulseCommand.errorDegrees), panCommandValid,
            requestedPulseUs, _compassPanAppliedPulseUs, _servo_Azim.readMicroseconds(),
            config.GetAatPanDeadbandDegrees());
    }
#endif
}

AatModule::CompassPanDiagnostics AatModule::getCompassPanDiagnostics(uint32_t now) const
{
    CompassPanDiagnostics status = {};
    status.modeActive = _compassPanTelemetryModeActive.load();
    status.setupActive = _compassPanTelemetrySetupActive.load();
    status.safetyState = _compassPanTelemetrySafetyState.load();
    status.blockReason = _compassPanTelemetryBlockReason.load();
    status.referencePressed = _compassPanTelemetryReferencePressed.load();
    status.compassDetected = _compassPanTelemetryCompassDetected.load();
    status.compassSampleValid = _compassPanTelemetrySampleValid.load();
    status.compassFresh = _compassPanTelemetryCompassFresh.load();
    status.deadbandDegrees = config.GetAatPanDeadbandDegrees();
    status.configMatchesRequested = _compassPanTelemetryConfigMatchesRequested.load();
    status.configA = _compassPanTelemetryConfigA.load();
    status.triggerStatusValid = _compassPanTelemetryTriggerStatusValid.load();
    status.triggerLockClear = _compassPanTelemetryTriggerLockClear.load();
    status.headingDegrees = _compassPanTelemetryHeadingDegrees.load();
    status.sampleAgeMs = now - _compassPanTelemetrySampleAtMs.load();
    status.northAligned = _compassPanTelemetryNorthAligned.load();
    status.startupAlignmentComplete = _compassPanTelemetryStartupComplete.load();
    status.gpsFresh = _compassPanTelemetryGpsFresh.load();
    status.targetHeadingDegrees = _compassPanTelemetryTargetHeadingDegrees.load();
    status.panCommandValid = _compassPanTelemetryPanCommandValid.load();
    status.currentPanDegrees = _compassPanTelemetryCurrentPanDegrees.load();
    status.targetPanDegrees = _compassPanTelemetryTargetPanDegrees.load();
    status.errorDegrees = _compassPanTelemetryErrorDegrees.load();
    status.requestedPulseUs = _compassPanTelemetryRequestedPulseUs.load();
    status.appliedPulseUs = _compassPanTelemetryAppliedPulseUs.load();
    return status;
}

bool AatModule::requestCompassPanSetup()
{
    if (!_compassPanTelemetryModeActive.load())
        return false;
    _compassPanSetupRequested.store(true);
    return true;
}
#endif

void AatModule::Init()
{
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
    _servoTestConfiguredNeutralUs = config.GetAatPanServoNeutral();
#endif
#if !defined(DEBUG_LOG)
    // Need to call _port's end but it is a stream reference not the HardwareSerial
    Serial.end();
#endif
    _servoPos[IDX_AZIM] = (config.GetAatServoLow(IDX_AZIM) + config.GetAatServoHigh(IDX_AZIM)) / 2;
#if defined(PIN_SERVO_AZIM)
#if defined(AAT_SERVO_TEST)
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    pinMode(PIN_AAT_REFERENCE_BUTTON, INPUT_PULLUP);
    _servoTestRawReferencePressed = digitalRead(PIN_AAT_REFERENCE_BUTTON) == LOW;
    _servoTestReferencePressed.store(_servoTestRawReferencePressed);
    _servoTestReferenceDebounceMs = millis();
    _servo_Azim.attach(PIN_SERVO_AZIM, 500, 2500);
    _servo_Azim.writeMicroseconds(neutralUs);
    _servoTestRequestedPulseUs.store(neutralUs);
    _servoTestAppliedPulseUs.store(neutralUs);
    _servoTestLastCommandMs.store(millis());
#elif defined(AAT_COMPASS_TRACKING)
#if defined(AAT_COMPASS_SETUP_TEST)
    _compassPanModeActive = true;
    _compassPanSetupMode.store(true);
#else
    _compassPanModeActive = config.GetAatServoMode() == AatServo::CompassContinuous;
    _compassPanSetupMode.store(AatServo::shouldEnterCompassPanSetup(
        _compassPanModeActive, config.GetAatPanCompassCalibration().valid == 1,
        config.GetAatPanPulseCalibrationValid(), connectionState == wifiUpdate));
#endif
    pinMode(PIN_AAT_REFERENCE_BUTTON, INPUT_PULLUP);
    _compassPanRawReferencePressed = digitalRead(PIN_AAT_REFERENCE_BUTTON) == LOW;
    _compassPanReferencePressed = _compassPanRawReferencePressed;
    _compassPanReferenceDebounceMs = millis();
    if (_compassPanModeActive)
    {
        const uint16_t neutralUs = config.GetAatPanServoNeutral();
        _servo_Azim.attach(PIN_SERVO_AZIM, 500, 2500);
        _servo_Azim.writeMicroseconds(neutralUs);
        _compassPanAppliedPulseUs = neutralUs;
    }
#if !defined(AAT_COMPASS_DEBUG)
    else
    {
        _servo_Azim.attach(PIN_SERVO_AZIM, 500, 2500);
        _servo_Azim.write(_servoPos[IDX_AZIM]);
    }
#endif
#elif !defined(AAT_COMPASS_DEBUG)
    _servo_Azim.attach(PIN_SERVO_AZIM, 500, 2500);
    _servo_Azim.write(_servoPos[IDX_AZIM]);
#endif
#endif
    _servoPos[IDX_ELEV] = (config.GetAatServoLow(IDX_ELEV) + config.GetAatServoHigh(IDX_ELEV)) / 2;
#if defined(PIN_SERVO_ELEV)
#if !defined(AAT_COMPASS_DEBUG) && !defined(AAT_SERVO_TEST)
    _servo_Elev.attach(PIN_SERVO_ELEV, 500, 2500);
    _servo_Elev.write(_servoPos[IDX_ELEV]);
#endif
#endif
#if defined(PIN_OLED_SDA) && !defined(AAT_SERVO_TEST_DISABLE_OLED)
    displayInit();
#if defined(AAT_SERVO_TEST)
    displayServoTest(config.GetAatPanServoNeutral(), false);
#endif
#if defined(AAT_COMPASS_TRACKING)
    if (_compassPanModeActive)
    {
        if (_compassPanSetupMode.load())
        {
            displayCompassPanSetup();
            _compassPanLastPromptState = 0xfc;
        }
        else
            displayCompassPanStartup(false, false, false);
    }
#endif
#elif defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING) || defined(AAT_I2C_TRACE)
#if defined(PIN_OLED_SDA)
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
#else
    Wire.begin();
#endif
#endif
    ModuleBase::Init();
}

void AatModule::SendGpsTelemetry(crsf_packet_gps_t *packet)
{
#if defined(AAT_PERF_DEBUG)
    recordGpsFrame();
#endif
    _gpsLast.lat = be32toh(packet->p.lat);
    _gpsLast.lon = be32toh(packet->p.lon);
    _gpsLast.speed = be16toh(packet->p.speed);
    _gpsLast.heading = be16toh(packet->p.heading);
    _gpsLast.altitude = (int32_t)be16toh(packet->p.altitude) - 1000;
    _gpsLast.satcnt = packet->p.satcnt;

    //DBGLN("GPS: (%d,%d) %dm %usats", _gpsLast.lat, _gpsLast.lon,
    //    _gpsLast.altitude, _gpsLast.satcnt);

    _gpsLast.updated = true;
}

void AatModule::updateGpsInterval(uint32_t interval)
{
    // Low pass filter. Note there is no fast init of the average, so it will take some time to grow
    // this prevents overprojection caused by the first update after setting home
    _gpsAvgUpdateIntervalMs += ((int32_t)interval - (int32_t)_gpsAvgUpdateIntervalMs) / 4;

    // Limit the maximum interval to provent projecting for too long
    const uint32_t GPS_UPDATE_INTERVAL_MAX = (10U * 1000U);
    if (_gpsAvgUpdateIntervalMs > GPS_UPDATE_INTERVAL_MAX)
        _gpsAvgUpdateIntervalMs = GPS_UPDATE_INTERVAL_MAX;
}

uint8_t AatModule::calcGpsIntervalPct(uint32_t now)
{
    if (_gpsAvgUpdateIntervalMs)
    {
        return constrain((now - _gpsLast.lastUpdateMs) * 100U / (uint32_t)_gpsAvgUpdateIntervalMs, 0U, 100U);
    }

    return 0;
}

void AatModule::processGps(uint32_t now)
{
    if (!_gpsLast.updated || _isOverrideMode)
        return;
    _gpsLast.updated = false;

    // Actually want to track time between _processing_ each GPS update
    uint32_t interval = now - _gpsLast.lastUpdateMs;
    _gpsLast.lastUpdateMs = now;

    // Check if need to set home position
    bool didSetHome = false;
    if (!isHomeSet())
    {
        if (_gpsLast.satcnt >= config.GetAatSatelliteHomeMin())
        {
            didSetHome = true;
            _home.lat = _gpsLast.lat;
            _home.lon = _gpsLast.lon;
            _home.alt = _gpsLast.altitude;
            DBGLN("GPS Home set to (%d,%d)", _home.lat, _home.lon);
        }
        else
            return;
    }

    uint32_t azimuth;
    uint32_t distance;
    AatNavigation::calculateDistanceAndAzimuth(_home.lat, _home.lon, _gpsLast.lat, _gpsLast.lon, &distance, &azimuth);
    uint8_t elevation = constrain(AatNavigation::calculateElevation(distance, _gpsLast.altitude - _home.alt), 0, 90);
    DBGLN("Azimuth: %udeg Elevation: %udeg Distance: %um", azimuth, elevation, distance);

    // Calculate angular velocity to allow dead reckoning projection
    if (!didSetHome)
    {
        updateGpsInterval(interval);
        // azimDelta is the azimuth change since the last packet, -180 to +180
        int32_t azimDelta = (azimuth - _targetAzim + 540) % 360 - 180;
        _azimMsPerDegree = (azimDelta == 0) ? 0 : (int32_t)interval / azimDelta;
        DBGLN("%d delta in %ums, %dms/d %uavg", azimDelta, interval, _azimMsPerDegree, _gpsAvgUpdateIntervalMs);
    }

    _targetDistance = distance;
    _targetElev = elevation;
    _targetAzim = azimuth;
}

int32_t AatModule::calcProjectedAzim(uint32_t now)
{
    return AatProjection::calculateProjectedAzimuth(config.GetAatProject(), _gpsAvgUpdateIntervalMs,
        now, _gpsLast.lastUpdateMs, _azimMsPerDegree, _targetDistance, _targetAzim);
}

/**
 * @brief: Calculate the bearing from centerdir to azim
 * @return: -180 to +179 bearing
*/
const int32_t AatModule::azimToBearing(int32_t azim) const
{
    int32_t center = 45 * config.GetAatCenterDir();
    return ((azim - center + 540) % 360) - 180;  // -180 to +179
}

#if defined(PIN_OLED_SDA)
void AatModule::displayInit()
{
    Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
    _display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    _display.setTextWrap(false);
    displayState();
}

#if defined(AAT_COMPASS_TRACKING)
void AatModule::displayCompassPanSetup()
{
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 0);
    if (connectionState == wifiUpdate)
        _display.write("WiFi update\nTracking disabled\nUse Backpack UI");
    else
        _display.write("Compass setup\nTracking disabled\nUse Backpack UI");
    _display.display();
}

void AatModule::displayCompassPanStartup(bool compassReady, bool northAligned,
    bool alignmentComplete)
{
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 0);
    if (!compassReady)
        _display.write("Compass waiting\nCheck axis fit");
    else if (alignmentComplete)
        _display.write("Compass ready\nPan home set");
    else if (northAligned)
        _display.write("Point pan home\nPress GPIO32");
    else
        _display.write("Point pan north\nPress GPIO32");
    _display.display();
}
#endif

void AatModule::displayState()
{
    // Boot screen with vbat, version, and bind/wifi state
    _display.clearDisplay();
    _display.setTextSize(2);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 0);
    if (connectionState == binding)
    {
        _display.write("Bind\nmode...\n\n");
    }
    else if (connectionState == wifiUpdate)
    {
        _display.write("Wifi\nmode...\n");
        _display.setTextSize(1);
        IPAddress localAddr;
        if (WiFi.getMode() == WIFI_STA)
            localAddr = WiFi.localIP();
        else
            localAddr = WiFi.softAPIP();
        _display.print(localAddr.toString());
        _display.write("\n\n");
    }
    else
        _display.write("AAT\nBackpack\n\n");

    _display.setTextSize(1);
    _display.print(VERSION);

    displayVBat();
    _display.display();
}

#if defined(AAT_SERVO_TEST)
void AatModule::displayServoTest(uint16_t pulseUs, bool active)
{
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 0);
    _display.print("AZIM SERVO TEST");
    _display.setTextSize(2);
    _display.setCursor(0, 18);
    _display.print(pulseUs);
    _display.print(" us");
    _display.setTextSize(1);
    _display.setCursor(0, 44);
    _display.print(active ? "PULSE COMMAND" : "NEUTRAL");
    _display.setCursor(0, 56);
    _display.print("Neutral via web UI");
    _display.display();
}
#endif

void AatModule::displayGpsIdle(uint32_t now)
{
    // A screen with just the GPS position, sat count, vbat, and interval bar
    _display.clearDisplay();
    _display.setCursor(0, 0);

    _display.setTextSize(2);
    _display.printf("Sats: %u\n",  _gpsLast.satcnt);

    _display.setTextSize(1);
    _display.printf("\nLat: %d.%07d\nLon: %d.%07d",
        _gpsLast.lat / 10000000, abs(_gpsLast.lat) % 10000000,
        _gpsLast.lon / 10000000, abs(_gpsLast.lon) % 10000000
    );

    displayGpsIntervalBar(now);
    displayVBat();
    _display.display();
}

void AatModule::displayAzimuthExtent(int32_t y)
{
    switch ((AatServo::Mode)config.GetAatServoMode())
    {
        // Just a line on the left / right of the azim line
        default: /* fallthrough */
        case AatServo::TwoToOne: /* fallthrough */
        case AatServo::Flip180:
            _display.drawFastVLine(0, y, 5, SSD1306_WHITE);
            _display.drawFastVLine(SCREEN_WIDTH-1, y, 5, SSD1306_WHITE);
            break;

        // 180 mode put the lines at the extent of the servo rotation
        case AatServo::Clip180:
            //_display.drawFastVLine(SCREEN_WIDTH/4, y, 5, SSD1306_WHITE);
            //_display.drawFastVLine(3*SCREEN_WIDTH/4, y, 5, SSD1306_WHITE);
            // Dotted line all the way up the elevation field
            for (int32_t dotY=0; dotY<(33+5); dotY+=4)
            {
                _display.drawPixel(SCREEN_WIDTH/4, dotY+16, SSD1306_WHITE);
                _display.drawPixel(3*SCREEN_WIDTH/4, dotY+16, SSD1306_WHITE);
            }
            break;
    }
}

void AatModule::displayAzimuth(int32_t projectedAzim)
{
    int32_t y = SCREEN_HEIGHT - FONT_H + 1 - 6;

    // horizon line + arrow + unprojected azim line
    // |-----/\---|-|
    _display.drawFastHLine(0, y+2, SCREEN_WIDTH-1, SSD1306_WHITE);
    displayAzimuthExtent(y);

    // Unprojected azimuth line
    int32_t azimPos = map(azimToBearing(_targetAzim), -180, 180, 0, SCREEN_WIDTH);
    _display.drawFastVLine(azimPos, y+1, 3, SSD1306_WHITE);
    // Projected Azimuth arrow
    azimPos = map(azimToBearing(projectedAzim), -180, 180, 0, SCREEN_WIDTH);
    const uint8_t oledim_arrowup[] = { 0x10, 0x38, 0x7c }; // 'arrowup', 7x3px
    _display.drawBitmap(azimPos-7/2, y+1, oledim_arrowup, 7, 3, SSD1306_WHITE, SSD1306_BLACK);

    // S    W    N    E    S under that
    y += 6;
    const char AZIM_LABELS[] = "SWNES" "WNESW" "NESWN" "ESWNE"; // 5 characters for each direction, 3rd character = center
    const char *labels = &AZIM_LABELS[config.GetAatCenterDir()*5/2];
    _display.drawChar(0, y, labels[0], SSD1306_WHITE, SSD1306_BLACK, 1);
    _display.drawChar(SCREEN_WIDTH/4-FONT_W/2, y, labels[1], SSD1306_WHITE, SSD1306_BLACK, 1);
    _display.drawChar(SCREEN_WIDTH/2-FONT_W/2+1, y, labels[2], SSD1306_WHITE, SSD1306_BLACK, 1);
    _display.drawChar(3*SCREEN_WIDTH/4-FONT_W/2, y, labels[3], SSD1306_WHITE, SSD1306_BLACK, 1);
    _display.drawChar(SCREEN_WIDTH-FONT_W+1, y, labels[4], SSD1306_WHITE, SSD1306_BLACK, 1);

    // Projected azim value as 3 digit number on a white background (max 2px each side)
    y -= 2;
    uint32_t azimStart = constrain(azimPos-((3*FONT_W)/2), -2, SCREEN_WIDTH-(3*FONT_W)-2);
    _display.fillRoundRect(azimStart, y, 3*FONT_W + 3, FONT_H + 1, 2, SSD1306_WHITE);
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_BLACK);
    _display.setCursor(azimStart+2, y+1);
    _display.printf("%03u", projectedAzim);
}

void AatModule::displayAltitude(int32_t azimPos, int32_t elevPos)
{
    int alt = constrain(_gpsLast.altitude - _home.alt, -99, 999);
    if (alt == 0)
        return;

    char buf[16];
    snprintf(buf, sizeof(buf), "%dm", alt);

    // If elevation is low, put alt over the pip, if elev high put the alt below it
    // and allow the units to go off the screen to the right if needed
    int32_t strWidth = strlen(buf) * FONT_W;
    int32_t distX = constrain(azimPos - (strWidth/2) + 1, 0, SCREEN_WIDTH + FONT_W - strWidth);
    int32_t distY = (_targetElev < 50) ? elevPos - FONT_H - 5 : elevPos + 6;
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(distX, distY);
    _display.write(buf);
}

void AatModule::displayTargetCircle(int32_t projectedAzim)
{
    const int32_t elevH = 33;
    // yellow-blue OLED have 16 pixels of yellow, start below that
    const int32_t elevOff = 16;
    // Dotted line to separate top of screen from tracking area
    for (int32_t x=0; x<SCREEN_WIDTH; x+=3)
        _display.drawPixel(x, elevOff, SSD1306_WHITE);

    int32_t elevPos = map(_targetElev, 0, 90, elevH, 0) + elevOff;
    //elevPos = 0 + elevOff;
    // X for projectedAzim
    int32_t azimPos = map(azimToBearing(projectedAzim), -180, 180, 0, SCREEN_WIDTH);
    _display.drawPixel(azimPos-1, elevPos-1, SSD1306_WHITE);
    _display.drawPixel(azimPos+1, elevPos-1, SSD1306_WHITE);
    _display.drawPixel(azimPos, elevPos, SSD1306_WHITE);
    _display.drawPixel(azimPos-1, elevPos+1, SSD1306_WHITE);
    _display.drawPixel(azimPos+1, elevPos+1, SSD1306_WHITE);

    // circle/rectangle cage for _servoPos
    if (!config.GetAatServoEndpointsValid())
    {
        displayAltitude(azimPos, elevPos);
        return;
    }
    int32_t servoXMin = ((AatServo::Mode)config.GetAatServoMode() == AatServo::Clip180) ? (SCREEN_WIDTH/4) : 0;
    int32_t servoXMax = ((AatServo::Mode)config.GetAatServoMode() == AatServo::Clip180) ? (3*SCREEN_WIDTH/4) : SCREEN_WIDTH;
    int32_t servoX = map(_servoPos[IDX_AZIM],
        config.GetAatServoLow(IDX_AZIM), config.GetAatServoHigh(IDX_AZIM),
        servoXMin, servoXMax);
#if defined(AAT_COMPASS_TRACKING)
    if (_compassPanModeActive)
        servoX = map(azimToBearing(_compassPanHeadingDegrees), -180, 180, 0, SCREEN_WIDTH);
#endif
    int32_t servoY = map(_servoPos[IDX_ELEV],
        config.GetAatServoLow(IDX_ELEV), config.GetAatServoHigh(IDX_ELEV),
        elevH, 0) + elevOff;
    //servoY = 0 + elevOff
    _display.drawFastHLine(servoX-1, servoY-3, 3, SSD1306_WHITE);
    _display.drawFastHLine(servoX-1, servoY+3, 3, SSD1306_WHITE);
    _display.drawFastVLine(servoX-3, servoY-1, 3, SSD1306_WHITE);
    _display.drawFastVLine(servoX+3, servoY-1, 3, SSD1306_WHITE);

    displayAltitude(azimPos, elevPos);
}

void AatModule::displayTargetDistance()
{
    // Target distance over the X: 9m, 99m, 999m, 9.9k, 99.9k, 999k
    char dist[16];
    const char *units;
    if (_targetDistance > 99999) // >99.99m = "xxx.x" (5 chars)
    {
        snprintf(dist, sizeof(dist), "%u.%u", _targetDistance / 1000, (_targetDistance % 1000) / 100);
        units = "km";
    }
    else if (_targetDistance > 999) // >999m = "[x]x.xx" (4-5 chars)
    {
        snprintf(dist, sizeof(dist), "%u.%02u", _targetDistance / 1000, (_targetDistance % 1000) / 10);
        units = "km";
    }
    else // "[x]xx" (2-3 chars)
    {
        snprintf(dist, sizeof(dist), "%2u", _targetDistance);
        units = "m";
    }
    _display.setTextColor(SSD1306_WHITE);
    _display.setTextSize(1);
    _display.setCursor(0, 0);
    _display.write("Dst");
    _display.setTextSize(2);
    _display.setCursor(3 * FONT_W + 3, 0);
    _display.printf(dist);

    _display.setTextSize(1);
    _display.setCursor(_display.getCursorX() + 1, FONT_H);
    _display.write(units);
}

void AatModule::displayVBat()
{
    if (_vbat.value() == 0)
        return;

    _display.setTextColor(SSD1306_WHITE);
    _display.setTextSize(1);
    _display.setCursor(SCREEN_WIDTH - 5*FONT_W, FONT_H);
    _display.printf("%2d.%1dV", _vbat.value() / 10, _vbat.value() % 10);
}

void AatModule::displayActive(uint32_t now, int32_t projectedAzim)
{
    // Throttle the dislay update rate to <60Hz as the tracker runs 100Hz
    const uint32_t DISPLAY_UPDATE_MS = 16;
    if (now - _lastDisplayActiveMs < DISPLAY_UPDATE_MS)
        return;
    _lastDisplayActiveMs = now;

    _display.clearDisplay();

    displayGpsIntervalBar(now);
    displayAzimuth(projectedAzim);
    displayTargetCircle(projectedAzim);
    displayTargetDistance();
    displayVBat();

    _display.display();
}

void AatModule::displayGpsIntervalBar(uint32_t now)
{
    if (_gpsAvgUpdateIntervalMs == 0)
        return;

    if (now - _gpsLast.lastUpdateMs > (3*_gpsAvgUpdateIntervalMs))
    {
        // Inverse "OLD" in the center of where the update bar would be
        _display.fillRoundRect(80, 0, SCREEN_WIDTH-1-80, FONT_H-1, 2, SSD1306_WHITE);
        _display.setTextSize(1);
        _display.setTextColor(SSD1306_BLACK);
        _display.setCursor(104 - (3*FONT_W)/2, 0);
        _display.write("OLD");
    }
    else
    {
        // A depleting bar going from screen center to right
        uint8_t gpsIntervalPct = calcGpsIntervalPct(now);
        uint8_t pxWidth = (SCREEN_WIDTH-1-80) * (100U - gpsIntervalPct) / 100U;
        _display.fillRect(SCREEN_WIDTH-pxWidth, 0, pxWidth, 2, SSD1306_WHITE);
    }
}
#endif /* defined(PIN_OLED_SDA) */

void AatModule::servoApplyMode(int32_t azim, int32_t elev, int32_t newServoPos[])
{
    const int32_t servoLow[2] = {
        config.GetAatServoLow(IDX_AZIM),
        config.GetAatServoLow(IDX_ELEV)
    };
    const int32_t servoHigh[2] = {
        config.GetAatServoHigh(IDX_AZIM),
        config.GetAatServoHigh(IDX_ELEV)
    };
    AatServo::applyMode(azimToBearing(azim), elev,
        (AatServo::Mode)config.GetAatServoMode(), servoLow, servoHigh, newServoPos);
}

void AatModule::servoUpdate(uint32_t now)
{
    bool continuousPan = false;
#if defined(AAT_COMPASS_TRACKING)
    continuousPan = _compassPanModeActive;
    if ((config.GetAatServoMode() == AatServo::CompassContinuous) != continuousPan)
        return;
#endif
    uint32_t interval = now - _lastServoUpdateMs;
    if (interval < 10U)
        return;
    _lastServoUpdateMs = now;

    // If the servo endpoints aren't valid, all this math will divide by zero all over
    if (!config.GetAatServoEndpointsValid())
        return;

    int32_t projectedAzim = calcProjectedAzim(now);
    int32_t newServoPos[IDX_COUNT] = {};
    servoApplyMode(projectedAzim, _targetElev, newServoPos);

    for (uint32_t idx = continuousPan ? IDX_ELEV : IDX_AZIM; idx<IDX_COUNT; ++idx)
    {
        // Use smoothness to denote the maximum us per 10ms update
        const int32_t SMOOTHNESS_US_PER_STEP = (config.GetAatServoSmooth() < 3) ? 6 : 4;
        int32_t range = (config.GetAatServoHigh(idx) - config.GetAatServoLow(idx));
        int32_t diff = newServoPos[idx] - _servoPos[idx];
        int32_t maxDiff = (10 - config.GetAatServoSmooth()) * SMOOTHNESS_US_PER_STEP;
        // If the distance the servo needs to go is more than 80% away
        // jump immediately. otherwise smooth it
        if (config.GetAatAzimuthServoFastFlip() && idx == IDX_AZIM && (abs(diff) * 100 / range) > 80)
        {
            // Prevent the servo from flipping back and forth around the 180 point
            // by only allowing 1 flip ever Xms. Just keep pushing toward the limit
            const uint32_t AZIM_FLIP_MIN_DELAY = 2000U;
            if (now - _lastAzimFlipMs > AZIM_FLIP_MIN_DELAY)
            {
                _servoPos[idx] = newServoPos[idx];
                _lastAzimFlipMs = now;
            }
            else if (diff < 0)
                _servoPos[idx] = constrain(_servoPos[idx] + maxDiff, _servoPos[idx], config.GetAatServoHigh(idx));
            else
                _servoPos[idx] = constrain(_servoPos[idx] - maxDiff, config.GetAatServoLow(idx), _servoPos[idx]);
        }
        else
            _servoPos[idx] += constrain(diff, -maxDiff, maxDiff);
    }
    //DBGLN("t=%u pro=%d us=%d smoo=%d", _targetAzim, projectedAzim, newServoPos[IDX_AZIM], _servoPos[IDX_AZIM]);

#if defined(PIN_SERVO_AZIM)
    if (!continuousPan)
        _servo_Azim.writeMicroseconds(_servoPos[IDX_AZIM]);
#endif
#if defined(PIN_SERVO_ELEV)
    _servo_Elev.writeMicroseconds(_servoPos[IDX_ELEV]);
#endif
#if defined(AAT_PERF_DEBUG)
    recordPerfServoUpdate(micros());
#endif

#if defined(PIN_OLED_SDA)
    if (!continuousPan)
        displayActive(now, projectedAzim);
#endif
}

uint32_t AatModule::getVbat()
{
    return _vbat.value();
}

#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
bool AatModule::setServoTestPulse(uint16_t pulseUs)
{
#if defined(AAT_COMPASS_TRACKING)
    if (!_compassPanSetupMode.load())
        return false;
#endif
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    if (pulseUs < AatServoTest::MIN_US || pulseUs > AatServoTest::MAX_US)
        return false;

    const uint8_t sweepState = _servoTestSweepState.load();
    if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
        || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping)
        || isServoTestPulseCalibrationBusy())
        return false;

    if (pulseUs != neutralUs && !AatServo::isManualPanTestPulseAllowed(
        pulseUs, neutralUs,
        AatServoTest::manualPulseMinimum(neutralUs, config.GetAatPanLeftMaxOffset()),
        AatServoTest::manualPulseMaximum(neutralUs, config.GetAatPanRightMaxOffset()),
        _servoTestManualPulseAllowed.load(), _servoTestManualPulseAllowed.load(),
        0, AatServoTest::MANUAL_PULSE_MAX_PAN_DEGREES))
        return false;

    _servoTestLastCommandMs.store(millis());
    _servoTestRequestedPulseUs.store(pulseUs);
    return true;
}

bool AatModule::startServoTestSweep(bool siteSweep)
{
    if (isServoTestPulseCalibrationBusy())
        return false;
#if defined(AAT_COMPASS_TRACKING)
    if (!_compassPanSetupMode.load())
        return false;
#endif
    const uint8_t currentSweepState = _servoTestSweepState.load();
    if (currentSweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
        || currentSweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping))
        return false;
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    if (_servoTestRequestedPulseUs.load() != neutralUs
        || _servoTestAppliedPulseUs.load() != neutralUs)
        return false;
    if (_servoTestReferencePressed.load())
        return false;
    if (siteSweep && config.GetAatPanCompassCalibration().valid != 1)
        return false;

    const uint32_t now = millis();
    const ServoTestCompassData compassData = getServoTestCompassData();
    if (!compassData.detected || !compassData.headingValid
        || now - compassData.sampleAtMs > 500U
        || !AatServo::isCompassConfigurationValid(compassData.configReadValid,
            compassData.configA, AatServoTest::COMPASS_CONFIG_A)
        || !AatCompass::isCompassSampleReleaseConfirmed(
            compassData.sensorStatusAfterTriggerValid, compassData.sensorStatusAfterTrigger))
        return false;

    _servoTestSweepSamples.store(0);
    _servoTestCompassSweep.reset();
    _servoTestCompassCalibrationResult = {};
    _servoTestFitAttempted.store(false);
    _servoTestCalibrationReady.store(false);
    _servoTestCalibrationSaved.store(false);
    _servoTestSweepFaultReason.store(static_cast<uint8_t>(ServoTestSweepFaultReason::None));
    _servoTestSiteSweep.store(siteSweep);
    _servoTestSweepState.store(static_cast<uint8_t>(ServoTestSweepState::Armed));
    return true;
}

bool AatModule::startServoTestPulseCalibration()
{
#if !defined(AAT_COMPASS_TRACKING)
    return false;
#else
    if (!_compassPanSetupMode.load())
        return false;
    const uint8_t sweepState = _servoTestSweepState.load();
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
        || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping)
        || !_servoTestManualPulseAllowed.load()
        || _servoTestRequestedPulseUs.load() != neutralUs
        || _servoTestAppliedPulseUs.load() != neutralUs
        || config.GetAatPanCompassCalibration().valid != 1
        || !config.GetAatPanCompassNorthAligned()
        || !AatConfigValidation::isPanPulseEnvelopeValid(neutralUs,
            config.GetAatPanLeftMaxOffset(), config.GetAatPanRightMaxOffset()))
        return false;

    bool expected = false;
    if (!_servoTestPulseCalibrationBusy.compare_exchange_strong(expected, true))
        return false;
    _servoTestPulseCalibrationStartRequested.store(true);
    return true;
#endif
}

bool AatModule::canConfigureServoTestPulseEnvelope()
{
    const uint8_t sweepState = _servoTestSweepState.load();
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    return !isServoTestPulseCalibrationBusy()
        && sweepState != static_cast<uint8_t>(ServoTestSweepState::Armed)
        && sweepState != static_cast<uint8_t>(ServoTestSweepState::Sweeping)
        && _servoTestRequestedPulseUs.load() == neutralUs
        && _servoTestAppliedPulseUs.load() == neutralUs
        && (!_servo_Azim.attached() || isServoTestSetupActive());
}

void AatModule::stopServoTestSweep()
{
    _servoTestPulseCalibrationStopRequested.store(true);
    _servoTestSweepState.store(static_cast<uint8_t>(ServoTestSweepState::Stopped));
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    _servoTestRequestedPulseUs.store(neutralUs);
    _servoTestLastCommandMs.store(millis());
}

bool AatModule::resetServoTestCompassCalibration()
{
    if (isServoTestPulseCalibrationBusy())
        return false;
    const uint8_t sweepState = _servoTestSweepState.load();
    const bool sweepActive = sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
        || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping);
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    if (!AatServo::canResetCompassCalibration(isServoTestSetupActive(), sweepActive,
        _servoTestReferencePressed.load() || digitalRead(PIN_AAT_REFERENCE_BUTTON) == LOW,
        _servoTestRequestedPulseUs.load(), _servoTestAppliedPulseUs.load(), neutralUs))
        return false;

    config.ResetAatPanCompassSetup();
    config.Commit();
    _servoTestCalibrationReady.store(false);
    _servoTestCalibrationSaved.store(false);
    _servoTestFitAttempted.store(false);
    _servoTestManualPulseAllowed.store(false);
    _servoTestCompassCalibratedHeadingValid.store(false);
    _servoTestCompassCalibratedRateValid.store(false);
                _servoTestCompassCalibratedRateSequence.fetch_add(1);
    return true;
}

bool AatModule::saveServoTestCompassCalibration()
{
    if (isServoTestPulseCalibrationBusy())
        return false;
#if defined(AAT_COMPASS_TRACKING)
    if (!_compassPanSetupMode.load())
        return false;
#endif
    if (!_servoTestCalibrationReady.load())
        return false;

    const auto &fit = _servoTestCompassCalibrationResult.calibration;
    aat_pan_compass_calibration_t saved = {};
    saved.offsetX = static_cast<int16_t>(lroundf(fit.offsetX));
    saved.offsetY = static_cast<int16_t>(lroundf(fit.offsetY));
    saved.scaleXPermille = static_cast<uint16_t>(lroundf(fit.scaleX * 1000.0f));
    saved.scaleYPermille = static_cast<uint16_t>(lroundf(fit.scaleY * 1000.0f));
    saved.headingOffsetDegrees = static_cast<int16_t>(fit.headingOffsetDegrees);
    saved.valid = 1;
    if (!config.SetAatPanCompassCalibration(saved))
        return false;
#if defined(AAT_COMPASS_TRACKING)
    if (!config.SetAatPanCompassNorthAligned(false))
        return false;
#else
    if (!config.SetAatPanCompassNorthAligned(true))
        return false;
#endif
    config.Commit();
    _servoTestCalibrationSaved.store(true);
    return true;
}

bool AatModule::canSaveServoTestCompassAlignment()
{
    if (isServoTestPulseCalibrationBusy())
        return false;
#if defined(AAT_COMPASS_TRACKING)
    if (!_compassPanSetupMode.load())
        return false;
#endif
    const uint8_t sweepState = _servoTestSweepState.load();
    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    const uint32_t now = millis();
    if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
        || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping)
        || _servoTestReferencePressed.load() || digitalRead(PIN_AAT_REFERENCE_BUTTON) == LOW
        || _servoTestRequestedPulseUs.load() != neutralUs
        || _servoTestAppliedPulseUs.load() != neutralUs
        || !_servoTestCompassDetected.load()
        || !_servoTestCompassCalibratedHeadingValid.load()
        || now - _servoTestCompassSampleAtMs.load() > 500U
        || !_servoTestCompassConfigReadValid.load()
        || _servoTestCompassConfigA.load() != AatServoTest::COMPASS_CONFIG_A
        || !_servoTestCompassStatusAfterTriggerValid.load()
        || (_servoTestCompassStatusAfterTrigger.load() & 0x02) != 0)
        return false;

    return config.GetAatPanCompassCalibration().valid == 1;
}

#if defined(AAT_COMPASS_TRACKING)
bool AatModule::captureCompassMountingStart()
{
    if (!canSaveServoTestCompassAlignment())
        return false;
    _compassMountingStartHeading.store(_servoTestCompassCalibratedHeadingDegrees.load());
    return true;
}

bool AatModule::saveCompassMountingResponse(bool reverse)
{
    if (!canSaveServoTestCompassAlignment())
        return false;
    const int32_t start = _compassMountingStartHeading.load();
    if (start < 0)
        return false;
    const int32_t turn = AatCompass::mountingTurnDegrees(start,
        _servoTestCompassCalibratedHeadingDegrees.load());
    if (!AatCompass::isQuarterTurn(reverse ? -turn : turn))
        return false;
    if (!(reverse ? config.SetAatPanCompassMountingReversed() : config.VerifyAatPanCompassMounting()))
        return false;
    _compassMountingStartHeading.store(-1);
    _servoTestCompassCalibratedHeadingValid.store(false);
    config.Commit();
    return true;
}
#endif

bool AatModule::saveServoTestCompassNorthBearing(int32_t referenceBearingDegrees)
{
    if (config.IsAatPanCompassMountingPending() || !canSaveServoTestCompassAlignment())
        return false;
    aat_pan_compass_calibration_t saved = config.GetAatPanCompassCalibration();
    const AatCompass::Calibration sensorCalibration = {
        static_cast<float>(saved.offsetX),
        static_cast<float>(saved.offsetY),
        saved.scaleXPermille / 1000.0f,
        saved.scaleYPermille / 1000.0f,
        0,
        true
    };
    int32_t sensorHeadingMilliDegrees = 0;
    if (!AatCompass::calculateHeadingMilliDegrees(_servoTestCompassRawX.load(),
        _servoTestCompassRawY.load(), sensorCalibration, &sensorHeadingMilliDegrees,
        config.IsAatPanCompassReversed())
        || !AatCompass::calculateCompassHeadingOffsetDegrees(sensorHeadingMilliDegrees,
            referenceBearingDegrees, &saved.headingOffsetDegrees))
        return false;

    if (!config.SetAatPanCompassCalibration(saved)
        || !config.SetAatPanCompassNorthAligned(true))
        return false;
    config.Commit();
    return true;
}

AatModule::ServoTestCompassData AatModule::getServoTestCompassData() const
{
    ServoTestCompassData data = {};
    data.detected = _servoTestCompassDetected.load();
    data.headingValid = _servoTestCompassHeadingValid.load();
    data.calibratedHeadingValid = _servoTestCompassCalibratedHeadingValid.load();
    data.rateValid = _servoTestCompassRateValid.load();
    data.calibratedRateValid = _servoTestCompassCalibratedRateValid.load();
    data.headingMilliDegrees = _servoTestCompassHeadingMilliDegrees.load();
    data.calibratedHeadingDegrees = _servoTestCompassCalibratedHeadingDegrees.load();
    data.rateMilliDegreesPerSecond = _servoTestCompassRateMilliDegreesPerSecond.load();
    data.calibratedRateMilliDegreesPerSecond = _servoTestCompassCalibratedRateMilliDegreesPerSecond.load();
    data.sampleAtMs = _servoTestCompassSampleAtMs.load();
    data.sampleCount = _servoTestCompassSampleCount.load();
    data.calibratedRateSequence = _servoTestCompassCalibratedRateSequence.load();
    data.pollCount = _servoTestCompassPollCount.load();
    data.rawX = _servoTestCompassRawX.load();
    data.rawY = _servoTestCompassRawY.load();
    data.rawZ = _servoTestCompassRawZ.load();
    data.sensorStatus = _servoTestCompassStatus.load();
    data.sensorStatusAfterRead = _servoTestCompassStatusAfterRead.load();
    data.sensorStatusAfterReadValid = _servoTestCompassStatusAfterReadValid.load();
    data.sensorStatusAfterTrigger = _servoTestCompassStatusAfterTrigger.load();
    data.sensorStatusAfterTriggerValid = _servoTestCompassStatusAfterTriggerValid.load();
    data.sensorStatusAfterTriggerAtMs = _servoTestCompassStatusAfterTriggerAtMs.load();
    data.configAAfterWrite = _servoTestCompassConfigAAfterWrite.load();
    data.configAAfterWriteReadValid = _servoTestCompassConfigAAfterWriteReadValid.load();
    data.configAAfterConfigBWrite = _servoTestCompassConfigAAfterConfigBWrite.load();
    data.configAAfterConfigBWriteReadValid = _servoTestCompassConfigAAfterConfigBWriteReadValid.load();
    data.configAAfterTrigger = _servoTestCompassConfigAAfterTrigger.load();
    data.configAAfterTriggerReadValid = _servoTestCompassConfigAAfterTriggerReadValid.load();
    data.configAAtFirstSampleDataReady = _servoTestCompassConfigAAtFirstSampleDataReady.load();
    data.configAAtFirstSampleDataReadyValid = _servoTestCompassConfigAAtFirstSampleDataReadyValid.load();
    data.configAAfterFirstSampleRead = _servoTestCompassConfigAAfterFirstSampleRead.load();
    data.configAAfterFirstSampleReadValid = _servoTestCompassConfigAAfterFirstSampleReadValid.load();
    data.configAAfterFirstSampleTrigger = _servoTestCompassConfigAAfterFirstSampleTrigger.load();
    data.configAAfterFirstSampleTriggerReadValid =
        _servoTestCompassConfigAAfterFirstSampleTriggerReadValid.load();
    data.configA = _servoTestCompassConfigA.load();
    data.configB = _servoTestCompassConfigB.load();
    data.configMode = _servoTestCompassConfigMode.load();
    data.configReadValid = _servoTestCompassConfigReadValid.load();
    data.configReadAtMs = _servoTestCompassConfigReadAtMs.load();
    data.panSafetyState = static_cast<AatServo::PanSafetyState>(_servoTestPanSafetyState.load());
    data.manualPulseAllowed = _servoTestManualPulseAllowed.load();
    data.referencePressed = _servoTestReferencePressed.load();
    data.sweepState = static_cast<ServoTestSweepState>(_servoTestSweepState.load());
    data.sweepFaultReason = static_cast<ServoTestSweepFaultReason>(
        _servoTestSweepFaultReason.load());
    data.sweepSamples = _servoTestSweepSamples.load();
    data.fitAttempted = _servoTestFitAttempted.load();
    data.calibrationReady = _servoTestCalibrationReady.load();
    data.calibrationSaved = _servoTestCalibrationSaved.load();
    data.siteSweep = _servoTestSiteSweep.load();
#if defined(AAT_COMPASS_TRACKING)
    data.setupActive = _compassPanSetupMode.load();
#else
    data.setupActive = true;
#endif
    data.pulseCalibrationState = _servoTestPulseCalibrationState.load();
    data.pulseCalibrationCandidateUs = _servoTestPulseCalibrationCandidateUs.load();
    data.pulseCalibrationSamples = _servoTestPulseCalibrationSamples.load();
    data.pulseCalibrationDisplacementMdeg = _servoTestPulseCalibrationDisplacementMdeg.load();
    data.pulseCalibrationFault = _servoTestPulseCalibrationFault.load();
    data.pulseCalibrationSettling = _servoTestPulseCalibrationSettling.load();
#if defined(AAT_COMPASS_TRACKING)
    data.pulseCalibrationSupported = true;
#endif
    data.pulseCalibrationValid = config.GetAatPanPulseCalibrationValid();
    data.leftStartPulseUs = config.GetAatPanLeftStartPulse();
    data.rightStartPulseUs = config.GetAatPanRightStartPulse();
    if (data.fitAttempted)
        data.calibration = _servoTestCompassCalibrationResult;
    return data;
}
#endif

void AatModule::overrideTargetCommon(int32_t azimuth, int32_t elevation)
{
    _targetAzim = azimuth;
    _targetElev = elevation;

    // Clear out other fields to not do projection or process any updates
    _isOverrideMode = true;
    _gpsAvgUpdateIntervalMs = 0;
    _targetDistance = 0;
    _azimMsPerDegree = 0;
}

void AatModule::overrideTargetBearing(int32_t bearing)
{
    // bearing is -180 to +180, azimuth 0-360
    int32_t azim = (360 + bearing + (45 * config.GetAatCenterDir())) % 360;
    // The furthest right a servo can go is +179 degrees, +180 goes to the -180 position
    if (bearing == 180)
        azim = (azim == 0) ? 359 : azim - 1;
    overrideTargetCommon(azim, _targetElev);
}

void AatModule::overrideTargetElev(int32_t elev)
{
    overrideTargetCommon(_targetAzim, constrain(elev, 0, 90));
}

void AatModule::onCrsfPacketIn(const crsf_header_t *pkt)
{
    if (pkt->sync_byte == CRSF_SYNC_BYTE)
    {
        if (pkt->type == CRSF_FRAMETYPE_GPS)
            SendGpsTelemetry((crsf_packet_gps_t *)pkt);
    }
}

void AatModule::Loop(uint32_t now)
{
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
    const uint16_t configuredNeutralUs = config.GetAatPanServoNeutral();
    if (_servoTestConfiguredNeutralUs != configuredNeutralUs)
    {
        _servoTestConfiguredNeutralUs = configuredNeutralUs;
        _servoTestRequestedPulseUs.store(configuredNeutralUs);
        _servoTestLastCommandMs.store(now);
        _servoTestManualPulseAllowed.store(false);
        _servoTestPanSafety.invalidate();
        _servoTestCalibratedRateWindow.reset();
        _servoTestCompassCalibratedRateValid.store(false);
        if (_servo_Azim.attached())
            _servo_Azim.writeMicroseconds(configuredNeutralUs);
        _servoTestAppliedPulseUs.store(configuredNeutralUs);
#if defined(AAT_COMPASS_TRACKING)
        _compassPanAppliedPulseUs = configuredNeutralUs;
#endif
    }
#endif
#if defined(AAT_SERVO_TEST)
    const bool rawReferencePressed = digitalRead(PIN_AAT_REFERENCE_BUTTON) == LOW;
    if (rawReferencePressed != _servoTestRawReferencePressed)
    {
        _servoTestRawReferencePressed = rawReferencePressed;
        _servoTestReferenceDebounceMs = now;
    }
    else if (now - _servoTestReferenceDebounceMs >= SERVO_TEST_REFERENCE_DEBOUNCE_MS)
    {
        _servoTestReferencePressed.store(rawReferencePressed);
    }

    if (now - _servoTestLastCompassPollMs >= 50U)
    {
        _servoTestLastCompassPollMs = now;
        _servoTestCompassPollCount.fetch_add(1);
        if (!_servoTestCompassConfigured)
        {
            uint8_t identity[3] = {};
            if (servoTestCompass.readIdentity(identity) && servoTestCompass.hasHmcIdentity(identity))
            {
#if defined(AAT_SERVO_TEST_CONTINUOUS_COMPASS)
                _servoTestCompassConfigured = servoTestCompass.configure(AatServoTest::COMPASS_CONFIG_A);
#else
                AatHmc5883l::ConfigurationReadback configReadback = {};
                _servoTestCompassConfigured = servoTestCompass.configureSingleShot(
                    AatServoTest::COMPASS_CONFIG_A, &configReadback);
                _servoTestCompassConfigAAfterWrite.store(configReadback.configAAfterWrite);
                _servoTestCompassConfigAAfterWriteReadValid.store(configReadback.afterWriteValid);
                _servoTestCompassConfigAAfterConfigBWrite.store(configReadback.configAAfterConfigBWrite);
                _servoTestCompassConfigAAfterConfigBWriteReadValid.store(
                    configReadback.afterConfigBWriteValid);
                _servoTestCompassConfigAAfterTrigger.store(configReadback.configAAfterTrigger);
                _servoTestCompassConfigAAfterTriggerReadValid.store(configReadback.afterTriggerValid);
#endif
                if (_servoTestCompassConfigured)
                {
                    _servoTestCompassLastTriggerMs = now;
                    uint8_t settings[3] = {};
                    const bool settingsRead = servoTestCompass.readConfiguration(settings);
                    _servoTestCompassConfigReadValid.store(settingsRead);
                    _servoTestCompassConfigReadAtMs.store(now);
                    if (settingsRead)
                    {
                        _servoTestCompassConfigA.store(settings[0]);
                        _servoTestCompassConfigB.store(settings[1]);
                        _servoTestCompassConfigMode.store(settings[2]);
                    }
                }
            }
            _servoTestCompassDetected.store(_servoTestCompassConfigured);
        }
        else
        {
            AatHmc5883l::RawSample sample = {};
            const bool captureFirstSampleConfig = _servoTestCompassSampleCount.load() == 0;
            AatHmc5883l::SampleConfigReadback sampleConfigReadback = {};
            const bool sampleReady = servoTestCompass.readRaw(&sample, nullptr,
                captureFirstSampleConfig ? &sampleConfigReadback : nullptr);
            _servoTestCompassStatusAfterReadValid.store(sample.statusAfterReadValid);
            if (sample.statusAfterReadValid)
                _servoTestCompassStatusAfterRead.store(sample.statusAfterRead);
            if (captureFirstSampleConfig)
            {
                _servoTestCompassConfigAAtFirstSampleDataReadyValid.store(
                    sampleConfigReadback.configAAtDataReadyValid);
                if (sampleConfigReadback.configAAtDataReadyValid)
                    _servoTestCompassConfigAAtFirstSampleDataReady.store(
                        sampleConfigReadback.configAAtDataReady);
            }
            if (sampleReady && captureFirstSampleConfig)
            {
                uint8_t settings[3] = {};
                const bool settingsRead = servoTestCompass.readConfiguration(settings);
                _servoTestCompassConfigAAfterFirstSampleReadValid.store(settingsRead);
                if (settingsRead)
                    _servoTestCompassConfigAAfterFirstSampleRead.store(settings[0]);
            }
            _servoTestCompassStatus.store(sample.status);
            if (sampleReady)
            {
                float headingDegrees = AatCompass::headingFromForwardRightXY(sample.x, sample.y);
                if (headingDegrees < 0.0f)
                    headingDegrees += 360.0f;
                const int32_t headingMilliDegrees = static_cast<int32_t>(lroundf(headingDegrees * 1000.0f));
                const aat_pan_compass_calibration_t savedCalibration = config.GetAatPanCompassCalibration();
                const AatCompass::Calibration calibration = {
                    static_cast<float>(savedCalibration.offsetX),
                    static_cast<float>(savedCalibration.offsetY),
                    savedCalibration.scaleXPermille / 1000.0f,
                    savedCalibration.scaleYPermille / 1000.0f,
                    savedCalibration.headingOffsetDegrees,
                    savedCalibration.valid == 1
                };
                int32_t calibratedHeadingMilliDegrees = 0;
                const bool calibratedHeadingValid = AatCompass::calculateHeadingMilliDegrees(
                    sample.x, sample.y, calibration, &calibratedHeadingMilliDegrees,
                    config.IsAatPanCompassReversed());
                const int32_t calibratedHeadingDegrees = calibratedHeadingValid
                    ? AatCompass::normalizeHeading((calibratedHeadingMilliDegrees + 500) / 1000)
                    : 0;
                if (_servoTestHasCompassHeading)
                {
                    int32_t deltaMilliDegrees = headingMilliDegrees - _servoTestPreviousCompassHeadingMilliDegrees;
                    if (deltaMilliDegrees > 180000)
                        deltaMilliDegrees -= 360000;
                    else if (deltaMilliDegrees < -180000)
                        deltaMilliDegrees += 360000;

                    const uint32_t sampleIntervalMs = now - _servoTestPreviousCompassSampleMs;
                    if (sampleIntervalMs > 0)
                    {
                        const int64_t rateMilliDegreesPerSecond =
                            (static_cast<int64_t>(deltaMilliDegrees) * 1000) / sampleIntervalMs;
                        _servoTestCompassRateMilliDegreesPerSecond.store(
                            static_cast<int32_t>(rateMilliDegreesPerSecond));
                        _servoTestCompassRateValid.store(true);
                    }
                }
                if (calibratedHeadingValid)
                {
                    int32_t calibratedRateMilliDegreesPerSecond = 0;
                    const AatCompass::RateWindowUpdate rateUpdate =
                        _servoTestCalibratedRateWindow.update(
                            now, calibratedHeadingMilliDegrees,
                            &calibratedRateMilliDegreesPerSecond);
                    if (rateUpdate == AatCompass::RateWindowUpdate::Updated)
                    {
                        _servoTestCompassCalibratedRateMilliDegreesPerSecond.store(
                            calibratedRateMilliDegreesPerSecond);
                        _servoTestCompassCalibratedRateSequence.fetch_add(1);
                        _servoTestCompassCalibratedRateValid.store(true);
                    }
                    else if (rateUpdate == AatCompass::RateWindowUpdate::Reset)
                        _servoTestCompassCalibratedRateValid.store(false);
                }
                else
                {
                    _servoTestCalibratedRateWindow.reset();
                    _servoTestCompassCalibratedRateValid.store(false);
                }
                _servoTestPreviousCompassHeadingMilliDegrees = headingMilliDegrees;
                _servoTestPreviousCompassSampleMs = now;
                _servoTestHasCompassHeading = true;
                _servoTestCompassRawX.store(sample.x);
                _servoTestCompassRawY.store(sample.y);
                _servoTestCompassRawZ.store(sample.z);
                _servoTestCompassHeadingMilliDegrees.store(headingMilliDegrees);
                _servoTestCompassCalibratedHeadingValid.store(calibratedHeadingValid);
                _servoTestCompassCalibratedHeadingDegrees.store(calibratedHeadingDegrees);
                _servoTestCompassSampleAtMs.store(now);
                _servoTestCompassSampleCount.fetch_add(1);
                _servoTestCompassHeadingValid.store(true);
                const uint8_t sweepState = _servoTestSweepState.load();
                if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping))
                {
                    if (_servoTestCompassSweep.addSample(sample.x, sample.y, sample.z,
                        now - _servoTestSweepStartMs.load()))
                        _servoTestSweepSamples.fetch_add(1);
                    else
                        _servoTestSweepState.store(static_cast<uint8_t>(ServoTestSweepState::Fault));
                }
#if !defined(AAT_SERVO_TEST_CONTINUOUS_COMPASS)
                if (!servoTestCompass.triggerSingleShot())
                {
                    _servoTestCompassStatusAfterTriggerValid.store(false);
                    _servoTestCompassConfigured = false;
                    _servoTestCompassDetected.store(false);
                }
                else
                {
                    _servoTestCompassLastTriggerMs = now;
                    uint8_t statusAfterTrigger = 0;
                    const bool statusRead = servoTestCompass.readStatus(&statusAfterTrigger);
                    _servoTestCompassStatusAfterTriggerValid.store(statusRead);
                    _servoTestCompassStatusAfterTriggerAtMs.store(now);
                    if (statusRead)
                        _servoTestCompassStatusAfterTrigger.store(statusAfterTrigger);
                    if (captureFirstSampleConfig)
                    {
                        uint8_t settings[3] = {};
                        const bool settingsRead = servoTestCompass.readConfiguration(settings);
                        _servoTestCompassConfigAAfterFirstSampleTriggerReadValid.store(settingsRead);
                        if (settingsRead)
                            _servoTestCompassConfigAAfterFirstSampleTrigger.store(settings[0]);
                    }
                }
#endif
            }
#if !defined(AAT_SERVO_TEST_CONTINUOUS_COMPASS)
            else if (now - _servoTestCompassLastTriggerMs >= SERVO_TEST_COMPASS_RETRY_MS)
            {
                if (servoTestCompass.triggerSingleShot())
                    _servoTestCompassLastTriggerMs = now;
                else
                {
                    _servoTestCompassConfigured = false;
                    _servoTestCompassDetected.store(false);
                }
            }
#endif
        }
    }

    if (_servoTestCompassConfigured
        && now - _servoTestCompassConfigReadAtMs.load() >= 1000U)
    {
        uint8_t settings[3] = {};
        const bool settingsRead = servoTestCompass.readConfiguration(settings);
        _servoTestCompassConfigReadValid.store(settingsRead);
        _servoTestCompassConfigReadAtMs.store(now);
        if (settingsRead)
        {
            _servoTestCompassConfigA.store(settings[0]);
            _servoTestCompassConfigB.store(settings[1]);
            _servoTestCompassConfigMode.store(settings[2]);
        }
    }

    uint8_t sweepState = _servoTestSweepState.load();
    if (AatServo::shouldStartSweepOnReferencePress(
        sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed),
        _servoTestReferencePressed.load()))
    {
        _servoTestSweepStartMs.store(now);
        sweepState = static_cast<uint8_t>(ServoTestSweepState::Sweeping);
        _servoTestSweepState.store(sweepState);
    }
    if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
        || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping))
    {
        const uint32_t sweepElapsedMs = now - _servoTestSweepStartMs.load();
        const uint32_t sampleAgeMs = now - _servoTestCompassSampleAtMs.load();
        if (AatServo::isSweepTimedOut(
            sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping),
            sweepElapsedMs, SERVO_TEST_SWEEP_TIMEOUT_MS))
        {
            _servoTestSweepFaultReason.store(
                static_cast<uint8_t>(ServoTestSweepFaultReason::Timeout));
            sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
        }
        else if (sampleAgeMs > 500U)
        {
            _servoTestSweepFaultReason.store(
                static_cast<uint8_t>(ServoTestSweepFaultReason::StaleCompass));
            sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
        }
        else if (AatServo::shouldCompleteSweepOnReferenceRelease(
            sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping),
            _servoTestReferencePressed.load()))
            sweepState = static_cast<uint8_t>(ServoTestSweepState::Complete);

        if (sweepState != static_cast<uint8_t>(ServoTestSweepState::Armed)
            && sweepState != static_cast<uint8_t>(ServoTestSweepState::Sweeping))
        {
            if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Complete))
            {
                AatCompassCalibration::Result result = {};
                const bool fitValid = _servoTestCompassSweep.fit(&result, true);
                _servoTestCompassCalibrationResult = result;
                _servoTestFitAttempted.store(true);
                if (fitValid)
                {
                    _servoTestCalibrationReady.store(true);
                    if (_servoTestSiteSweep.load())
                    {
                        aat_pan_compass_calibration_t saved = config.GetAatPanCompassCalibration();
                        if (AatCompassCalibration::refreshLocalOffset(saved, result)
                            && config.SetAatPanCompassCalibration(saved))
                        {
                            config.Commit();
                            _servoTestCalibrationSaved.store(true);
                        }
                        else
                            sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
                    }
                }
                else
                    sweepState = static_cast<uint8_t>(ServoTestSweepState::Fault);
            }
            _servoTestSweepState.store(sweepState);
            const uint16_t neutralUs = config.GetAatPanServoNeutral();
            _servoTestRequestedPulseUs.store(neutralUs);
            _servoTestLastCommandMs.store(now);
        }
    }

    const uint16_t neutralUs = config.GetAatPanServoNeutral();
    sweepState = _servoTestSweepState.load();
    const bool sweepActive = sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
        || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping);

    if (sweepActive)
    {
        _servoTestPanSafety.invalidate();
        _servoTestManualPulseAllowed.store(false);
    }
    else
    {
        const uint32_t compassSampleAgeMs = now - _servoTestCompassSampleAtMs.load();
        const bool compassFreshAndValid = _servoTestCompassDetected.load()
            && _servoTestCompassCalibratedHeadingValid.load()
            && compassSampleAgeMs <= 500U
            && AatCompass::isCompassSampleReleaseConfirmed(
                _servoTestCompassStatusAfterTriggerValid.load(),
                _servoTestCompassStatusAfterTrigger.load())
            && AatServo::isCompassConfigurationValid(
                _servoTestCompassConfigReadValid.load(),
                _servoTestCompassConfigA.load(), AatServoTest::COMPASS_CONFIG_A);
        const bool neutralCommanded = _servoTestAppliedPulseUs.load() == neutralUs;
        _servoTestPanSafety.updateReference(_servoTestReferencePressed.load(),
            neutralCommanded, compassFreshAndValid,
            _servoTestCompassCalibratedHeadingDegrees.load());

        AatServo::PanCommand currentPan = {};
        const int32_t currentHeading = _servoTestCompassCalibratedHeadingDegrees.load();
        const bool withinTravel = _servoTestPanSafety.calculateCommand(
            compassFreshAndValid, currentHeading, currentHeading, &currentPan);
        const bool manualPulseAllowed = withinTravel
            && currentPan.currentPanDegrees >= -AatServoTest::MANUAL_PULSE_MAX_PAN_DEGREES
            && currentPan.currentPanDegrees <= AatServoTest::MANUAL_PULSE_MAX_PAN_DEGREES
            && !_servoTestReferencePressed.load();
        _servoTestManualPulseAllowed.store(manualPulseAllowed);
        if (!manualPulseAllowed)
        {
            _servoTestRequestedPulseUs.store(neutralUs);
            _servoTestLastCommandMs.store(now);
        }
    }
    _servoTestPanSafetyState.store(static_cast<uint8_t>(_servoTestPanSafety.state()));

    const uint16_t pulseRequestUs = sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping)
        ? SERVO_TEST_SWEEP_PULSE_US : _servoTestRequestedPulseUs.load();
    const uint16_t requestedPulseUs = AatServo::getServoTestPulse(sweepActive, pulseRequestUs,
        neutralUs, now, _servoTestLastCommandMs.load(), AatServoTest::MANUAL_PULSE_TIMEOUT_MS);
    if (!sweepActive && requestedPulseUs == neutralUs)
        AatServo::clearServoTestPulseRequestIfUnchanged(
            _servoTestRequestedPulseUs, pulseRequestUs, neutralUs);
    if (requestedPulseUs != _servoTestAppliedPulseUs.load())
    {
        _servo_Azim.writeMicroseconds(requestedPulseUs);
        _servoTestAppliedPulseUs.store(requestedPulseUs);
#if defined(PIN_OLED_SDA)
        displayServoTest(requestedPulseUs, requestedPulseUs != neutralUs);
#endif
    }

    delay(DELAY_IDLE);
    return;
#endif
#if defined(AAT_PERF_DEBUG)
    const uint32_t loopStartedAtUs = micros();
    recordPerfLoopStart(loopStartedAtUs);
#endif
    processGps(now);
    _vbat.update(now);

#if defined(AAT_COMPASS_TRACKING)
    if (_compassPanModeActive)
    {
        updateCompassPan(now);
        if (!_compassPanSetupMode.load() && now > DELAY_FIRST_UPDATE
            && _compassPanTelemetryPanCommandValid.load())
            servoUpdate(now);
#if defined(PIN_OLED_SDA) && !defined(AAT_SERVO_TEST_DISABLE_OLED)
        if (!_compassPanSetupMode.load() && _compassPanStartupAlignmentComplete)
        {
            if (_compassPanSafety.state() != AatServo::PanSafetyState::Referenced)
            {
                if (_compassPanLastPromptState != 0xf9)
                {
                    displayCompassPanStartup(true, true, false);
                    _compassPanLastPromptState = 0xf9;
                }
            }
            else if (now - _lastDisplayActiveMs >= 250U)
            {
                _compassPanLastPromptState = 0xfd;
                if (isHomeSet() && now > DELAY_FIRST_UPDATE)
                    displayActive(now, calcProjectedAzim(now));
                else
                {
                    _lastDisplayActiveMs = now;
                    if (isGpsActive())
                        displayGpsIdle(now);
                    else
                        displayState();
                }
            }
        }
#endif
        CrsfModuleBase::Loop(now);
#if defined(AAT_PERF_DEBUG)
        recordPerfLoopDuration(micros() - loopStartedAtUs);
#endif
        return;
    }
#endif

#if defined(AAT_COMPASS_DEBUG)
#if defined(AAT_I2C_TRACE)
    pollCompassTrace(now);
#endif
    delay(DELAY_IDLE);
#else
    if (isHomeSet() && now > DELAY_FIRST_UPDATE)
    {
        servoUpdate(now);
    }
    else
    {
#if defined(PIN_OLED_SDA) && !defined(AAT_SERVO_TEST_DISABLE_OLED)
        if (isGpsActive())
            displayGpsIdle(now);
        else
            displayState();
#endif
        delay(DELAY_IDLE);
    }
#endif

    CrsfModuleBase::Loop(now);
#if defined(AAT_PERF_DEBUG)
    recordPerfLoopDuration(micros() - loopStartedAtUs);
#endif
}

#if defined(AAT_PERF_DEBUG)
AatPerfStats AatModule::getPerfStats() const
{
    AatPerfStats snapshot;
    portENTER_CRITICAL(&aatPerfMux);
    snapshot = _perfStats;
    portEXIT_CRITICAL(&aatPerfMux);
    return snapshot;
}

void AatModule::resetPerfStats()
{
    portENTER_CRITICAL(&aatPerfMux);
    _perfLastLoopStartUs = 0;
    _perfLastServoUpdateUs = 0;
    _perfStats = {};
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordPerfLoopStart(uint32_t startedAtUs)
{
    portENTER_CRITICAL(&aatPerfMux);
    if (_perfLastLoopStartUs != 0)
    {
        const uint32_t periodUs = startedAtUs - _perfLastLoopStartUs;
        if (_perfStats.loopIntervals == 0 || periodUs < _perfStats.loopPeriodMinUs)
            _perfStats.loopPeriodMinUs = periodUs;
        if (periodUs > _perfStats.loopPeriodMaxUs)
            _perfStats.loopPeriodMaxUs = periodUs;
        ++_perfStats.loopIntervals;
    }
    _perfLastLoopStartUs = startedAtUs;
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordPerfLoopDuration(uint32_t durationUs)
{
    portENTER_CRITICAL(&aatPerfMux);
    if (durationUs > _perfStats.loopDurationMaxUs)
        _perfStats.loopDurationMaxUs = durationUs;
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordPerfServoUpdate(uint32_t updatedAtUs)
{
    portENTER_CRITICAL(&aatPerfMux);
    if (_perfLastServoUpdateUs != 0)
    {
        const uint32_t periodUs = updatedAtUs - _perfLastServoUpdateUs;
        if (_perfStats.servoUpdates == 0 || periodUs < _perfStats.servoPeriodMinUs)
            _perfStats.servoPeriodMinUs = periodUs;
        if (periodUs > _perfStats.servoPeriodMaxUs)
            _perfStats.servoPeriodMaxUs = periodUs;
        ++_perfStats.servoUpdates;
    }
    _perfLastServoUpdateUs = updatedAtUs;
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordEspNowDatagram()
{
    portENTER_CRITICAL(&aatPerfMux);
    ++_perfStats.espNowDatagrams;
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordEspNowMspPacket()
{
    portENTER_CRITICAL(&aatPerfMux);
    ++_perfStats.espNowMspPackets;
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordEspNowDecision(bool macAccepted, bool queued)
{
    portENTER_CRITICAL(&aatPerfMux);
    if (macAccepted)
    {
        ++_perfStats.espNowMacAccepted;
        if (!queued)
            ++_perfStats.espNowQueueDrops;
    }
    else
        ++_perfStats.espNowMacRejected;
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordCrsfTelemetryPacket()
{
    portENTER_CRITICAL(&aatPerfMux);
    ++_perfStats.crsfTelemetryPackets;
    portEXIT_CRITICAL(&aatPerfMux);
}

void AatModule::recordGpsFrame()
{
    portENTER_CRITICAL(&aatPerfMux);
    ++_perfStats.gpsFrames;
    portEXIT_CRITICAL(&aatPerfMux);
}
#endif
#endif /* AAT_BACKPACK */
