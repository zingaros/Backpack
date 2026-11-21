#pragma once

#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
#include <atomic>
#include "aat_config_defaults.h"
#include "aat_compass_calibration.h"
#include "aat_servo.h"
#include "aat_pulse_calibration.h"
namespace AatServoTest
{
    constexpr uint16_t MIN_US = AatConfigDefaults::PAN_MIN_PULSE_US;
    constexpr uint16_t MAX_US = AatConfigDefaults::PAN_MAX_PULSE_US;
    constexpr int32_t MANUAL_PULSE_MAX_PAN_DEGREES = 90;
    constexpr uint32_t MANUAL_PULSE_TIMEOUT_MS = 3000;
#if defined(AAT_SERVO_TEST_COMPASS_CONFIG_A)
    constexpr uint8_t COMPASS_CONFIG_A = AAT_SERVO_TEST_COMPASS_CONFIG_A;
#elif defined(AAT_COMPASS_TRACKING_CONFIG_A)
    constexpr uint8_t COMPASS_CONFIG_A = AAT_COMPASS_TRACKING_CONFIG_A;
#else
    constexpr uint8_t COMPASS_CONFIG_A = 0x68;
#endif
    constexpr uint32_t CALIBRATED_RATE_WINDOW_MS = 500;
    constexpr uint32_t CALIBRATED_RATE_MAX_INTERVAL_MS = 1000;

    inline uint16_t manualPulseMinimum(uint16_t neutralUs, uint16_t maximumOffsetUs)
    {
        return neutralUs > MIN_US + maximumOffsetUs
            ? neutralUs - maximumOffsetUs : MIN_US;
    }

    inline uint16_t manualPulseMaximum(uint16_t neutralUs, uint16_t maximumOffsetUs)
    {
        return neutralUs < MAX_US - maximumOffsetUs
            ? neutralUs + maximumOffsetUs : MAX_US;
    }
}
#endif

#if defined(AAT_COMPASS_TRACKING)
#include <atomic>
#include "aat_config_defaults.h"
#include "aat_compass.h"
#include "aat_servo.h"
#endif

#if defined(AAT_BACKPACK)
#include "config.h"
#include "module_crsf.h"
#include "crsf_protocol.h"
#include "devWIFI.h"

#if defined(PIN_SERVO_AZIM) || defined(PIN_SERVO_ELEV)
#if defined(ESP32)
#include "ESP32Servo.h"
#else
#include <Servo.h>
#endif
#endif

#if defined(PIN_OLED_SDA)
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#endif

#if defined(AAT_PERF_DEBUG)
struct AatPerfStats
{
    uint32_t loopIntervals;
    uint32_t loopPeriodMinUs;
    uint32_t loopPeriodMaxUs;
    uint32_t loopDurationMaxUs;
    uint32_t servoUpdates;
    uint32_t servoPeriodMinUs;
    uint32_t servoPeriodMaxUs;
    uint32_t espNowDatagrams;
    uint32_t espNowMspPackets;
    uint32_t espNowMacAccepted;
    uint32_t espNowMacRejected;
    uint32_t espNowQueueDrops;
    uint32_t crsfTelemetryPackets;
    uint32_t gpsFrames;
};
#endif

class VbatSampler
{
public:
    VbatSampler();

    void update(uint32_t now);
    // Reported in V * 10
    uint32_t value() const { return _value; }
private:
    const uint32_t VBAT_UPDATE_INTERVAL = 100U;
    const uint32_t VBAT_UPDATE_COUNT = 5U;

    uint32_t _lastUpdate;
    uint32_t _value;
    uint32_t _sum;  // in progress sum/count
    uint32_t _cnt;
};

class AatModule : public CrsfModuleBase
{
public:
    AatModule() = delete;
    AatModule(Stream &port);

    void Init();
    void Loop(uint32_t now);

    void SendGpsTelemetry(crsf_packet_gps_t *packet);
    bool isHomeSet() const { return _home.lat != 0 || _home.lon != 0 || _isOverrideMode; }
    bool isGpsActive() const { return _gpsLast.lastUpdateMs != 0; };
    bool isGpsFresh(uint32_t now) const
    {
        return _gpsLast.lastUpdateMs != 0 && _gpsAvgUpdateIntervalMs > 0
            && now - _gpsLast.lastUpdateMs <= 3U * _gpsAvgUpdateIntervalMs;
    }
    uint32_t getGpsAgeMs(uint32_t now) const
    {
        return _gpsLast.lastUpdateMs == 0 ? UINT32_MAX : now - _gpsLast.lastUpdateMs;
    }
    uint8_t getGpsSatelliteCount() const { return _gpsLast.satcnt; }
    uint32_t getGpsAverageIntervalMs() const { return _gpsAvgUpdateIntervalMs; }

    void overrideTargetBearing(int32_t bearing);
    void overrideTargetElev(int32_t elev);
    uint32_t getVbat();
#if defined(AAT_COMPASS_TRACKING)
    struct CompassPanDiagnostics {
        bool modeActive;
        bool setupActive;
        uint8_t safetyState;
        uint8_t blockReason;
        bool referencePressed;
        bool compassDetected;
        bool compassSampleValid;
        bool compassFresh;
        uint8_t deadbandDegrees;
        bool configMatchesRequested;
        uint8_t configA;
        bool triggerStatusValid;
        bool triggerLockClear;
        int32_t headingDegrees;
        uint32_t sampleAgeMs;
        bool northAligned;
        bool startupAlignmentComplete;
        bool gpsFresh;
        int32_t targetHeadingDegrees;
        bool panCommandValid;
        int32_t currentPanDegrees;
        int32_t targetPanDegrees;
        int32_t errorDegrees;
        uint16_t requestedPulseUs;
        uint16_t appliedPulseUs;
    };

    CompassPanDiagnostics getCompassPanDiagnostics(uint32_t now) const;
    bool requestCompassPanSetup();
    bool captureCompassMountingStart();
    bool saveCompassMountingResponse(bool reverse);
    int32_t getCompassMountingStart() const { return _compassMountingStartHeading.load(); }
#endif
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
    enum class ServoTestSweepState : uint8_t {
        Ready,
        Armed,
        Sweeping,
        Complete,
        Stopped,
        Fault
    };
    enum class ServoTestSweepFaultReason : uint8_t {
        None,
        Timeout,
        NoCompassSamples,
        StaleCompass,
        SampleBufferFull
    };
    struct ServoTestCompassData {
        bool detected;
        bool headingValid;
        bool calibratedHeadingValid;
        bool rateValid;
        bool calibratedRateValid;
        int32_t headingMilliDegrees;
        int32_t calibratedHeadingDegrees;
        int32_t rateMilliDegreesPerSecond;
        int32_t calibratedRateMilliDegreesPerSecond;
        uint32_t sampleAtMs;
        uint32_t sampleCount;
        uint32_t calibratedRateSequence;
        uint32_t pollCount;
        int16_t rawX;
        int16_t rawY;
        int16_t rawZ;
        uint8_t sensorStatus;
        uint8_t sensorStatusAfterRead;
        bool sensorStatusAfterReadValid;
        uint8_t sensorStatusAfterTrigger;
        bool sensorStatusAfterTriggerValid;
        uint32_t sensorStatusAfterTriggerAtMs;
        uint8_t configAAfterWrite;
        bool configAAfterWriteReadValid;
        uint8_t configAAfterConfigBWrite;
        bool configAAfterConfigBWriteReadValid;
        uint8_t configAAfterTrigger;
        bool configAAfterTriggerReadValid;
        uint8_t configAAtFirstSampleDataReady;
        bool configAAtFirstSampleDataReadyValid;
        uint8_t configAAfterFirstSampleRead;
        bool configAAfterFirstSampleReadValid;
        uint8_t configAAfterFirstSampleTrigger;
        bool configAAfterFirstSampleTriggerReadValid;
        uint8_t configA;
        uint8_t configB;
        uint8_t configMode;
        bool configReadValid;
        uint32_t configReadAtMs;
        AatServo::PanSafetyState panSafetyState;
        bool manualPulseAllowed;
        bool referencePressed;
        ServoTestSweepState sweepState;
        ServoTestSweepFaultReason sweepFaultReason;
        uint32_t sweepSamples;
        bool fitAttempted;
        bool calibrationReady;
        bool calibrationSaved;
        bool siteSweep;
        bool setupActive;
        uint8_t pulseCalibrationState;
        uint16_t pulseCalibrationCandidateUs;
        uint8_t pulseCalibrationSamples;
        int32_t pulseCalibrationDisplacementMdeg;
        uint8_t pulseCalibrationFault;
        bool pulseCalibrationSettling;
        bool pulseCalibrationSupported;
        bool pulseCalibrationValid;
        uint16_t leftStartPulseUs;
        uint16_t rightStartPulseUs;
        AatCompassCalibration::Result calibration;
    };

    bool setServoTestPulse(uint16_t pulseUs);
    bool startServoTestSweep(bool siteSweep = false);
    bool startServoTestPulseCalibration();
    bool isServoTestPulseCalibrationBusy() const {
        return _servoTestPulseCalibrationBusy.load();
    }
    bool canConfigureServoTestPulseEnvelope();
    void stopServoTestSweep();
    bool saveServoTestCompassCalibration();
    bool resetServoTestCompassCalibration();
    bool saveServoTestCompassNorthBearing(int32_t referenceBearingDegrees);
    bool canSaveServoTestCompassAlignment();
    uint16_t getServoTestPulse() const { return _servoTestRequestedPulseUs.load(); }
    uint16_t getServoTestAppliedPulse() const { return _servoTestAppliedPulseUs.load(); }
        bool isServoTestSetupActive() const
        {
    #if defined(AAT_COMPASS_TRACKING)
        return _compassPanSetupMode.load();
    #else
        return true;
    #endif
        }
    bool isServoTestAttached() { return _servo_Azim.attached(); }
    uint16_t getServoTestLibraryPulse() { return _servo_Azim.readMicroseconds(); }
    ServoTestCompassData getServoTestCompassData() const;
    bool getServoTestCompassSweepSample(uint16_t index,
        AatCompassCalibration::Sample *sample) const
    {
        const uint8_t sweepState = _servoTestSweepState.load();
        if (sweepState == static_cast<uint8_t>(ServoTestSweepState::Armed)
            || sweepState == static_cast<uint8_t>(ServoTestSweepState::Sweeping))
            return false;
        return _servoTestCompassSweep.getSample(index, sample);
    }
#endif
#if defined(AAT_PERF_DEBUG)
    AatPerfStats getPerfStats() const;
    void resetPerfStats();
    void recordEspNowDatagram();
    void recordEspNowMspPacket();
    void recordEspNowDecision(bool macAccepted, bool queued);
    void recordCrsfTelemetryPacket();
    void recordGpsFrame();
#endif
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
    std::atomic<uint16_t> _servoTestRequestedPulseUs{AatConfigDefaults::PAN_SERVO_NEUTRAL_US};
    std::atomic<uint32_t> _servoTestLastCommandMs{0};
    std::atomic<uint16_t> _servoTestAppliedPulseUs{AatConfigDefaults::PAN_SERVO_NEUTRAL_US};
    std::atomic<bool> _servoTestCompassDetected{false};
    std::atomic<bool> _servoTestCompassHeadingValid{false};
    std::atomic<bool> _servoTestCompassCalibratedHeadingValid{false};
    std::atomic<bool> _servoTestCompassRateValid{false};
    std::atomic<bool> _servoTestCompassCalibratedRateValid{false};
    std::atomic<int32_t> _servoTestCompassHeadingMilliDegrees{0};
    std::atomic<int32_t> _servoTestCompassCalibratedHeadingDegrees{0};
    std::atomic<int32_t> _servoTestCompassRateMilliDegreesPerSecond{0};
    std::atomic<int32_t> _servoTestCompassCalibratedRateMilliDegreesPerSecond{0};
    std::atomic<uint32_t> _servoTestCompassSampleAtMs{0};
    std::atomic<uint32_t> _servoTestCompassSampleCount{0};
    std::atomic<uint32_t> _servoTestCompassCalibratedRateSequence{0};
    std::atomic<uint32_t> _servoTestCompassPollCount{0};
    std::atomic<int16_t> _servoTestCompassRawX{0};
    std::atomic<int16_t> _servoTestCompassRawY{0};
    std::atomic<int16_t> _servoTestCompassRawZ{0};
    std::atomic<uint8_t> _servoTestCompassStatus{0};
    std::atomic<uint8_t> _servoTestCompassStatusAfterRead{0};
    std::atomic<bool> _servoTestCompassStatusAfterReadValid{false};
    std::atomic<uint8_t> _servoTestCompassStatusAfterTrigger{0};
    std::atomic<bool> _servoTestCompassStatusAfterTriggerValid{false};
    std::atomic<uint32_t> _servoTestCompassStatusAfterTriggerAtMs{0};
    std::atomic<uint8_t> _servoTestCompassConfigAAfterWrite{0};
    std::atomic<bool> _servoTestCompassConfigAAfterWriteReadValid{false};
    std::atomic<uint8_t> _servoTestCompassConfigAAfterConfigBWrite{0};
    std::atomic<bool> _servoTestCompassConfigAAfterConfigBWriteReadValid{false};
    std::atomic<uint8_t> _servoTestCompassConfigAAfterTrigger{0};
    std::atomic<bool> _servoTestCompassConfigAAfterTriggerReadValid{false};
    std::atomic<uint8_t> _servoTestCompassConfigAAtFirstSampleDataReady{0};
    std::atomic<bool> _servoTestCompassConfigAAtFirstSampleDataReadyValid{false};
    std::atomic<uint8_t> _servoTestCompassConfigAAfterFirstSampleRead{0};
    std::atomic<bool> _servoTestCompassConfigAAfterFirstSampleReadValid{false};
    std::atomic<uint8_t> _servoTestCompassConfigAAfterFirstSampleTrigger{0};
    std::atomic<bool> _servoTestCompassConfigAAfterFirstSampleTriggerReadValid{false};
    std::atomic<uint8_t> _servoTestCompassConfigA{0};
    std::atomic<uint8_t> _servoTestCompassConfigB{0};
    std::atomic<uint8_t> _servoTestCompassConfigMode{0};
    std::atomic<bool> _servoTestCompassConfigReadValid{false};
    std::atomic<uint32_t> _servoTestCompassConfigReadAtMs{0};
    std::atomic<uint8_t> _servoTestPanSafetyState{
        static_cast<uint8_t>(AatServo::PanSafetyState::AwaitButtonRelease)};
    std::atomic<bool> _servoTestManualPulseAllowed{false};
    AatServo::PanSafetyController _servoTestPanSafety;
    uint32_t _servoTestCompassLastTriggerMs = 0;
    std::atomic<bool> _servoTestReferencePressed{false};
    std::atomic<uint8_t> _servoTestSweepState{static_cast<uint8_t>(ServoTestSweepState::Ready)};
    std::atomic<uint8_t> _servoTestSweepFaultReason{
        static_cast<uint8_t>(ServoTestSweepFaultReason::None)};
    std::atomic<uint32_t> _servoTestSweepStartMs{0};
    std::atomic<uint32_t> _servoTestSweepSamples{0};
    std::atomic<bool> _servoTestFitAttempted{false};
    std::atomic<bool> _servoTestCalibrationReady{false};
    std::atomic<bool> _servoTestCalibrationSaved{false};
    std::atomic<bool> _servoTestSiteSweep{false};
    AatCompassCalibration::Sweep _servoTestCompassSweep;
    AatCompassCalibration::Result _servoTestCompassCalibrationResult = {};
    bool _servoTestRawReferencePressed = false;
    AatServo::PulseCalibration _servoTestPulseCalibration;
    uint16_t _servoTestConfiguredNeutralUs = AatConfigDefaults::PAN_SERVO_NEUTRAL_US;
    std::atomic<bool> _servoTestPulseCalibrationBusy{false};
    std::atomic<bool> _servoTestPulseCalibrationStartRequested{false};
    std::atomic<bool> _servoTestPulseCalibrationStopRequested{false};
    std::atomic<uint8_t> _servoTestPulseCalibrationState{0};
    std::atomic<uint16_t> _servoTestPulseCalibrationCandidateUs{0};
    std::atomic<uint8_t> _servoTestPulseCalibrationSamples{0};
    std::atomic<int32_t> _servoTestPulseCalibrationDisplacementMdeg{0};
    std::atomic<uint8_t> _servoTestPulseCalibrationFault{0};
    std::atomic<bool> _servoTestPulseCalibrationSettling{false};
    uint32_t _servoTestReferenceDebounceMs = 0;
    uint32_t _servoTestLastCompassPollMs = 0;
    uint32_t _servoTestPreviousCompassSampleMs = 0;
    int32_t _servoTestPreviousCompassHeadingMilliDegrees = 0;
    AatCompass::HeadingRateWindow _servoTestCalibratedRateWindow{
        AatServoTest::CALIBRATED_RATE_WINDOW_MS,
        AatServoTest::CALIBRATED_RATE_MAX_INTERVAL_MS};
    bool _servoTestCompassConfigured = false;
    bool _servoTestHasCompassHeading = false;
#endif
protected:
    void overrideTargetCommon(int32_t azimuth, int32_t elevation);
    virtual void onCrsfPacketIn(const crsf_header_t *pkt);
private:
    enum ServoIndex { IDX_AZIM, IDX_ELEV, IDX_COUNT };

    void displayInit();
    void updateGpsInterval(uint32_t interval);
    uint8_t calcGpsIntervalPct(uint32_t now);
    int32_t calcProjectedAzim(uint32_t now);
    void servoApplyMode(int32_t azim, int32_t elev, int32_t newServoPos[]);
    void processGps(uint32_t now);
    void servoUpdate(uint32_t now);
#if defined(AAT_COMPASS_TRACKING)
    void updateCompassPan(uint32_t now);
    void displayCompassPanStartup(bool compassReady, bool northAligned,
        bool alignmentComplete);
#endif
    const int32_t azimToBearing(int32_t azim) const;
#if defined(AAT_PERF_DEBUG)
    void recordPerfLoopStart(uint32_t startedAtUs);
    void recordPerfLoopDuration(uint32_t durationUs);
    void recordPerfServoUpdate(uint32_t updatedAtUs);
#endif

#if defined(PIN_OLED_SDA)
    void displayState();
#if defined(AAT_SERVO_TEST)
    void displayServoTest(uint16_t pulseUs, bool active);
#endif
#if defined(AAT_COMPASS_TRACKING)
    void displayCompassPanSetup();
#endif
    void displayGpsIdle(uint32_t now);
    void displayActive(uint32_t now, int32_t projectedAzim);
    void displayGpsIntervalBar(uint32_t now);
    void displayAzimuthExtent(int32_t y);
    void displayAzimuth(int32_t projectedAzim);
    void displayAltitude(int32_t azimPos, int32_t elevPos);
    void displayTargetCircle(int32_t projectedAzim);
    void displayTargetDistance();
    void displayVBat();
#endif

    struct {
        int32_t lat;
        int32_t lon;
        uint32_t speed;   // km/h * 10
        uint32_t heading; // degrees * 10
        int32_t altitude; // meters
        uint32_t lastUpdateMs; // timestamp of last update
        uint8_t satcnt;   // number of satellites
        bool updated;
    } _gpsLast;
    struct {
        int32_t lat;
        int32_t lon;
        int32_t alt;
    } _home;
    uint32_t _gpsAvgUpdateIntervalMs;
    // Servo Position
    uint32_t _lastServoUpdateMs;
    uint32_t _targetDistance; // meters
    uint16_t _targetAzim; // degrees
    uint8_t _targetElev; // degrees
    bool    _isOverrideMode;
    int32_t _azimMsPerDegree; // milliseconds per degree
    int32_t _servoPos[IDX_COUNT]; // smoothed azim servo output us
    uint32_t _lastAzimFlipMs;
#if defined(AAT_COMPASS_TRACKING)
    bool _compassPanModeActive = false;
    std::atomic<bool> _compassPanSetupMode{false};
    std::atomic<bool> _compassPanSetupRequested{false};
    std::atomic<int32_t> _compassMountingStartHeading{-1};
    AatServo::PanSafetyController _compassPanSafety;
    AatServo::TrackingPanMedian _compassPanTrackingMedian;
    bool _compassPanRawReferencePressed = false;
    bool _compassPanReferencePressed = false;
    bool _compassPanConfigured = false;
    bool _compassPanDetected = false;
    bool _compassPanHeadingValid = false;
    bool _compassPanConfigReadValid = false;
    bool _compassPanStatusAfterTriggerValid = false;
    bool _compassPanStartupAlignmentComplete = false;
    uint8_t _compassPanConfigA = 0;
    uint8_t _compassPanStatusAfterTrigger = 0;
    uint8_t _compassPanLastPromptState = 0xff;
    int32_t _compassPanHeadingDegrees = 0;
    int16_t _compassPanRawX = 0;
    int16_t _compassPanRawY = 0;
#if defined(AAT_COMPASS_TRACKING_TRACE)
    int16_t _compassPanRawZ = 0;
    uint8_t _compassPanSampleStatus = 0;
    uint8_t _compassPanSampleStatusAfterRead = 0;
    bool _compassPanSampleStatusAfterReadValid = false;
#endif
    uint32_t _compassPanSampleAtMs = 0;
    uint32_t _compassPanLastPollMs = 0;
    uint32_t _compassPanLastTriggerMs = 0;
    uint32_t _compassPanConfigReadAtMs = 0;
    uint32_t _compassPanReferenceDebounceMs = 0;
    uint16_t _compassPanAppliedPulseUs = AatConfigDefaults::PAN_SERVO_NEUTRAL_US;
    std::atomic<bool> _compassPanTelemetryModeActive{false};
    std::atomic<bool> _compassPanTelemetrySetupActive{false};
    std::atomic<uint8_t> _compassPanTelemetrySafetyState{
        static_cast<uint8_t>(AatServo::PanSafetyState::AwaitButtonRelease)};
    std::atomic<uint8_t> _compassPanTelemetryBlockReason{
        static_cast<uint8_t>(AatServo::ContinuousPanBlockReason::Setup)};
    std::atomic<bool> _compassPanTelemetryReferencePressed{false};
    std::atomic<bool> _compassPanTelemetryCompassDetected{false};
    std::atomic<bool> _compassPanTelemetrySampleValid{false};
    std::atomic<bool> _compassPanTelemetryCompassFresh{false};
    std::atomic<bool> _compassPanTelemetryConfigMatchesRequested{false};
    std::atomic<uint8_t> _compassPanTelemetryConfigA{0};
    std::atomic<bool> _compassPanTelemetryTriggerStatusValid{false};
    std::atomic<bool> _compassPanTelemetryTriggerLockClear{false};
    std::atomic<int32_t> _compassPanTelemetryHeadingDegrees{0};
    std::atomic<uint32_t> _compassPanTelemetrySampleAtMs{0};
    std::atomic<bool> _compassPanTelemetryNorthAligned{false};
    std::atomic<bool> _compassPanTelemetryStartupComplete{false};
    std::atomic<bool> _compassPanTelemetryGpsFresh{false};
    std::atomic<int32_t> _compassPanTelemetryTargetHeadingDegrees{0};
    std::atomic<bool> _compassPanTelemetryPanCommandValid{false};
    std::atomic<int32_t> _compassPanTelemetryCurrentPanDegrees{0};
    std::atomic<int32_t> _compassPanTelemetryTargetPanDegrees{0};
    std::atomic<int32_t> _compassPanTelemetryErrorDegrees{0};
    std::atomic<uint16_t> _compassPanTelemetryRequestedPulseUs{AatConfigDefaults::PAN_SERVO_NEUTRAL_US};
    std::atomic<uint16_t> _compassPanTelemetryAppliedPulseUs{AatConfigDefaults::PAN_SERVO_NEUTRAL_US};
#endif
    VbatSampler _vbat;
#if defined(AAT_PERF_DEBUG)
    uint32_t _perfLastLoopStartUs = 0;
    uint32_t _perfLastServoUpdateUs = 0;
    AatPerfStats _perfStats = {};
#endif
#if defined(PIN_SERVO_AZIM)
    Servo _servo_Azim;
#endif
#if defined(PIN_SERVO_ELEV)
    Servo _servo_Elev;
#endif
#if defined(PIN_OLED_SDA)
    Adafruit_SSD1306 _display;
    uint32_t _lastDisplayActiveMs;
#endif
};

extern AatModule vrxModule;

#endif /* AAT_BACKPACK */
