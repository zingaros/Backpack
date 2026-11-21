#if defined(AAT_BACKPACK)

#include "devwifi_proxies.h"
#include "module_aat.h"
#include "common.h"
#include "options.h"
#include "aat_config_validation.h"
#include <stdlib.h>
#if defined(AAT_COMPASS_DEBUG) && !defined(AAT_COMPASS_TRACKING)
#include "aat_hmc5883l.h"
#endif

#if defined(AAT_COMPASS_TRACKING)
const char *continuousPanBlockReasonName(AatServo::ContinuousPanBlockReason reason)
{
    switch (reason)
    {
        case AatServo::ContinuousPanBlockReason::Setup: return "setup-active";
        case AatServo::ContinuousPanBlockReason::StartupAlignment: return "startup-alignment-required";
        case AatServo::ContinuousPanBlockReason::CompassInvalid: return "compass-invalid";
        case AatServo::ContinuousPanBlockReason::ReferenceUnavailable: return "reference-required";
        case AatServo::ContinuousPanBlockReason::GpsUnavailable: return "gps-stale";
        case AatServo::ContinuousPanBlockReason::SafetyFault: return "safety-fault";
        default: return "none";
    }
}
#endif

void WebAatAppendConfig(ArduinoJson::JsonDocument &json)
{
    auto connection = json.createNestedObject("connection");
    connection["state"] = connectionState == binding ? "binding" :
        (connectionState == running ? "running" :
        (connectionState == wifiUpdate ? "wifi_update" : "starting"));
    connection["binding"] = connectionState == binding;
    connection["uid_provisioned"] = firmwareOptions.hasUID;
    connection["gps_active"] = vrxModule.isGpsActive();
    connection["gps_fresh"] = vrxModule.isGpsFresh(millis());
    connection["gps_age_ms"] = vrxModule.getGpsAgeMs(millis());
    connection["gps_satellites"] = vrxModule.getGpsSatelliteCount();
    connection["gps_average_interval_ms"] = vrxModule.getGpsAverageIntervalMs();
    connection["boot_count"] = config.GetBootCount();
    connection["bind_window_ms"] = 120000U;

    auto aat = json["config"].createNestedObject("aat");
    aat["satmin"] = config.GetAatSatelliteHomeMin();
    aat["azim_sff"] = config.GetAatAzimuthServoFastFlip();
    aat["servosmoo"] = config.GetAatServoSmooth();
    aat["servomode"] = config.GetAatServoMode();
    aat["project"] = config.GetAatProject();
    aat["azim_center"] = config.GetAatCenterDir();
    aat["azim_min"] = config.GetAatServoLow(0);
    aat["azim_max"] = config.GetAatServoHigh(0);
    aat["pan_neutral_us"] = config.GetAatPanServoNeutral();
    aat["pan_deadband_degrees"] = config.GetAatPanDeadbandDegrees();
    aat["pan_pulse_calibration_valid"] =
        config.GetAatPanPulseCalibrationValid();
    aat["pan_left_start_us"] = config.GetAatPanLeftStartPulse();
    aat["pan_right_start_us"] = config.GetAatPanRightStartPulse();
    aat["pan_left_max_offset_us"] = config.GetAatPanLeftMaxOffset();
    aat["pan_right_max_offset_us"] = config.GetAatPanRightMaxOffset();
    aat["pan_north_aligned"] = config.GetAatPanCompassNorthAligned();
    const aat_pan_compass_calibration_t panCalibration = config.GetAatPanCompassCalibration();
    aat["pan_compass_valid"] = panCalibration.valid == 1;
    aat["pan_offset_x"] = panCalibration.offsetX;
    aat["pan_offset_y"] = panCalibration.offsetY;
    aat["pan_scale_x_permille"] = panCalibration.scaleXPermille;
    aat["pan_scale_y_permille"] = panCalibration.scaleYPermille;
    aat["pan_heading_offset_deg"] = panCalibration.headingOffsetDegrees;
    aat["elev_min"] = config.GetAatServoLow(1);
    aat["elev_max"] = config.GetAatServoHigh(1);

    // VBat
    auto vbat = json["config"].createNestedObject("vbat");
    vbat["offset"] = config.GetVbatOffset();
    vbat["scale"] = config.GetVbatScale();
    vbat["vbat"] = vrxModule.getVbat();

#if defined(AAT_COMPASS_TRACKING)
    const AatModule::CompassPanDiagnostics panStatus =
        vrxModule.getCompassPanDiagnostics(millis());
    auto panRuntime = json.createNestedObject("pan_runtime");
    panRuntime["mode_active"] = panStatus.modeActive;
    panRuntime["setup_active"] = panStatus.setupActive;
    panRuntime["safety_state"] = panStatus.safetyState;
    panRuntime["block_reason"] = continuousPanBlockReasonName(
        static_cast<AatServo::ContinuousPanBlockReason>(panStatus.blockReason));
    panRuntime["reference_pressed"] = panStatus.referencePressed;
    panRuntime["startup_alignment_complete"] = panStatus.startupAlignmentComplete;
    panRuntime["north_aligned"] = panStatus.northAligned;
    panRuntime["compass_detected"] = panStatus.compassDetected;
    panRuntime["compass_sample_valid"] = panStatus.compassSampleValid;
    panRuntime["compass_fresh"] = panStatus.compassFresh;
    panRuntime["deadband_degrees"] = panStatus.deadbandDegrees;
    panRuntime["compass_config_a"] = panStatus.configA;
    panRuntime["compass_config_matches_requested"] = panStatus.configMatchesRequested;
    panRuntime["trigger_status_valid"] = panStatus.triggerStatusValid;
    panRuntime["trigger_lock_clear"] = panStatus.triggerLockClear;
    panRuntime["heading_deg"] = panStatus.headingDegrees;
    panRuntime["sample_age_ms"] = panStatus.sampleAgeMs;
    panRuntime["gps_fresh"] = panStatus.gpsFresh;
    panRuntime["target_heading_deg"] = panStatus.targetHeadingDegrees;
    panRuntime["pan_command_valid"] = panStatus.panCommandValid;
    panRuntime["current_pan_deg"] = panStatus.currentPanDegrees;
    panRuntime["target_pan_deg"] = panStatus.targetPanDegrees;
    panRuntime["error_deg"] = panStatus.errorDegrees;
    panRuntime["requested_pulse_us"] = panStatus.requestedPulseUs;
    panRuntime["applied_pulse_us"] = panStatus.appliedPulseUs;
    panRuntime["neutral_us"] = config.GetAatPanServoNeutral();
#endif
}

static bool parseUnsignedRequestArg(AsyncWebServerRequest *request, const char *name,
    uint32_t defaultValue, uint32_t *value)
{
    if (value == nullptr)
        return false;
    *value = defaultValue;
    if (!request->hasArg(name))
        return true;

    const String text = request->arg(name);
    if (text.length() == 0 || text.length() > 10)
        return false;
    uint32_t parsed = 0;
    for (size_t index = 0; index < text.length(); ++index)
    {
        const char character = text[index];
        if (character < '0' || character > '9')
            return false;
        const uint32_t digit = static_cast<uint32_t>(character - '0');
        if (parsed > (UINT32_MAX - digit) / 10U)
            return false;
        parsed = parsed * 10U + digit;
    }
    *value = parsed;
    return true;
}

void WebAatConfig(AsyncWebServerRequest *request)
{
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
    if (vrxModule.isServoTestPulseCalibrationBusy())
    {
        request->send(409, "text/plain", "Stop pulse calibration before changing configuration");
        return;
    }
#endif
    const bool hasPanDeadband = request->hasArg("pan_deadband_degrees");
    long panDeadbandDegrees = -1;
    if (hasPanDeadband)
    {
        const String deadbandText = request->arg("pan_deadband_degrees");
        char *end = nullptr;
        panDeadbandDegrees = strtol(deadbandText.c_str(), &end, 10);
        if (deadbandText.length() == 0 || *end != '\0'
            || !AatConfigValidation::isPanDeadbandValid(panDeadbandDegrees))
        {
            request->send(400, "text/plain", "Pan deadband must be an integer from 1 to 30 degrees");
            return;
        }
    }

    const bool hasPanMaximumOffsets = request->hasArg("pan_left_max_offset_us")
        || request->hasArg("pan_right_max_offset_us");
    const bool hasPanEnvelope = request->hasArg("pan_neutral_us") || hasPanMaximumOffsets;
    uint32_t neutralUs = config.GetAatPanServoNeutral();
    uint32_t leftOffsetUs = config.GetAatPanLeftMaxOffset();
    uint32_t rightOffsetUs = config.GetAatPanRightMaxOffset();
    if (hasPanEnvelope)
    {
        if (hasPanMaximumOffsets && (!request->hasArg("pan_left_max_offset_us")
            || !request->hasArg("pan_right_max_offset_us")))
        {
            request->send(400, "text/plain", "Both continuous-pan maximum offsets are required");
            return;
        }
        if (!parseUnsignedRequestArg(request, "pan_neutral_us", neutralUs, &neutralUs)
            || !parseUnsignedRequestArg(request, "pan_left_max_offset_us", leftOffsetUs, &leftOffsetUs)
            || !parseUnsignedRequestArg(request, "pan_right_max_offset_us", rightOffsetUs, &rightOffsetUs)
            || !AatConfigValidation::isPanPulseEnvelopeValid(
                neutralUs, leftOffsetUs, rightOffsetUs))
        {
            request->send(400, "text/plain",
                "Pan neutral must be 1450-1550 us; each offset must be at least 5 us and keep pulse limits within 1000-2000 us");
            return;
        }
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
        const bool changed = neutralUs != config.GetAatPanServoNeutral()
            || leftOffsetUs != config.GetAatPanLeftMaxOffset()
            || rightOffsetUs != config.GetAatPanRightMaxOffset();
        if (changed && !vrxModule.canConfigureServoTestPulseEnvelope())
        {
            request->send(409, "text/plain", "Pulse settings require idle setup or a detached servo at neutral");
            return;
        }
#endif
    }

    const bool hasPanOffsetX = request->hasArg("pan_offset_x");
    const bool hasPanOffsetY = request->hasArg("pan_offset_y");
    const bool hasPanScaleX = request->hasArg("pan_scale_x_permille");
    const bool hasPanScaleY = request->hasArg("pan_scale_y_permille");
    const bool hasAnyPanCalibration = hasPanOffsetX || hasPanOffsetY
        || hasPanScaleX || hasPanScaleY;
    if (hasAnyPanCalibration)
    {
        if (config.GetAatServoMode() == 3)
        {
            request->send(409, "text/plain", "Compass axis fit cannot be changed while continuous pan mode is selected");
            return;
        }
        if (!hasPanOffsetX || !hasPanOffsetY || !hasPanScaleX || !hasPanScaleY)
        {
            request->send(400, "text/plain", "All four pan axis-fit values are required");
            return;
        }

        const String offsetXText = request->arg("pan_offset_x");
        const String offsetYText = request->arg("pan_offset_y");
        const String scaleXText = request->arg("pan_scale_x_permille");
        const String scaleYText = request->arg("pan_scale_y_permille");
        char *end = nullptr;
        const long offsetX = strtol(offsetXText.c_str(), &end, 10);
        if (offsetXText.length() == 0 || *end != '\0' || offsetX < INT16_MIN || offsetX > INT16_MAX)
        {
            request->send(400, "text/plain", "Pan X offset must be an integer from -32768 to 32767");
            return;
        }
        const long offsetY = strtol(offsetYText.c_str(), &end, 10);
        if (offsetYText.length() == 0 || *end != '\0' || offsetY < INT16_MIN || offsetY > INT16_MAX)
        {
            request->send(400, "text/plain", "Pan Y offset must be an integer from -32768 to 32767");
            return;
        }
        const long scaleX = strtol(scaleXText.c_str(), &end, 10);
        if (scaleXText.length() == 0 || *end != '\0' || scaleX < 250 || scaleX > 4000)
        {
            request->send(400, "text/plain", "Pan X scale must be an integer from 250 to 4000 permille");
            return;
        }
        const long scaleY = strtol(scaleYText.c_str(), &end, 10);
        if (scaleYText.length() == 0 || *end != '\0' || scaleY < 250 || scaleY > 4000)
        {
            request->send(400, "text/plain", "Pan Y scale must be an integer from 250 to 4000 permille");
            return;
        }

        aat_pan_compass_calibration_t restored = {};
        restored.offsetX = static_cast<int16_t>(offsetX);
        restored.offsetY = static_cast<int16_t>(offsetY);
        restored.scaleXPermille = static_cast<uint16_t>(scaleX);
        restored.scaleYPermille = static_cast<uint16_t>(scaleY);
        restored.headingOffsetDegrees = 0;
        restored.valid = 1;
        if (!config.SetAatPanCompassCalibration(restored))
        {
            request->send(400, "text/plain", "Pan axis fit failed validation");
            return;
        }
    }

    if (hasPanEnvelope && !config.SetAatPanPulseEnvelope(neutralUs, leftOffsetUs, rightOffsetUs))
    {
        request->send(400, "text/plain", "Invalid continuous-pan pulse envelope");
        return;
    }

    // Servos
    if (request->hasArg("servosmoo"))
        config.SetAatServoSmooth(request->arg("servosmoo").toInt());
    if (request->hasArg("azim_sff"))
        config.SetAatAzimuthServoFastFlip(request->arg("azim_sff").toInt());
    else
        config.SetAatAzimuthServoFastFlip(0);
    if (request->hasArg("servomode"))
    {
        const int servoMode = request->arg("servomode").toInt();
        if (servoMode < 0 || servoMode > 3)
        {
            request->send(400, "text/plain", "Unsupported servo mode");
            return;
        }
        if (servoMode == 3)
        {
#if !defined(AAT_COMPASS_TRACKING)
            request->send(409, "text/plain", "Continuous pan is unavailable in this firmware");
            return;
#else
            // Mode 3 boots into a neutral setup state until the compass fit is saved.
#endif
        }
        config.SetAatServoMode(static_cast<uint8_t>(servoMode));
    }
    if (request->hasArg("azim_center"))
        config.SetAatCenterDir(request->arg("azim_center").toInt());
    if (request->hasArg("azim_min"))
        config.SetAatServoLow(0, request->arg("azim_min").toInt());
    if (request->hasArg("azim_max"))
        config.SetAatServoHigh(0, request->arg("azim_max").toInt());
    if (request->hasArg("elev_min"))
        config.SetAatServoLow(1, request->arg("elev_min").toInt());
    if (request->hasArg("elev_max"))
        config.SetAatServoHigh(1, request->arg("elev_max").toInt());
    // VBat
    if (request->hasArg("vbat_offset"))
        config.SetVbatOffset(request->arg("vbat_offset").toInt());
    if (request->hasArg("vbat_scale"))
        config.SetVbatScale(request->arg("vbat_scale").toInt());
    // Servo Target Override (must be set after azim_center because bearing relies on center)
    if (request->hasArg("bear"))
        vrxModule.overrideTargetBearing(request->arg("bear").toInt());
    if (request->hasArg("elev"))
        vrxModule.overrideTargetElev(request->arg("elev").toInt());
    // Satellite Config
    if (request->hasArg("satmin"))
        config.SetAatSatelliteHomeMin(request->arg("satmin").toInt());

    if (hasPanDeadband
        && !config.SetAatPanDeadbandDegrees(static_cast<uint8_t>(panDeadbandDegrees)))
    {
        request->send(400, "text/plain", "Pan deadband must be an integer from 1 to 30 degrees");
        return;
    }

    const char *response;
    if (request->arg("commit").toInt() == 1)
    {
        response = "Saved";
        config.Commit();
    }
    else
        response = "Modified";

    request->send(200, "text/plain", response);
}

#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
const char *servoTestSweepStateName(AatModule::ServoTestSweepState state)
{
    switch (state)
    {
        case AatModule::ServoTestSweepState::Armed: return "armed";
        case AatModule::ServoTestSweepState::Sweeping: return "sweeping";
        case AatModule::ServoTestSweepState::Complete: return "complete";
        case AatModule::ServoTestSweepState::Stopped: return "stopped";
        case AatModule::ServoTestSweepState::Fault: return "fault";
        default: return "ready";
    }
}

const char *servoTestSweepFaultReasonName(AatModule::ServoTestSweepFaultReason reason)
{
    switch (reason)
    {
        case AatModule::ServoTestSweepFaultReason::Timeout: return "timeout";
        case AatModule::ServoTestSweepFaultReason::NoCompassSamples: return "no-compass-samples";
        case AatModule::ServoTestSweepFaultReason::StaleCompass: return "stale-compass";
        case AatModule::ServoTestSweepFaultReason::SampleBufferFull: return "sample-buffer-full";
        default: return "none";
    }
}

const char *panSafetyStateName(AatServo::PanSafetyState state)
{
    switch (state)
    {
        case AatServo::PanSafetyState::AwaitButtonRelease: return "await-button-release";
        case AatServo::PanSafetyState::Unreferenced: return "unreferenced";
        case AatServo::PanSafetyState::Referenced: return "referenced";
        case AatServo::PanSafetyState::Fault: return "fault";
        default: return "unknown";
    }
}

const char *compassFitFailureName(AatCompassCalibration::Failure failure)
{
    switch (failure)
    {
        case AatCompassCalibration::Failure::FullRotationNotConfirmed: return "rotation not confirmed";
        case AatCompassCalibration::Failure::TooFewSamples: return "too few samples";
        case AatCompassCalibration::Failure::NoXVariation: return "no X-axis variation";
        case AatCompassCalibration::Failure::NoYVariation: return "no Y-axis variation";
        case AatCompassCalibration::Failure::InsufficientCoverage: return "insufficient sector coverage";
        case AatCompassCalibration::Failure::ReturnClosureMismatch: return "return position mismatch";
        default: return "none";
    }
}

void WebAatServo(AsyncWebServerRequest *request)
{
    if (request->hasArg("reset_calibration"))
    {
        if (request->method() != HTTP_POST)
        {
            request->send(405, "text/plain", "Resetting compass calibration must use POST");
            return;
        }
        if (request->args() != 1 || request->arg("reset_calibration") != "1")
        {
            request->send(400, "text/plain", "Reset must be a standalone reset_calibration=1 request");
            return;
        }
        if (!vrxModule.resetServoTestCompassCalibration())
        {
            request->send(409, "text/plain", "Reset requires setup mode, no sweep, released GPIO32, and neutral output");
            return;
        }
    }
    if (request->hasArg("enter_setup")
        && request->arg("enter_setup").toInt() == 1)
    {
        if (request->method() != HTTP_POST)
        {
            request->send(405, "text/plain", "Entering setup mode must use POST");
            return;
        }
    #if defined(AAT_COMPASS_TRACKING)
        if (!vrxModule.requestCompassPanSetup())
        {
            request->send(409, "text/plain", "CompassPan tracking must be active before entering setup mode");
            return;
        }
    #else
        request->send(409, "text/plain", "ServoTest firmware is already in setup mode");
        return;
    #endif
    }
    if (request->hasArg("calibrate_pulses"))
    {
        if (request->method() != HTTP_POST
            || request->arg("calibrate_pulses") != "1" || request->args() != 1)
        {
            request->send(400, "text/plain",
                "Pulse calibration must use a standalone POST calibrate_pulses=1 request");
            return;
        }
        if (!vrxModule.startServoTestPulseCalibration())
        {
            request->send(409, "text/plain",
                "Pulse calibration requires CompassPan setup, saved fit/north, a referenced pan at neutral, valid compass gates, and no active sweep");
            return;
        }
    }
    if (request->hasArg("sweep"))
    {
        const String action = request->arg("sweep");
        if (action == "owner" && !vrxModule.startServoTestSweep(false))
        {
            request->send(409, "text/plain", "Owner calibration requires GPIO32 released, a fresh compass sample, matching config, clear trigger LOCK, and neutral output");
            return;
        }
        if (action == "site")
        {
            request->send(410, "text/plain", "Site alignment sweep is not available in this setup flow");
            return;
        }
        if (action == "stop")
            vrxModule.stopServoTestSweep();
    }
    if (request->hasArg("save_calibration")
        && request->arg("save_calibration").toInt() == 1
        && !vrxModule.saveServoTestCompassCalibration())
    {
        request->send(409, "text/plain", "No valid completed sweep to save");
        return;
    }
    if (request->hasArg("mount_check"))
    {
        const String action = request->arg("mount_check");
        if (request->method() != HTTP_POST || request->args() != 1
            || (action != "capture" && action != "reverse" && action != "verify"))
        {
            request->send(400, "text/plain", "Mounting check requires a standalone POST with capture, reverse, or verify");
            return;
        }
#if defined(AAT_COMPASS_TRACKING)
        const bool accepted = action == "capture" ? vrxModule.captureCompassMountingStart()
            : vrxModule.saveCompassMountingResponse(action == "reverse");
        if (!accepted)
        {
            request->send(409, "text/plain", "Mounting check requires fresh compass, neutral setup, and a valid captured quarter turn before saving");
            return;
        }
#else
        request->send(409, "text/plain", "Mounting correction requires CompassPan firmware");
        return;
#endif
    }
    if (request->hasArg("north_bearing_deg"))
    {
        if (request->method() != HTTP_POST)
        {
            request->send(405, "text/plain", "North alignment must use POST");
            return;
        }
        const String bearingText = request->arg("north_bearing_deg");
        if (bearingText.length() == 0 || bearingText.length() > 3)
        {
            request->send(400, "text/plain", "Reference bearing must be an integer from 0 to 359 degrees");
            return;
        }
        for (size_t index = 0; index < bearingText.length(); ++index)
        {
            if (bearingText[index] < '0' || bearingText[index] > '9')
            {
                request->send(400, "text/plain", "Reference bearing must be an integer from 0 to 359 degrees");
                return;
            }
        }
        const int referenceBearing = bearingText.toInt();
        if (referenceBearing > 359)
        {
            request->send(400, "text/plain", "Reference bearing must be an integer from 0 to 359 degrees");
            return;
        }
        if (!vrxModule.saveServoTestCompassNorthBearing(referenceBearing))
        {
            request->send(409, "text/plain", "North alignment requires a saved compass fit, fresh valid sample, matching config, clear trigger LOCK, no sweep, and neutral output");
            return;
        }
    }
    if (request->hasArg("pulse"))
    {
        const int pulseUs = request->arg("pulse").toInt();
        if (pulseUs < AatServoTest::MIN_US || pulseUs > AatServoTest::MAX_US)
        {
            request->send(400, "text/plain", "Pulse must be between 1000 and 2000 us");
            return;
        }
        if (!vrxModule.setServoTestPulse((uint16_t)pulseUs))
        {
            request->send(409, "text/plain", "Manual pulse requires a fresh reference, valid compass, safe pan angle, and pulse within the limited test range");
            return;
        }
    }

    ArduinoJson::JsonDocument json;
    json["pulse_us"] = vrxModule.getServoTestPulse();
    json["output_pulse_us"] = vrxModule.getServoTestAppliedPulse();
    json["servo_attached"] = vrxModule.isServoTestAttached();
    json["library_pulse_us"] = vrxModule.getServoTestLibraryPulse();
    json["neutral_us"] = config.GetAatPanServoNeutral();
    json["min_us"] = AatServoTest::manualPulseMinimum(config.GetAatPanServoNeutral(),
        config.GetAatPanLeftMaxOffset());
    json["max_us"] = AatServoTest::manualPulseMaximum(config.GetAatPanServoNeutral(),
        config.GetAatPanRightMaxOffset());
    const AatModule::ServoTestCompassData compassData = vrxModule.getServoTestCompassData();
    const uint32_t sampleAgeMs = millis() - compassData.sampleAtMs;
    json["compass_detected"] = compassData.detected;
    json["heading_valid"] = compassData.headingValid && sampleAgeMs < 1000U;
    json["heading_mdeg"] = compassData.headingMilliDegrees;
    json["calibrated_heading_valid"] = compassData.calibratedHeadingValid && sampleAgeMs <= 500U;
    json["calibrated_heading_deg"] = compassData.calibratedHeadingDegrees;
    json["pan_reference_state"] = panSafetyStateName(compassData.panSafetyState);
    json["manual_pulse_allowed"] = compassData.manualPulseAllowed;
    json["raw_x"] = compassData.rawX;
    json["raw_y"] = compassData.rawY;
    json["compass_poll_count"] = compassData.pollCount;
    json["compass_data_ready"] = (compassData.sensorStatus & 0x01) != 0;
    json["compass_data_locked"] = (compassData.sensorStatus & 0x02) != 0;
    json["compass_data_ready_after_read"] = (compassData.sensorStatusAfterRead & 0x01) != 0;
    json["compass_data_locked_after_read"] = (compassData.sensorStatusAfterRead & 0x02) != 0;
    json["compass_status_after_read_valid"] = compassData.sensorStatusAfterReadValid;
    json["compass_status_after_trigger_valid"] = compassData.sensorStatusAfterTriggerValid;
    json["compass_status_after_trigger_age_ms"] = millis() - compassData.sensorStatusAfterTriggerAtMs;
    json["compass_data_ready_after_trigger"] = (compassData.sensorStatusAfterTrigger & 0x01) != 0;
    json["compass_data_locked_after_trigger"] = (compassData.sensorStatusAfterTrigger & 0x02) != 0;
    json["compass_config_a_after_write"] = compassData.configAAfterWrite;
    json["compass_config_a_after_write_read_valid"] = compassData.configAAfterWriteReadValid;
    json["compass_config_a_after_write_matches_requested"] = compassData.configAAfterWriteReadValid
        && compassData.configAAfterWrite == AatServoTest::COMPASS_CONFIG_A;
    json["compass_config_a_after_config_b_write"] = compassData.configAAfterConfigBWrite;
    json["compass_config_a_after_config_b_write_read_valid"] = compassData.configAAfterConfigBWriteReadValid;
    json["compass_config_a_after_trigger"] = compassData.configAAfterTrigger;
    json["compass_config_a_after_trigger_read_valid"] = compassData.configAAfterTriggerReadValid;
    json["compass_config_a_at_first_sample_data_ready"] = compassData.configAAtFirstSampleDataReady;
    json["compass_config_a_at_first_sample_data_ready_valid"] =
        compassData.configAAtFirstSampleDataReadyValid;
    json["compass_config_a_after_first_sample_read"] = compassData.configAAfterFirstSampleRead;
    json["compass_config_a_after_first_sample_read_valid"] = compassData.configAAfterFirstSampleReadValid;
    json["compass_config_a_after_first_sample_trigger"] = compassData.configAAfterFirstSampleTrigger;
    json["compass_config_a_after_first_sample_trigger_read_valid"] =
        compassData.configAAfterFirstSampleTriggerReadValid;
    json["compass_config_a"] = compassData.configA;
    json["compass_config_a_requested"] = AatServoTest::COMPASS_CONFIG_A;
    json["compass_config_a_matches_requested"] = compassData.configReadValid
        && compassData.configA == AatServoTest::COMPASS_CONFIG_A;
    json["compass_config_read_valid"] = compassData.configReadValid;
    json["compass_config_read_age_ms"] = millis() - compassData.configReadAtMs;
    json["compass_config_b"] = compassData.configB;
    json["compass_mode"] = compassData.configMode;
    json["rate_valid"] = compassData.rateValid && sampleAgeMs < 1000U;
    json["rate_mdeg_s"] = compassData.rateMilliDegreesPerSecond;
    json["calibrated_rate_valid"] = compassData.calibratedRateValid && sampleAgeMs <= 500U;
    json["calibrated_rate_mdeg_s"] = compassData.calibratedRateMilliDegreesPerSecond;
    json["calibrated_rate_sequence"] = compassData.calibratedRateSequence;
    json["compass_sample_age_ms"] = sampleAgeMs;
    json["compass_sample_count"] = compassData.sampleCount;
    json["reference_raw_pressed"] = digitalRead(PIN_AAT_REFERENCE_BUTTON) == LOW;
    json["reference_pressed"] = compassData.referencePressed;
    json["sweep_state"] = servoTestSweepStateName(compassData.sweepState);
    json["sweep_fault_reason"] = servoTestSweepFaultReasonName(compassData.sweepFaultReason);
    json["sweep_samples"] = compassData.sweepSamples;
    json["fit_attempted"] = compassData.fitAttempted;
    json["calibration_ready"] = compassData.calibrationReady;
    json["calibration_saved"] = compassData.calibrationSaved;
    json["site_sweep"] = compassData.siteSweep;
    json["setup_active"] = compassData.setupActive;
    json["pulse_calibration_state"] = compassData.pulseCalibrationState;
    json["pulse_calibration_candidate_us"] = compassData.pulseCalibrationCandidateUs;
    json["pulse_calibration_samples"] = compassData.pulseCalibrationSamples;
    json["pulse_calibration_displacement_mdeg"] = compassData.pulseCalibrationDisplacementMdeg;
    json["pulse_calibration_fault"] = compassData.pulseCalibrationFault;
    json["pulse_calibration_settling"] = compassData.pulseCalibrationSettling;
    json["pulse_calibration_supported"] = compassData.pulseCalibrationSupported;
    json["pulse_calibration_active"] = vrxModule.isServoTestPulseCalibrationBusy();
    json["pulse_calibration_valid"] = compassData.pulseCalibrationValid;
    json["left_start_pulse_us"] = compassData.leftStartPulseUs;
    json["right_start_pulse_us"] = compassData.rightStartPulseUs;
    const aat_pan_compass_calibration_t savedCalibration = config.GetAatPanCompassCalibration();
    json["saved_calibration_valid"] = savedCalibration.valid == 1;
    json["saved_north_aligned"] = config.GetAatPanCompassNorthAligned();
    json["mounting_reversed"] = config.IsAatPanCompassReversed();
    json["mounting_verification_pending"] = config.IsAatPanCompassMountingPending();
#if defined(AAT_COMPASS_TRACKING)
    if (vrxModule.getCompassMountingStart() >= 0)
        json["mounting_start_deg"] = vrxModule.getCompassMountingStart();
#endif
    if (savedCalibration.valid == 1)
    {
        json["saved_offset_x"] = savedCalibration.offsetX;
        json["saved_offset_y"] = savedCalibration.offsetY;
        json["saved_scale_x"] = savedCalibration.scaleXPermille / 1000.0f;
        json["saved_scale_y"] = savedCalibration.scaleYPermille / 1000.0f;
        json["saved_heading_offset"] = savedCalibration.headingOffsetDegrees;
    }
    if (compassData.fitAttempted)
    {
        const AatCompassCalibration::Result &fit = compassData.calibration;
        json["fit_valid"] = fit.valid;
        json["fit_failure"] = compassFitFailureName(fit.failure);
        json["fit_samples"] = fit.sampleCount;
        json["span_x"] = fit.spanX;
        json["span_y"] = fit.spanY;
        json["offset_x"] = fit.calibration.offsetX;
        json["offset_y"] = fit.calibration.offsetY;
        json["scale_x"] = fit.calibration.scaleX;
        json["scale_y"] = fit.calibration.scaleY;
        json["covered_sectors"] = fit.coveredSectors;
        json["field_magnitude_min"] = fit.fieldMagnitudeMin;
        json["field_magnitude_max"] = fit.fieldMagnitudeMax;
        json["field_magnitude_ratio"] = fit.fieldMagnitudeRatio;
        json["field_magnitude_warning"] = fit.fieldMagnitudeWarning;
        json["return_closure_error_deg"] = fit.returnClosureErrorDegrees;
    }
    String body;
    serializeJson(json, body);
    request->send(200, "application/json", body);
}

static constexpr uint32_t AAT_SWEEP_EXPORT_CHUNK_MAX = 32;

void WebAatSweep(AsyncWebServerRequest *request)
{
    if (request->args() > 2)
    {
        request->send(400, "text/plain", "Only start and count query parameters are supported");
        return;
    }
    for (size_t index = 0; index < request->args(); ++index)
    {
        const String name = request->argName(index);
        if (name != "start" && name != "count")
        {
            request->send(400, "text/plain", "Only start and count query parameters are supported");
            return;
        }
    }

    uint32_t start = 0;
    uint32_t count = AAT_SWEEP_EXPORT_CHUNK_MAX;
    if (!parseUnsignedRequestArg(request, "start", 0, &start)
        || !parseUnsignedRequestArg(request, "count", AAT_SWEEP_EXPORT_CHUNK_MAX, &count)
        || count == 0 || count > AAT_SWEEP_EXPORT_CHUNK_MAX)
    {
        request->send(400, "text/plain", "Use a nonnegative start and count from 1 to 32");
        return;
    }

    const AatModule::ServoTestCompassData compassData = vrxModule.getServoTestCompassData();
    if (!compassData.setupActive
        || compassData.sweepState != AatModule::ServoTestSweepState::Complete)
    {
        request->send(409, "text/plain", "Raw sweep export is available only after a completed setup sweep");
        return;
    }

    const uint32_t sampleCount = compassData.sweepSamples;
    if (start > sampleCount)
    {
        request->send(416, "text/plain", "Sweep sample start is out of range");
        return;
    }

    const uint32_t chunkCount = min(count, sampleCount - start);
    ArduinoJson::JsonDocument json;
    json["schema"] = "aat-sweep-v1";
    json["sweep_state"] = "complete";
    json["sample_count"] = sampleCount;
    json["start_index"] = start;
    json["next_index"] = start + chunkCount;
    json["elapsed_units"] = "milliseconds from sweep start";
    if (compassData.fitAttempted)
    {
        json["fit_valid"] = compassData.calibration.valid;
        json["covered_sectors"] = compassData.calibration.coveredSectors;
        json["field_magnitude_min"] = compassData.calibration.fieldMagnitudeMin;
        json["field_magnitude_max"] = compassData.calibration.fieldMagnitudeMax;
        json["field_magnitude_ratio"] = compassData.calibration.fieldMagnitudeRatio;
        json["return_closure_error_deg"] = compassData.calibration.returnClosureErrorDegrees;
    }

    ArduinoJson::JsonArray samples = json["samples"].to<ArduinoJson::JsonArray>();
    for (uint32_t index = start; index < start + chunkCount; ++index)
    {
        AatCompassCalibration::Sample sample = {};
        if (!vrxModule.getServoTestCompassSweepSample(static_cast<uint16_t>(index), &sample))
        {
            request->send(409, "text/plain", "Sweep samples are not stable for export");
            return;
        }
        ArduinoJson::JsonArray row = samples.add<ArduinoJson::JsonArray>();
        row.add(index);
        row.add(sample.elapsedMs);
        row.add(sample.x);
        row.add(sample.y);
        row.add(sample.z);
    }

    String body;
    serializeJson(json, body);
    AsyncWebServerResponse *response = request->beginResponse(200, "application/json", body);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
}
#endif

#if defined(AAT_PERF_DEBUG)
void WebAatPerf(AsyncWebServerRequest *request)
{
    if (request->hasArg("reset") && request->arg("reset").toInt() == 1)
        vrxModule.resetPerfStats();

    const AatPerfStats stats = vrxModule.getPerfStats();
    ArduinoJson::JsonDocument json;
    json["loop_intervals"] = stats.loopIntervals;
    json["loop_period_min_us"] = stats.loopPeriodMinUs;
    json["loop_period_max_us"] = stats.loopPeriodMaxUs;
    json["loop_period_jitter_range_us"] = stats.loopPeriodMaxUs - stats.loopPeriodMinUs;
    json["loop_duration_max_us"] = stats.loopDurationMaxUs;
    json["servo_updates"] = stats.servoUpdates;
    json["servo_period_min_us"] = stats.servoPeriodMinUs;
    json["servo_period_max_us"] = stats.servoPeriodMaxUs;
    json["servo_period_jitter_range_us"] = stats.servoPeriodMaxUs - stats.servoPeriodMinUs;
    json["gps_active"] = vrxModule.isGpsActive();
    json["home_set"] = vrxModule.isHomeSet();
    json["binding"] = connectionState == binding;
    json["bound_marker"] = config.GetBootCount() == 0xA7;
    json["connection_state"] = connectionState == binding ? "binding" :
        (connectionState == running ? "running" :
        (connectionState == wifiUpdate ? "wifi_update" : "starting"));
    json["uid_provisioned"] = firmwareOptions.hasUID;
    json["espnow_datagrams"] = stats.espNowDatagrams;
    json["espnow_msp_packets"] = stats.espNowMspPackets;
    json["espnow_mac_accepted"] = stats.espNowMacAccepted;
    json["espnow_mac_rejected"] = stats.espNowMacRejected;
    json["espnow_queue_drops"] = stats.espNowQueueDrops;
    json["crsf_telemetry_packets"] = stats.crsfTelemetryPackets;
    json["gps_frames"] = stats.gpsFrames;

    String body;
    serializeJson(json, body);
    request->send(200, "application/json", body);
}
#endif

#if defined(AAT_COMPASS_DEBUG) && !defined(AAT_COMPASS_TRACKING)
void WebAatCompass(AsyncWebServerRequest *request)
{
#if defined(AAT_I2C_TRACE)
    request->send(409, "text/plain", "I2C trace owns sensor reads; capture serial output");
    return;
#endif
    static AatHmc5883l compass(Wire);
    static bool configured = false;
    static uint32_t sampleCount = 0;
    static uint32_t lastSampleMs = 0;
    static AatHmc5883l::RawSample lastSample = {};
    static uint8_t settings[3] = {};
    static bool settingsRead = false;
    uint8_t identity[3] = {};
    const bool identityRead = compass.readIdentity(identity);
    const bool sensorDetected = identityRead && compass.hasHmcIdentity(identity);
    if (sensorDetected && !configured)
        configured = compass.configure();

    AatHmc5883l::RawSample sample = {};
    uint8_t status = 0;
    const bool newSample = configured && compass.readRaw(&sample, &status);
    if (sample.statusAfterReadValid)
        lastSample = sample;
    if (newSample)
    {
        ++sampleCount;
        lastSampleMs = millis();
    }
    if (configured)
        settingsRead = compass.readConfiguration(settings);

    char identityText[4] = {};
    if (identityRead)
    {
        identityText[0] = static_cast<char>(identity[0]);
        identityText[1] = static_cast<char>(identity[1]);
        identityText[2] = static_cast<char>(identity[2]);
    }

    ArduinoJson::JsonDocument json;
    json["identity_read"] = identityRead;
    json["identity"] = identityText;
    json["sensor_detected"] = sensorDetected;
    json["configured"] = configured;
    json["sample_available"] = sampleCount > 0;
    json["sample_count"] = sampleCount;
    json["sample_age_ms"] = sampleCount > 0 ? millis() - lastSampleMs : UINT32_MAX;
    json["hmc_status"] = status;
    json["hmc_status_after_read"] = lastSample.statusAfterRead;
    json["hmc_status_after_read_valid"] = lastSample.statusAfterReadValid;
    json["data_ready"] = (status & 0x01) != 0;
    json["data_locked"] = (status & 0x02) != 0;
    json["data_locked_after_read"] = (lastSample.statusAfterRead & 0x02) != 0;
    json["configuration_read"] = settingsRead;
    if (settingsRead)
    {
        json["config_a"] = settings[0];
        json["config_b"] = settings[1];
        json["mode"] = settings[2];
    }
    if (sampleCount > 0)
    {
        json["x"] = lastSample.x;
        json["y"] = lastSample.y;
        json["z"] = lastSample.z;
        json["raw_x_msb"] = lastSample.registers[0];
        json["raw_x_lsb"] = lastSample.registers[1];
        json["raw_z_msb"] = lastSample.registers[2];
        json["raw_z_lsb"] = lastSample.registers[3];
        json["raw_y_msb"] = lastSample.registers[4];
        json["raw_y_lsb"] = lastSample.registers[5];
    }

    String body;
    serializeJson(json, body);
    request->send(200, "application/json", body);
}
#endif

void WebAatInit(AsyncWebServer &server)
{
    server.on("/aatconfig", WebAatConfig);
#if defined(AAT_SERVO_TEST) || defined(AAT_COMPASS_TRACKING)
    server.on("/aatservo", HTTP_GET, WebAatServo);
    server.on("/aatservo", HTTP_POST, WebAatServo);
    server.on("/aatsweep", HTTP_GET, WebAatSweep);
#endif
#if defined(AAT_COMPASS_DEBUG) && !defined(AAT_COMPASS_TRACKING)
    server.on("/aatcompass", WebAatCompass);
#endif
#if defined(AAT_PERF_DEBUG)
    server.on("/aatperf", WebAatPerf);
#endif
}

#endif /* defined(AAT_BACKPACK) */

