#pragma once

#include <atomic>
#include <stdint.h>

namespace AatServo
{
enum Mode : uint8_t { TwoToOne, Clip180, Flip180, CompassContinuous };

enum class CompassPanOperatingState : uint8_t
{
    Disabled,
    Setup,
    StartupAlignment,
    Tracking
};

enum class ContinuousPanBlockReason : uint8_t
{
    None,
    Setup,
    StartupAlignment,
    CompassInvalid,
    ReferenceUnavailable,
    GpsUnavailable,
    SafetyFault
};

inline bool hasDebouncedPressEdge(bool wasPressed, bool rawPressed,
    uint32_t stableMs, uint32_t debounceMs)
{
    return !wasPressed && rawPressed && stableMs >= debounceMs;
}

inline bool shouldStartSweepOnReferencePress(bool armed, bool referencePressed)
{
    return armed && referencePressed;
}

inline bool isSweepTimedOut(bool sweeping, uint32_t elapsedMs, uint32_t timeoutMs)
{
    return sweeping && elapsedMs >= timeoutMs;
}

inline bool shouldCompleteSweepOnReferenceRelease(bool sweeping, bool referencePressed)
{
    return sweeping && !referencePressed;
}

inline uint16_t getCompassSetupSweepPulse(bool sweeping, uint16_t sweepPulseUs,
    uint16_t neutralPulseUs)
{
    return sweeping ? sweepPulseUs : neutralPulseUs;
}

inline CompassPanOperatingState getCompassPanOperatingState(bool modeEnabled,
    bool setupRequested, bool compassFitValid, bool northAligned,
    bool startupAlignmentComplete)
{
    if (!modeEnabled)
        return CompassPanOperatingState::Disabled;
    if (setupRequested || !compassFitValid)
        return CompassPanOperatingState::Setup;
    if (!northAligned || !startupAlignmentComplete)
        return CompassPanOperatingState::StartupAlignment;
    return CompassPanOperatingState::Tracking;
}

inline bool shouldEnterCompassPanSetup(bool modeEnabled, bool compassFitValid,
    bool pulseCalibrationValid, bool wifiUpdateMode)
{
    return modeEnabled && (!compassFitValid || !pulseCalibrationValid || wifiUpdateMode);
}

inline bool canResetCompassCalibration(bool setupActive, bool sweepActive,
    bool referencePressed, uint16_t requestedPulseUs, uint16_t appliedPulseUs,
    uint16_t neutralPulseUs)
{
    return setupActive && !sweepActive && !referencePressed
        && requestedPulseUs == neutralPulseUs && appliedPulseUs == neutralPulseUs;
}

inline uint16_t getServoTestPulse(bool sweepActive, uint16_t requestedPulseUs,
    uint16_t neutralUs, uint32_t nowMs, uint32_t lastCommandMs, uint32_t commandTimeoutMs)
{
    const uint32_t elapsedMs = nowMs - lastCommandMs;
    if (!sweepActive && requestedPulseUs != neutralUs
        && elapsedMs < 0x80000000UL && elapsedMs >= commandTimeoutMs)
        return neutralUs;
    return requestedPulseUs;
}

inline bool clearServoTestPulseRequestIfUnchanged(std::atomic<uint16_t> &requestedPulseUs,
    uint16_t observedPulseUs, uint16_t neutralUs)
{
    return requestedPulseUs.compare_exchange_strong(observedPulseUs, neutralUs);
}

inline bool isManualPanTestPulseAllowed(uint16_t pulseUs, uint16_t neutralUs,
    uint16_t minimumTestPulseUs, uint16_t maximumTestPulseUs,
    bool referenceValid, bool compassFreshAndValid, int32_t currentPanDegrees,
    int32_t maximumTestPanDegrees)
{
    if (pulseUs == neutralUs)
        return true;
    if (!referenceValid || !compassFreshAndValid ||
        minimumTestPulseUs > neutralUs || maximumTestPulseUs < neutralUs ||
        pulseUs < minimumTestPulseUs || pulseUs > maximumTestPulseUs ||
        maximumTestPanDegrees < 0)
        return false;

    return currentPanDegrees >= -maximumTestPanDegrees
        && currentPanDegrees <= maximumTestPanDegrees;
}

inline int32_t shortestPathDelta(int32_t currentDegrees, int32_t targetDegrees)
{
    currentDegrees %= 360;
    targetDegrees %= 360;
    if (currentDegrees < 0)
        currentDegrees += 360;
    if (targetDegrees < 0)
        targetDegrees += 360;

    int32_t delta = targetDegrees - currentDegrees;
    if (delta > 180)
        delta -= 360;
    else if (delta < -180)
        delta += 360;
    else if (delta == -180)
        delta = 180;
    return delta;
}

struct PanCommand
{
    int32_t currentPanDegrees;
    int32_t targetPanDegrees;
    int32_t errorDegrees;
};

class TrackingPanMedian
{
public:
    void reset()
    {
        _count = 0;
        _next = 0;
    }

    bool update(uint32_t sampleAtMs, int32_t panDegrees)
    {
        if (_count != 0 && sampleAtMs == _sampleAtMs)
            return _count == 3;

        _sampleAtMs = sampleAtMs;
        _samples[_next] = panDegrees;
        _next = (_next + 1) % 3;
        if (_count < 3)
            ++_count;
        if (_count < 3)
            return false;

        int32_t a = _samples[0], b = _samples[1], c = _samples[2];
        if (a > b) { const int32_t tmp = a; a = b; b = tmp; }
        if (b > c) { const int32_t tmp = b; b = c; c = tmp; }
        if (a > b) b = a;
        _panDegrees = b;
        return true;
    }

    int32_t panDegrees() const { return _panDegrees; }

private:
    int32_t _samples[3] = {};
    int32_t _panDegrees = 0;
    uint32_t _sampleAtMs = 0;
    uint8_t _count = 0;
    uint8_t _next = 0;
};

inline uint16_t constrainFilteredPanPulse(uint16_t pulseUs, uint16_t neutralUs,
    int32_t rawErrorDegrees, int32_t deadbandDegrees)
{
    if ((pulseUs > neutralUs && rawErrorDegrees <= deadbandDegrees)
        || (pulseUs < neutralUs && rawErrorDegrees >= -deadbandDegrees))
        return neutralUs;
    return pulseUs;
}

struct PanReference
{
    int32_t homeHeadingDegrees;
    bool established;
};

enum class PanSafetyState : uint8_t
{
    AwaitButtonRelease,
    Unreferenced,
    Referenced,
    Fault
};

enum class CompassStartupAlignmentAction : uint8_t
{
    WaitForCompass,
    WaitForOperator,
    CaptureNorthAndHome,
    CaptureHome,
    Complete
};

inline CompassStartupAlignmentAction getCompassStartupAlignmentAction(
    bool alignmentComplete, bool northAligned, bool compassFreshAndValid,
    bool buttonPressed)
{
    if (alignmentComplete)
        return CompassStartupAlignmentAction::Complete;
    if (!compassFreshAndValid)
        return CompassStartupAlignmentAction::WaitForCompass;
    if (!buttonPressed)
        return CompassStartupAlignmentAction::WaitForOperator;
    if (northAligned)
        return CompassStartupAlignmentAction::CaptureHome;
    return CompassStartupAlignmentAction::CaptureNorthAndHome;
}

inline bool establishPanReference(PanReference *reference, bool operatorConfirmed,
    bool actuatorNeutral, bool compassFreshAndValid, int32_t currentHeadingDegrees)
{
    if (reference == nullptr)
        return false;

    reference->homeHeadingDegrees = 0;
    reference->established = false;
    if (!operatorConfirmed || !actuatorNeutral || !compassFreshAndValid)
        return false;

    reference->homeHeadingDegrees = currentHeadingDegrees;
    reference->established = true;
    return true;
}

inline bool calculatePanCommand(bool homed, bool compassValid, int32_t homeHeading,
    int32_t currentHeading, int32_t targetHeading, PanCommand *command)
{
    if (command == nullptr)
        return false;

    command->currentPanDegrees = 0;
    command->targetPanDegrees = 0;
    command->errorDegrees = 0;
    if (!homed || !compassValid)
        return false;

    const int32_t currentPan = shortestPathDelta(homeHeading, currentHeading);
    if (currentPan < -170 || currentPan > 170)
        return false;

    int32_t targetPan = shortestPathDelta(homeHeading, targetHeading);
    if (targetPan < -170)
        targetPan = -170;
    else if (targetPan > 170)
        targetPan = 170;

    command->currentPanDegrees = currentPan;
    command->targetPanDegrees = targetPan;
    command->errorDegrees = targetPan - currentPan;
    return true;
}

inline bool calculatePanCommand(const PanReference &reference, bool compassFreshAndValid,
    int32_t currentHeading, int32_t targetHeading, PanCommand *command)
{
    return calculatePanCommand(reference.established, compassFreshAndValid,
        reference.homeHeadingDegrees, currentHeading, targetHeading, command);
}

inline bool calculateContinuousPanPulse(const PanReference &reference,
    bool compassFreshAndValid, int32_t currentHeading, int32_t targetHeading,
    uint16_t neutralPulseUs, uint16_t leftPulseUs, uint16_t rightPulseUs,
    int32_t deadbandDegrees, uint16_t *pulseUs, PanCommand *command)
{
    if (pulseUs == nullptr)
        return false;
    *pulseUs = neutralPulseUs;
    if (deadbandDegrees < 0 || leftPulseUs >= neutralPulseUs
        || rightPulseUs <= neutralPulseUs
        || !calculatePanCommand(reference, compassFreshAndValid,
            currentHeading, targetHeading, command))
        return false;

    if (command->errorDegrees > deadbandDegrees)
        *pulseUs = rightPulseUs;
    else if (command->errorDegrees < -deadbandDegrees)
        *pulseUs = leftPulseUs;
    return true;
}

inline uint16_t continuousPanSpeedPermille(uint8_t servoSmooth)
{
    if (servoSmooth > 9)
        return 0;
    return static_cast<uint16_t>(500 + (9 - servoSmooth) * 500 / 9);
}

inline bool calculateContinuousPanPulseProfile(const PanReference &reference,
    bool compassFreshAndValid, int32_t currentHeading, int32_t targetHeading,
    uint16_t neutralPulseUs, uint16_t leftStartPulseUs, uint16_t leftPulseUs,
    uint16_t rightStartPulseUs, uint16_t rightPulseUs,
    int32_t deadbandDegrees, uint8_t servoSmooth, uint16_t *pulseUs,
    PanCommand *command)
{
    if (pulseUs == nullptr)
        return false;
    *pulseUs = neutralPulseUs;

    const uint16_t speedPermille = continuousPanSpeedPermille(servoSmooth);
    if (speedPermille == 0 || deadbandDegrees < 0
        || leftStartPulseUs >= neutralPulseUs || leftStartPulseUs < leftPulseUs
        || rightStartPulseUs <= neutralPulseUs || rightStartPulseUs > rightPulseUs
        || leftPulseUs >= neutralPulseUs || rightPulseUs <= neutralPulseUs
        || !calculatePanCommand(reference, compassFreshAndValid,
            currentHeading, targetHeading, command))
        return false;

    const int32_t errorMagnitude =
        command->errorDegrees < 0 ? -command->errorDegrees : command->errorDegrees;
    if (errorMagnitude <= deadbandDegrees)
        return true;

    constexpr int32_t APPROACH_BAND_DEGREES = 45;
    const int32_t approachDegrees = errorMagnitude - deadbandDegrees;
    const int32_t approachPermille = approachDegrees >= APPROACH_BAND_DEGREES
        ? 1000
        : approachDegrees * 1000 / APPROACH_BAND_DEGREES;
    const uint32_t outputPermille =
        static_cast<uint32_t>(speedPermille) * approachPermille / 1000U;

    if (command->errorDegrees > 0)
    {
        const uint32_t startOffsetUs = rightStartPulseUs - neutralPulseUs;
        const uint32_t offsetUs = startOffsetUs
            + static_cast<uint32_t>(rightPulseUs - rightStartPulseUs)
                * outputPermille / 1000U;
        *pulseUs = neutralPulseUs + static_cast<uint16_t>(offsetUs);
    }
    else
    {
        const uint32_t startOffsetUs = neutralPulseUs - leftStartPulseUs;
        const uint32_t capOffsetUs = startOffsetUs
            + static_cast<uint32_t>(leftStartPulseUs - leftPulseUs)
                * outputPermille / 1000U;
        *pulseUs = neutralPulseUs - static_cast<uint16_t>(capOffsetUs);
    }
    return true;
}

inline bool isCompassConfigurationValid(bool readValid, uint8_t actualConfigA,
    uint8_t requestedConfigA)
{
    return readValid && actualConfigA == requestedConfigA;
}

inline bool isPanFeedbackFresh(uint32_t sampleAtMs, uint32_t nowMs,
    uint32_t timeoutMs)
{
    return nowMs - sampleAtMs <= timeoutMs;
}

inline bool areCompassPanSampleInputsValid(bool compassDetected, bool headingValid,
    bool sampleFresh, bool calibrationValid, bool configMatchesRequested,
    bool triggerStatusValid, bool triggerLockClear)
{
    return compassDetected && headingValid && sampleFresh && calibrationValid
        && configMatchesRequested && triggerStatusValid && triggerLockClear;
}

inline bool isContinuousPanGpsFresh(bool homeSet, uint32_t averageIntervalMs,
    uint32_t fixAgeMs)
{
    constexpr uint32_t maximumFixAgeMs = 3000U;
    return homeSet && averageIntervalMs != 0 && fixAgeMs < maximumFixAgeMs
        && (averageIntervalMs >= maximumFixAgeMs / 3U || fixAgeMs <= 3U * averageIntervalMs);
}

inline bool areContinuousPanCommandInputsValid(bool gpsFresh,
    bool compassFreshAndValid, bool panReferenced)
{
    return gpsFresh && compassFreshAndValid && panReferenced;
}

inline ContinuousPanBlockReason getContinuousPanBlockReason(bool setupActive,
    bool startupAlignmentComplete, bool compassFreshAndValid, bool gpsFresh,
    PanSafetyState safetyState)
{
    if (setupActive)
        return ContinuousPanBlockReason::Setup;
    if (!startupAlignmentComplete)
        return ContinuousPanBlockReason::StartupAlignment;
    if (safetyState == PanSafetyState::Fault)
        return ContinuousPanBlockReason::SafetyFault;
    if (!compassFreshAndValid)
        return ContinuousPanBlockReason::CompassInvalid;
    if (safetyState != PanSafetyState::Referenced)
        return ContinuousPanBlockReason::ReferenceUnavailable;
    if (!gpsFresh)
        return ContinuousPanBlockReason::GpsUnavailable;
    return ContinuousPanBlockReason::None;
}

class PanSafetyController
{
public:
    PanSafetyState state() const { return _state; }
    const PanReference &reference() const { return _reference; }

    void invalidate()
    {
        enterFault();
    }

    bool updateReference(bool buttonPressed, bool actuatorNeutral,
        bool compassFreshAndValid, int32_t currentHeadingDegrees)
    {
        if (_state == PanSafetyState::Referenced)
        {
            PanCommand command = {};
            return calculateCommand(compassFreshAndValid, currentHeadingDegrees,
                currentHeadingDegrees, &command);
        }

        if (_state == PanSafetyState::AwaitButtonRelease)
        {
            if (!buttonPressed)
                _state = PanSafetyState::Unreferenced;
            return false;
        }

        if (_state == PanSafetyState::Fault)
        {
            if (!buttonPressed)
                _state = PanSafetyState::Unreferenced;
            return false;
        }

        if (!buttonPressed)
            return false;
        if (!actuatorNeutral || !compassFreshAndValid)
        {
            enterFault();
            return false;
        }

        if (!establishPanReference(&_reference, true, true, true, currentHeadingDegrees))
        {
            enterFault();
            return false;
        }
        _lastPanDegrees = 0;
        _state = PanSafetyState::Referenced;
        return true;
    }

    bool calculateCommand(bool compassFreshAndValid, int32_t currentHeadingDegrees,
        int32_t targetHeadingDegrees, PanCommand *command)
    {
        if (command == nullptr)
            return false;
        if (_state != PanSafetyState::Referenced)
            return calculatePanCommand(false, compassFreshAndValid, 0,
                currentHeadingDegrees, targetHeadingDegrees, command);

        if (!calculatePanCommand(_reference, compassFreshAndValid,
            currentHeadingDegrees, targetHeadingDegrees, command))
        {
            enterFault();
            return false;
        }
        const int32_t panDelta = command->currentPanDegrees - _lastPanDegrees;
        if (panDelta < -180 || panDelta > 180)
        {
            *command = {};
            enterFault();
            return false;
        }
        _lastPanDegrees = command->currentPanDegrees;
        return true;
    }

private:
    void enterFault()
    {
        _reference.homeHeadingDegrees = 0;
        _reference.established = false;
        _lastPanDegrees = 0;
        _state = PanSafetyState::Fault;
    }

    PanReference _reference = {};
    int32_t _lastPanDegrees = 0;
    PanSafetyState _state = PanSafetyState::AwaitButtonRelease;
};

inline int32_t mapValue(int32_t value, int32_t inputLow, int32_t inputHigh,
    int32_t outputLow, int32_t outputHigh)
{
    return (value - inputLow) * (outputHigh - outputLow) / (inputHigh - inputLow) + outputLow;
}

inline void applyMode(int32_t bearing, int32_t elevation, Mode mode,
    const int32_t servoLow[2], const int32_t servoHigh[2], int32_t servoPosition[2])
{
    if (mode == CompassContinuous)
    {
        servoPosition[1] = mapValue(elevation, 0, 90, servoLow[1], servoHigh[1]);
        return;
    }

    if (mode == TwoToOne)
    {
        servoPosition[0] = mapValue(bearing, -180, 179, servoLow[0], servoHigh[0]);
        servoPosition[1] = mapValue(elevation, 0, 90, servoLow[1], servoHigh[1]);
        return;
    }

    if (mode == Clip180)
    {
        if (bearing < -90)
            bearing = -90;
        else if (bearing > 90)
            bearing = 90;
        servoPosition[0] = mapValue(bearing, -90, 90, servoLow[0], servoHigh[0]);
        servoPosition[1] = mapValue(elevation, 0, 90, servoLow[1], servoHigh[1]);
        return;
    }

    if (mode == Flip180)
    {
        if (bearing < -90)
        {
            bearing = -180 - bearing;
            elevation = 180 - elevation;
        }
        else if (bearing > 90)
        {
            bearing = 180 - bearing;
            elevation = 180 - elevation;
        }
        servoPosition[0] = mapValue(bearing, -90, 90, servoLow[0], servoHigh[0]);
        servoPosition[1] = mapValue(elevation, 0, 180, servoLow[1], servoHigh[1]);
    }
}
}