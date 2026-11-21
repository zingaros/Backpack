#pragma once

#include <stdint.h>
#include "aat_config_defaults.h"

namespace AatConfigValidation
{
inline bool areServoEndpointsValid(uint16_t azimuthLow, uint16_t azimuthHigh,
    uint16_t elevationLow, uint16_t elevationHigh)
{
    return azimuthLow != azimuthHigh && elevationLow != elevationHigh;
}

inline bool isPanServoNeutralValid(uint16_t pulseUs)
{
    return pulseUs >= 1450 && pulseUs <= 1550;
}

inline bool isPanDeadbandValid(int32_t deadbandDegrees)
{
    return deadbandDegrees >= AatConfigDefaults::PAN_DEADBAND_MIN_DEGREES
        && deadbandDegrees <= AatConfigDefaults::PAN_DEADBAND_MAX_DEGREES;
}

inline bool isPanPulseEnvelopeValid(uint32_t neutralUs,
    uint32_t leftMaxOffsetUs, uint32_t rightMaxOffsetUs)
{
    return neutralUs >= 1450 && neutralUs <= 1550
        && leftMaxOffsetUs >= AatConfigDefaults::PAN_PULSE_CALIBRATION_STEP_US
        && rightMaxOffsetUs >= AatConfigDefaults::PAN_PULSE_CALIBRATION_STEP_US
        && leftMaxOffsetUs <= neutralUs - AatConfigDefaults::PAN_MIN_PULSE_US
        && rightMaxOffsetUs <= AatConfigDefaults::PAN_MAX_PULSE_US - neutralUs;
}

inline bool isPanPulseCalibrationValid(uint16_t neutralUs,
    uint16_t leftStartPulseUs, uint16_t rightStartPulseUs,
    uint16_t leftMaxOffsetUs, uint16_t rightMaxOffsetUs)
{
    return isPanPulseEnvelopeValid(neutralUs, leftMaxOffsetUs, rightMaxOffsetUs)
        && leftStartPulseUs < neutralUs
        && leftStartPulseUs >= neutralUs - leftMaxOffsetUs
        && rightStartPulseUs > neutralUs
        && rightStartPulseUs <= neutralUs + rightMaxOffsetUs;
}

template <typename Calibration>
inline bool isPanCompassCalibrationValid(const Calibration &calibration)
{
    return calibration.valid == 1 &&
        calibration.scaleXPermille >= 250 && calibration.scaleXPermille <= 4000 &&
        calibration.scaleYPermille >= 250 && calibration.scaleYPermille <= 4000 &&
        calibration.headingOffsetDegrees >= -180 && calibration.headingOffsetDegrees <= 180;
}

inline bool isPanCompassNorthAlignmentValid(bool compassCalibrationValid)
{
    return compassCalibrationValid;
}
}