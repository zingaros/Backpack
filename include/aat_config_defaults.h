#pragma once

#include <stdint.h>

namespace AatConfigDefaults
{
constexpr uint16_t PAN_SERVO_NEUTRAL_US = 1520;
constexpr uint8_t PAN_DEADBAND_DEFAULT_DEGREES = 3;
constexpr uint8_t PAN_DEADBAND_MIN_DEGREES = 1;
constexpr uint8_t PAN_DEADBAND_MAX_DEGREES = 30;
constexpr uint16_t PAN_LEFT_MAX_OFFSET_US = 40;
constexpr uint16_t PAN_RIGHT_MAX_OFFSET_US = 45;
constexpr uint16_t PAN_MIN_PULSE_US = 1000;
constexpr uint16_t PAN_MAX_PULSE_US = 2000;
constexpr uint16_t PAN_PULSE_CALIBRATION_STEP_US = 5;

template <typename AatSettings, typename BatterySettings>
inline void apply(AatSettings &aat, BatterySettings &battery)
{
    aat.satelliteHomeMin = 5;
    aat.project = 0xff;
    aat.servoSmooth = 5;
    aat.centerDir = 0;
    aat.azimuthServoFastFlip = 1;
    aat.servoEndpoints[0].low = 500;
    aat.servoEndpoints[0].high = 2500;
    aat.servoEndpoints[1].low = 1000;
    aat.servoEndpoints[1].high = 2000;
    battery.scale = 292;
    battery.offset = -2;
}
}