#include <unity.h>
#include <atomic>
#include <stdio.h>

#include "aat_navigation.h"
#include "aat_compass.h"
#include "aat_compass_calibration.h"
#include "aat_projection.h"
#include "aat_servo.h"
#include "aat_pulse_calibration.h"
#include "aat_config_defaults.h"
#include "aat_config_migration.h"
#include "aat_config_validation.h"

void setUp(void) {}
void tearDown(void) {}

void test_eastward_bearing_and_distance()
{
    uint32_t distance = 0;
    uint32_t azimuth = 0;
    AatNavigation::calculateDistanceAndAzimuth(0, 0, 0, 10000000, &distance, &azimuth);

    TEST_ASSERT_EQUAL_UINT32(90, azimuth);
    TEST_ASSERT_UINT32_WITHIN(1, 111194, distance);
}

void test_westward_bearing_is_normalized()
{
    uint32_t azimuth = 0;
    AatNavigation::calculateDistanceAndAzimuth(0, 0, 0, -10000000, nullptr, &azimuth);

    TEST_ASSERT_EQUAL_UINT32(270, azimuth);
}

void test_antimeridian_distance_uses_wide_longitude_delta()
{
    uint32_t distance = 0;
    uint32_t azimuth = 0;
    AatNavigation::calculateDistanceAndAzimuth(0, 1799000000, 0, -1799000000, &distance, &azimuth);

    TEST_ASSERT_UINT32_WITHIN(2, 22239, distance);
    TEST_ASSERT_EQUAL_UINT32(90, azimuth);
}

void test_elevation()
{
    TEST_ASSERT_EQUAL_INT32(45, AatNavigation::calculateElevation(1000, 1000));
    TEST_ASSERT_EQUAL_INT32(-45, AatNavigation::calculateElevation(1000, -1000));
}

void test_continuous_pan_uses_shortest_path_across_zero()
{
    TEST_ASSERT_EQUAL_INT32(2, AatServo::shortestPathDelta(359, 1));
    TEST_ASSERT_EQUAL_INT32(-2, AatServo::shortestPathDelta(1, 359));
}

void test_continuous_pan_uses_positive_direction_for_half_turn_tie()
{
    TEST_ASSERT_EQUAL_INT32(180, AatServo::shortestPathDelta(0, 180));
    TEST_ASSERT_EQUAL_INT32(180, AatServo::shortestPathDelta(180, 0));
}

void test_active_servo_sweep_bypasses_manual_pulse_timeout()
{
    TEST_ASSERT_EQUAL_UINT16(2000, AatServo::getServoTestPulse(true, 2000, 1520, 5000, 1000, 1000));
    TEST_ASSERT_EQUAL_UINT16(1520, AatServo::getServoTestPulse(false, 1600, 1520, 2000, 1000, 1000));
    TEST_ASSERT_EQUAL_UINT16(1600, AatServo::getServoTestPulse(false, 1600, 1520, 1999, 1000, 1000));
}

void test_manual_pulse_timeout_handles_future_and_wrapped_timestamps()
{
    TEST_ASSERT_EQUAL_UINT16(1565,
        AatServo::getServoTestPulse(false, 1565, 1520, 100, 101, 1000));
    TEST_ASSERT_EQUAL_UINT16(1565,
        AatServo::getServoTestPulse(false, 1565, 1520, 400, UINT32_MAX - 99, 501));
    TEST_ASSERT_EQUAL_UINT16(1520,
        AatServo::getServoTestPulse(false, 1565, 1520, 400, UINT32_MAX - 99, 500));
}

void test_manual_pulse_uses_three_second_timeout()
{
    TEST_ASSERT_EQUAL_UINT16(1565,
        AatServo::getServoTestPulse(false, 1565, 1520, 3099, 100, 3000));
    TEST_ASSERT_EQUAL_UINT16(1520,
        AatServo::getServoTestPulse(false, 1565, 1520, 3100, 100, 3000));
}

void test_manual_pulse_clear_preserves_a_newer_request()
{
    std::atomic<uint16_t> requestedPulseUs(1520);
    const uint16_t observedPulseUs = requestedPulseUs.load();
    requestedPulseUs.store(1565);

    TEST_ASSERT_FALSE(AatServo::clearServoTestPulseRequestIfUnchanged(
        requestedPulseUs, observedPulseUs, 1520));
    TEST_ASSERT_EQUAL_UINT16(1565, requestedPulseUs.load());
}

void test_manual_pulse_clear_retires_unchanged_timed_out_request()
{
    std::atomic<uint16_t> requestedPulseUs(1565);

    TEST_ASSERT_TRUE(AatServo::clearServoTestPulseRequestIfUnchanged(
        requestedPulseUs, 1565, 1520));
    TEST_ASSERT_EQUAL_UINT16(1520, requestedPulseUs.load());
}

void test_manual_pan_pulse_requires_reference_and_fresh_compass()
{
    TEST_ASSERT_TRUE(AatServo::isManualPanTestPulseAllowed(
        1520, 1520, 1480, 1565, false, false, 180, 90));
    TEST_ASSERT_FALSE(AatServo::isManualPanTestPulseAllowed(
        1565, 1520, 1480, 1565, false, true, 0, 90));
    TEST_ASSERT_FALSE(AatServo::isManualPanTestPulseAllowed(
        1565, 1520, 1480, 1565, true, false, 0, 90));
}

void test_manual_pan_pulse_is_limited_to_a_safe_test_window()
{
    TEST_ASSERT_TRUE(AatServo::isManualPanTestPulseAllowed(
        1480, 1520, 1480, 1565, true, true, -90, 90));
    TEST_ASSERT_TRUE(AatServo::isManualPanTestPulseAllowed(
        1565, 1520, 1480, 1565, true, true, 90, 90));
    TEST_ASSERT_FALSE(AatServo::isManualPanTestPulseAllowed(
        1480, 1520, 1480, 1565, true, true, -91, 90));
    TEST_ASSERT_FALSE(AatServo::isManualPanTestPulseAllowed(
        1565, 1520, 1480, 1565, true, true, 91, 90));
    TEST_ASSERT_FALSE(AatServo::isManualPanTestPulseAllowed(
        1479, 1520, 1480, 1565, true, true, 0, 90));
    TEST_ASSERT_FALSE(AatServo::isManualPanTestPulseAllowed(
        1566, 1520, 1480, 1565, true, true, 0, 90));
}

void test_pan_command_converts_compass_headings_relative_to_home()
{
    AatServo::PanCommand command = {};
    TEST_ASSERT_TRUE(AatServo::calculatePanCommand(true, true, 350, 5, 20, &command));
    TEST_ASSERT_EQUAL_INT32(15, command.currentPanDegrees);
    TEST_ASSERT_EQUAL_INT32(30, command.targetPanDegrees);
    TEST_ASSERT_EQUAL_INT32(15, command.errorDegrees);
}

void test_pan_command_clamps_target_to_soft_limits()
{
    AatServo::PanCommand command = {};
    TEST_ASSERT_TRUE(AatServo::calculatePanCommand(true, true, 0, 0, 175, &command));
    TEST_ASSERT_EQUAL_INT32(170, command.targetPanDegrees);
    TEST_ASSERT_EQUAL_INT32(170, command.errorDegrees);
}

void test_pan_command_uses_linear_error_within_limited_sweep()
{
    AatServo::PanCommand command = {};
    TEST_ASSERT_TRUE(AatServo::calculatePanCommand(true, true, 0, 169, 191, &command));
    TEST_ASSERT_EQUAL_INT32(169, command.currentPanDegrees);
    TEST_ASSERT_EQUAL_INT32(-169, command.targetPanDegrees);
    TEST_ASSERT_EQUAL_INT32(-338, command.errorDegrees);
}

void test_pan_command_rejects_unhomed_or_invalid_compass()
{
    AatServo::PanCommand command = {1, 2, 3};
    TEST_ASSERT_FALSE(AatServo::calculatePanCommand(false, true, 0, 0, 90, &command));
    TEST_ASSERT_EQUAL_INT32(0, command.currentPanDegrees);
    TEST_ASSERT_EQUAL_INT32(0, command.targetPanDegrees);
    TEST_ASSERT_EQUAL_INT32(0, command.errorDegrees);

    TEST_ASSERT_FALSE(AatServo::calculatePanCommand(true, false, 0, 0, 90, &command));
}

void test_pan_command_rejects_current_position_outside_soft_limits()
{
    AatServo::PanCommand command = {};
    TEST_ASSERT_FALSE(AatServo::calculatePanCommand(true, true, 0, 175, 0, &command));
}

void test_pan_reference_requires_confirmation_neutral_and_fresh_compass()
{
    AatServo::PanReference reference = {};
    TEST_ASSERT_FALSE(AatServo::establishPanReference(&reference, false, true, true, 350));
    TEST_ASSERT_FALSE(reference.established);
    TEST_ASSERT_FALSE(AatServo::establishPanReference(&reference, true, false, true, 350));
    TEST_ASSERT_FALSE(reference.established);
    TEST_ASSERT_FALSE(AatServo::establishPanReference(&reference, true, true, false, 350));
    TEST_ASSERT_FALSE(reference.established);
    TEST_ASSERT_TRUE(AatServo::establishPanReference(&reference, true, true, true, 350));
    TEST_ASSERT_TRUE(reference.established);
    TEST_ASSERT_EQUAL_INT32(350, reference.homeHeadingDegrees);
}

void test_pan_reference_is_required_and_commands_stay_within_soft_limits()
{
    AatServo::PanReference reference = {};
    AatServo::PanCommand command = {};
    TEST_ASSERT_FALSE(AatServo::calculatePanCommand(reference, true, 5, 20, &command));

    TEST_ASSERT_TRUE(AatServo::establishPanReference(&reference, true, true, true, 350));
    TEST_ASSERT_TRUE(AatServo::calculatePanCommand(reference, true, 5, 20, &command));
    TEST_ASSERT_EQUAL_INT32(15, command.currentPanDegrees);
    TEST_ASSERT_EQUAL_INT32(30, command.targetPanDegrees);
    TEST_ASSERT_FALSE(AatServo::calculatePanCommand(reference, true, 161, 20, &command));
}

void test_failed_pan_reference_clears_previous_reference()
{
    AatServo::PanReference reference = {};
    TEST_ASSERT_TRUE(AatServo::establishPanReference(&reference, true, true, true, 90));
    TEST_ASSERT_FALSE(AatServo::establishPanReference(&reference, true, false, true, 180));
    TEST_ASSERT_FALSE(reference.established);
    TEST_ASSERT_EQUAL_INT32(0, reference.homeHeadingDegrees);
}

void test_pan_safety_requires_button_release_after_boot()
{
    AatServo::PanSafetyController safety;
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::AwaitButtonRelease, safety.state());
    TEST_ASSERT_FALSE(safety.updateReference(true, true, true, 350));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::AwaitButtonRelease, safety.state());
    TEST_ASSERT_FALSE(safety.updateReference(false, true, true, 350));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Unreferenced, safety.state());
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 350));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Referenced, safety.state());
}

void test_continuous_pan_pulse_maps_error_and_deadband()
{
    const AatServo::PanReference reference = {0, true};
    AatServo::PanCommand command = {};
    uint16_t pulseUs = 0;

    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulse(reference, true,
        10, 20, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1565, pulseUs);
    TEST_ASSERT_EQUAL_INT32(10, command.errorDegrees);

    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulse(reference, true,
        10, 5, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1480, pulseUs);

    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulse(reference, true,
        10, 13, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, pulseUs);
}

void test_continuous_pan_uses_configured_deadband()
{
    const AatServo::PanReference reference = {0, true};
    AatServo::PanCommand command = {};
    uint16_t pulseUs = 0;

    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulse(reference, true,
        0, 5, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1565, pulseUs);
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulse(reference, true,
        0, 5, 1520, 1480, 1565, 5, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, pulseUs);
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulse(reference, true,
        0, -6, 1520, 1480, 1565, 5, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1480, pulseUs);
}

void test_continuous_pan_profile_tapers_approach_and_preserves_direction()
{
    const AatServo::PanReference reference = {0, true};
    AatServo::PanCommand command = {};
    uint16_t pulseUs = 0;

    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 90, 1520, 1510, 1480, 1530, 1565,
        3, 0, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1565, pulseUs);
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 4, 1520, 1510, 1480, 1530, 1565,
        3, 0, &pulseUs, &command));
    TEST_ASSERT_TRUE(pulseUs > 1520 && pulseUs < 1565);
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, -4, 1520, 1510, 1480, 1530, 1565,
        3, 0, &pulseUs, &command));
    TEST_ASSERT_TRUE(pulseUs < 1520 && pulseUs > 1480);
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 3, 1520, 1510, 1480, 1530, 1565,
        3, 0, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, pulseUs);
}

void test_continuous_pan_profile_applies_existing_speed_setting()
{
    const AatServo::PanReference reference = {0, true};
    AatServo::PanCommand command = {};
    uint16_t fastPulseUs = 0;
    uint16_t slowPulseUs = 0;

    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 90, 1520, 1510, 1480, 1530, 1565, 3, 0,
        &fastPulseUs, &command));
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 90, 1520, 1510, 1480, 1530, 1565, 3, 9,
        &slowPulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1565, fastPulseUs);
    TEST_ASSERT_TRUE(slowPulseUs > 1520 && slowPulseUs < fastPulseUs);
    TEST_ASSERT_EQUAL_UINT16(1000,
        AatServo::continuousPanSpeedPermille(0));
    TEST_ASSERT_EQUAL_UINT16(500,
        AatServo::continuousPanSpeedPermille(9));

    TEST_ASSERT_FALSE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 90, 1520, 1510, 1480, 1530, 1565, 3, 10,
        &slowPulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, slowPulseUs);
}

void test_continuous_pan_profile_scales_approach_once_and_is_monotonic()
{
    const AatServo::PanReference reference = {0, true};
    AatServo::PanCommand command = {};
    for (uint8_t smooth = 0; smooth <= 9; ++smooth)
    {
        uint16_t previousRight = 1520, previousLeft = 1520;
        for (int32_t error = 0; error <= 170; ++error)
        {
            uint16_t right = 0, left = 0;
            TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
                reference, true, 0, error, 1520, 1507, 1440, 1539, 1630,
                3, smooth, &right, &command));
            TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
                reference, true, 0, -error, 1520, 1507, 1440, 1539, 1630,
                3, smooth, &left, &command));
            TEST_ASSERT_TRUE(right >= previousRight && right <= 1630);
            TEST_ASSERT_TRUE(left <= previousLeft && left >= 1440);
            if (error > 3)
            {
                TEST_ASSERT_TRUE(right >= 1539);
                TEST_ASSERT_TRUE(left <= 1507);
                const uint32_t approach = error >= 48 ? 1000 : (error - 3) * 1000 / 45;
                const uint32_t scale = AatServo::continuousPanSpeedPermille(smooth) * approach / 1000;
                TEST_ASSERT_EQUAL_UINT16(1539 + 91 * scale / 1000, right);
                TEST_ASSERT_EQUAL_UINT16(1507 - 67 * scale / 1000, left);
            }
            previousRight = right;
            previousLeft = left;
        }
    }
}

void test_tracking_pan_median_requires_distinct_samples_and_resets()
{
    AatServo::TrackingPanMedian median;
    TEST_ASSERT_FALSE(median.update(0, -1));
    TEST_ASSERT_FALSE(median.update(0, 100));
    TEST_ASSERT_FALSE(median.update(50, 0));
    TEST_ASSERT_TRUE(median.update(100, 1));
    TEST_ASSERT_EQUAL_INT32(0, median.panDegrees());
    TEST_ASSERT_TRUE(median.update(100, 100));
    TEST_ASSERT_EQUAL_INT32(0, median.panDegrees());
    TEST_ASSERT_TRUE(median.update(150, 2));
    TEST_ASSERT_EQUAL_INT32(1, median.panDegrees());

    median.reset();
    TEST_ASSERT_FALSE(median.update(UINT32_MAX, 10));
    TEST_ASSERT_FALSE(median.update(0, 11));
    TEST_ASSERT_TRUE(median.update(50, 12));
    TEST_ASSERT_EQUAL_INT32(11, median.panDegrees());
}

void test_tracking_pan_median_suppresses_single_outlier_across_north()
{
    AatServo::PanSafetyController safety;
    safety.updateReference(false, true, true, 359);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 359));
    AatServo::TrackingPanMedian median;
    AatServo::PanCommand raw = {}, filtered = {};
    const int32_t headings[] = {125, 125, 125, 359, 125};
    for (unsigned i = 0; i < sizeof(headings) / sizeof(headings[0]); ++i)
    {
        TEST_ASSERT_TRUE(safety.calculateCommand(true, headings[i], 127, &raw));
        const bool ready = median.update(50 * (i + 1), raw.currentPanDegrees);
        TEST_ASSERT_EQUAL(i >= 2, ready);
        if (ready)
        {
            uint16_t pulse = 0;
            TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
                safety.reference(), true,
                safety.reference().homeHeadingDegrees + median.panDegrees(), 127,
                1520, 1480, 1220, 1565, 1820, 3, 5, &pulse, &filtered));
            TEST_ASSERT_EQUAL_INT32(126, filtered.currentPanDegrees);
            TEST_ASSERT_EQUAL_UINT16(1520,
                AatServo::constrainFilteredPanPulse(pulse, 1520, raw.errorDegrees, 3));
        }
    }

    median.reset();
    const int32_t northHeadings[] = {358, 359, 0, 1};
    for (unsigned i = 0; i < sizeof(northHeadings) / sizeof(northHeadings[0]); ++i)
        median.update(i + 1, AatServo::shortestPathDelta(359, northHeadings[i]));
    TEST_ASSERT_EQUAL_INT32(1, median.panDegrees());
}

void test_filtered_pan_pulse_respects_raw_direction_and_deadband()
{
    TEST_ASSERT_EQUAL_UINT16(1520, AatServo::constrainFilteredPanPulse(1568, 1520, 3, 3));
    TEST_ASSERT_EQUAL_UINT16(1520, AatServo::constrainFilteredPanPulse(1568, 1520, -10, 3));
    TEST_ASSERT_EQUAL_UINT16(1568, AatServo::constrainFilteredPanPulse(1568, 1520, 4, 3));
    TEST_ASSERT_EQUAL_UINT16(1520, AatServo::constrainFilteredPanPulse(1477, 1520, -3, 3));
    TEST_ASSERT_EQUAL_UINT16(1520, AatServo::constrainFilteredPanPulse(1477, 1520, 10, 3));
    TEST_ASSERT_EQUAL_UINT16(1477, AatServo::constrainFilteredPanPulse(1477, 1520, -4, 3));
}

void test_tracking_pan_filter_does_not_bypass_raw_travel_guard()
{
    AatServo::PanSafetyController safety;
    safety.updateReference(false, true, true, 0);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 0));
    AatServo::TrackingPanMedian median;
    AatServo::PanCommand raw = {};
    for (unsigned i = 1; i <= 3; ++i)
    {
        TEST_ASSERT_TRUE(safety.calculateCommand(true, 165, 175, &raw));
        median.update(50 * i, raw.currentPanDegrees);
    }
    TEST_ASSERT_EQUAL_INT32(165, median.panDegrees());
    TEST_ASSERT_FALSE(safety.calculateCommand(true, 171, 175, &raw));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
    TEST_ASSERT_FALSE(safety.reference().established);
}

struct PulseCalibrationSimulation
{
    AatServo::PulseCalibration calibration;
    uint32_t now = 0, sequence = 0;
    uint16_t appliedUs = 1520;
    double panDegrees = 0;

    void start(uint16_t leftOffset = 40, uint16_t rightOffset = 45)
    {
        TEST_ASSERT_TRUE(calibration.start(now, 1520, leftOffset, rightOffset, sequence));
    }

    uint16_t step(int32_t rate = 0, bool valid = true, bool safe = true,
        int32_t pan = INT32_MIN, uint32_t deltaMs = 500)
    {
        for (uint32_t elapsed = 0; elapsed < deltaMs; elapsed += 100)
        {
            now += 100;
            ++sequence;
            if (pan == INT32_MIN)
                panDegrees += rate / 10000.0;
            else
                panDegrees = pan;
            appliedUs = calibration.update(now, safe,
                static_cast<int32_t>(lround(panDegrees)), appliedUs, valid, sequence);
        }
        return appliedUs;
    }

    void settle()
    {
        for (unsigned i = 0; i < 6 && calibration.settling(); ++i)
            step();
        TEST_ASSERT_FALSE(calibration.settling());
        TEST_ASSERT_TRUE(calibration.active());
    }
};

void test_pulse_calibration_requires_fresh_heading_windows_and_neutral_reversal()
{
    PulseCalibrationSimulation sim;
    sim.start();
    sim.settle();
    TEST_ASSERT_EQUAL_UINT16(1515, sim.appliedUs);
    sim.step();
    TEST_ASSERT_EQUAL_UINT8(0, sim.calibration.samples());
    sim.step(-4000);
    TEST_ASSERT_EQUAL_UINT8(1, sim.calibration.samples());
    sim.calibration.update(sim.now + 1, true,
        static_cast<int32_t>(lround(sim.panDegrees)), sim.appliedUs, true, sim.sequence);
    TEST_ASSERT_EQUAL_UINT8(1, sim.calibration.samples());
    sim.step();
    sim.step();
    TEST_ASSERT_EQUAL_UINT8(0, sim.calibration.samples());
    TEST_ASSERT_EQUAL_UINT16(0, sim.calibration.leftStartUs());

    PulseCalibrationSimulation reversal;
    reversal.start();
    reversal.settle();
    for (unsigned i = 0; i < 6
        && reversal.calibration.state() == AatServo::PulseCalibration::State::Left; ++i)
        reversal.step(-4000);
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::State::Right, reversal.calibration.state());
    TEST_ASSERT_EQUAL_UINT16(1520, reversal.appliedUs);
    TEST_ASSERT_TRUE(reversal.calibration.settling());
    // Continued leftward inertia cannot start a rightward pulse.
    TEST_ASSERT_EQUAL_UINT16(1520, reversal.step(-4000));
    for (unsigned i = 0; i < 5 && reversal.calibration.settling(); ++i)
        reversal.step();
    TEST_ASSERT_TRUE(reversal.calibration.active());
    TEST_ASSERT_EQUAL_UINT16(1525, reversal.appliedUs);
}

void test_pulse_calibration_clamps_non_multiple_step_endpoints()
{
    for (unsigned direction = 0; direction < 2; ++direction)
    {
        PulseCalibrationSimulation sim;
        sim.start(7, 12);
        bool sawEndpoint = false;
        for (unsigned i = 0; i < 150 && sim.calibration.active(); ++i)
        {
            const int32_t rate = direction == 1 && sim.appliedUs < 1520 ? -4000 : 0;
            const uint16_t pulse = sim.step(rate);
            TEST_ASSERT_TRUE(pulse >= 1513 && pulse <= 1532);
            sawEndpoint |= pulse == (direction == 0 ? 1513 : 1532);
        }
        TEST_ASSERT_TRUE(sawEndpoint);
        TEST_ASSERT_EQUAL(AatServo::PulseCalibration::State::Fault, sim.calibration.state());
        TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::StageTimeout, sim.calibration.fault());
        TEST_ASSERT_EQUAL_UINT16(1520, sim.appliedUs);
    }
}

void test_pulse_calibration_aborts_on_feedback_direction_travel_and_cancel()
{
    for (unsigned fault = 0; fault < 6; ++fault)
    {
        PulseCalibrationSimulation sim;
        sim.start();
        sim.settle();
        sim.step(-4000);
        if (fault == 0)
            sim.step(-4000, true, false);
        else if (fault == 1)
        {
            for (unsigned i = 0; i < 6 && sim.calibration.active(); ++i)
                sim.step(4000);
            TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::WrongDirection, sim.calibration.fault());
        }
        else if (fault == 2)
            sim.step(-4000, true, true, -15);
        else if (fault == 3)
            sim.step(-4000, true, true, 90);
        else if (fault == 4)
            sim.step(-4000, true, true, -90);
        else
            sim.calibration.cancel();
        TEST_ASSERT_FALSE(sim.calibration.active());
        TEST_ASSERT_EQUAL_UINT16(1520, sim.calibration.requestedPulseUs());
        TEST_ASSERT_EQUAL_UINT16(1520,
            sim.calibration.update(sim.now + 100, true, 0, sim.appliedUs, true, ++sim.sequence));
    }
}

void test_pulse_calibration_handles_missing_headings_settle_timeout_and_clock_wrap()
{
    PulseCalibrationSimulation sim;
    sim.now = UINT32_MAX - 900;
    sim.start(5, 5);
    sim.settle();
    TEST_ASSERT_TRUE(sim.now < 2000);
    for (unsigned i = 0; i < 10 && sim.calibration.active(); ++i)
        sim.step(0, false);
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::StageTimeout, sim.calibration.fault());
    TEST_ASSERT_EQUAL_UINT16(1520, sim.appliedUs);

    sim.start();
    for (unsigned i = 0; i < 10 && sim.calibration.active(); ++i)
        sim.step(4000);
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::SettleTimeout, sim.calibration.fault());

    sim.start();
    sim.now += AatServo::PulseCalibration::TOTAL_TIMEOUT_MS;
    sim.appliedUs = sim.calibration.update(sim.now, true, 0, sim.appliedUs, true, ++sim.sequence);
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::Timeout, sim.calibration.fault());
    TEST_ASSERT_EQUAL_UINT16(1520, sim.appliedUs);
}

void test_pulse_calibration_rejects_output_loss_and_duplicate_start()
{
    PulseCalibrationSimulation sim;
    sim.start();
    TEST_ASSERT_FALSE(sim.calibration.start(1, 1520, 480, 480, 0));
    sim.settle();
    sim.step(-4000);
    sim.appliedUs = 1520;
    sim.step();
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::Output, sim.calibration.fault());
    TEST_ASSERT_EQUAL_UINT16(1520, sim.appliedUs);

    sim.start();
    sim.now += 3000;
    sim.calibration.update(sim.now, true, 0, sim.appliedUs, true, ++sim.sequence);
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::Output, sim.calibration.fault());
}

void test_pulse_calibration_simulates_dead_zone_inertia_and_complete_restart()
{
    PulseCalibrationSimulation sim;
    for (unsigned run = 0; run < 2; ++run)
    {
        sim.start();
        double pan = 0, velocity = 0;
        for (unsigned tick = 0; tick < 1200 && sim.calibration.active(); ++tick)
        {
            const double desired = sim.appliedUs <= 1510
                ? -0.8 - (1510 - sim.appliedUs) * 0.4
                : sim.appliedUs >= 1535 ? 0.8 + (sim.appliedUs - 1535) * 0.4 : 0;
            velocity += (desired - velocity) / 4.0;
            pan += velocity * 0.1;
            sim.now += 100;
            ++sim.sequence;
            const uint16_t pulse = sim.calibration.update(sim.now, true,
                static_cast<int32_t>(lround(pan)), sim.appliedUs,
                true, sim.sequence);
            TEST_ASSERT_TRUE(pulse >= 1480 && pulse <= 1565);
            sim.appliedUs = pulse;
        }
        TEST_ASSERT_EQUAL(AatServo::PulseCalibration::State::Complete, sim.calibration.state());
        TEST_ASSERT_EQUAL_UINT16(1505, sim.calibration.leftStartUs());
        TEST_ASSERT_EQUAL_UINT16(1540, sim.calibration.rightStartUs());
        TEST_ASSERT_EQUAL_UINT16(1520, sim.appliedUs);
        TEST_ASSERT_TRUE(fabs(velocity) < 0.5);
        TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseCalibrationValid(
            1520, sim.calibration.leftStartUs(), sim.calibration.rightStartUs(), 40, 45));
    }
}

void test_pulse_calibration_tolerates_measured_stationary_jitter_without_inventing_motion()
{
    const int32_t observed[] = {357, 356, 355, 356, 355, 355, 355, 357, 355, 356,
        357, 353, 358, 356, 357, 356, 356, 355, 354, 355};
    AatServo::PulseCalibration calibration;
    TEST_ASSERT_TRUE(calibration.start(0, 1520, 7, 12, 0));
    uint16_t pulse = 1520;
    bool sawPoweredStage = false;
    for (uint32_t now = 50; now <= 20000 && calibration.active(); now += 50)
    {
        const uint32_t sequence = now / 50;
        const int32_t pan = observed[(sequence - 1) % 20] - 355;
        pulse = calibration.update(now, true, pan, pulse, true, sequence);
        sawPoweredStage |= pulse != 1520;
        TEST_ASSERT_TRUE(pulse >= 1513 && pulse <= 1532);
        TEST_ASSERT_EQUAL_UINT16(0, calibration.leftStartUs());
    }
    TEST_ASSERT_TRUE(sawPoweredStage);
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::StageTimeout, calibration.fault());
    TEST_ASSERT_EQUAL_UINT16(1520, pulse);
}

void test_pulse_calibration_rejects_duplicate_sparse_and_invalid_heading_evidence()
{
    for (unsigned fault = 0; fault < 3; ++fault)
    {
        PulseCalibrationSimulation sim;
        sim.start(5, 5);
        sim.settle();
        const uint32_t sequence = sim.sequence;
        for (unsigned tick = 0; tick < 30 && sim.calibration.active(); ++tick)
        {
            sim.now += 100;
            const uint32_t sampleSequence = fault == 0 ? sequence
                : fault == 1 ? sequence + tick / 5 : sequence + tick;
            sim.appliedUs = sim.calibration.update(sim.now, true,
                -static_cast<int32_t>(tick / 3), sim.appliedUs, fault != 2, sampleSequence);
        }
        TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::StageTimeout, sim.calibration.fault());
        TEST_ASSERT_EQUAL_UINT16(0, sim.calibration.leftStartUs());
        TEST_ASSERT_EQUAL_UINT16(1520, sim.appliedUs);
    }
}

void test_pulse_calibration_simulates_inertia_with_bounded_jitter_in_both_directions()
{
    for (uint32_t seed = 1; seed <= 12; ++seed)
    {
        AatServo::PulseCalibration calibration;
        TEST_ASSERT_TRUE(calibration.start(0, 1520, 40, 45, 0));
        uint16_t pulse = 1520;
        uint32_t random = seed;
        double pan = 0, velocity = 0;
        bool reversedAtNeutral = false;
        for (uint32_t now = 50; now <= 120000 && calibration.active(); now += 50)
        {
            const double desired = pulse <= 1510 ? -0.8 - (1510 - pulse) * 0.4
                : pulse >= 1535 ? 0.8 + (pulse - 1535) * 0.4 : 0;
            velocity += (desired - velocity) / 7.0;
            pan += velocity * 0.05;
            random = random * 1664525U + 1013904223U;
            const int32_t noise = static_cast<int32_t>((random >> 16) % 7) - 3;
            const int32_t measuredPan = static_cast<int32_t>(lround(pan)) + noise;
            pulse = calibration.update(now, true, measuredPan, pulse, true, now / 50);
            reversedAtNeutral |= calibration.state() == AatServo::PulseCalibration::State::Right
                && calibration.settling() && pulse == 1520;
            TEST_ASSERT_TRUE(pulse >= 1480 && pulse <= 1565);
        }
        char context[160];
        snprintf(context, sizeof(context),
            "seed=%lu state=%u fault=%u candidate=%u left=%u right=%u pan=%.2f displacement=%ld samples=%u",
            static_cast<unsigned long>(seed), static_cast<unsigned>(calibration.state()),
            static_cast<unsigned>(calibration.fault()), calibration.candidateUs(),
            calibration.leftStartUs(), calibration.rightStartUs(), pan,
            static_cast<long>(calibration.displacementMilliDegrees()), calibration.samples());
        TEST_ASSERT_EQUAL_MESSAGE(AatServo::PulseCalibration::State::Complete, calibration.state(), context);
        TEST_ASSERT_TRUE(reversedAtNeutral);
        TEST_ASSERT_TRUE(calibration.leftStartUs() <= 1505 && calibration.leftStartUs() >= 1480);
        TEST_ASSERT_TRUE(calibration.rightStartUs() >= 1540 && calibration.rightStartUs() <= 1565);
        TEST_ASSERT_EQUAL_UINT16(1520, pulse);
    }
}

void test_pulse_calibration_keeps_raw_travel_guards_ahead_of_heading_filter()
{
    const int32_t jumps[] = {-15, 15, -90, 90};
    for (const int32_t jump : jumps)
    {
        PulseCalibrationSimulation sim;
        sim.start();
        sim.settle();
        // Even the first applied sample must stop motion before filling a filter window.
        const uint16_t pulse = sim.calibration.update(
            sim.now + 50, true, jump, sim.appliedUs, true, ++sim.sequence);
        TEST_ASSERT_EQUAL_UINT16(1520, pulse);
        TEST_ASSERT_EQUAL(AatServo::PulseCalibration::State::Fault, sim.calibration.state());
        TEST_ASSERT_EQUAL(jump == -90 || jump == 90
            ? AatServo::PulseCalibration::Fault::SafetyGate
            : AatServo::PulseCalibration::Fault::Travel, sim.calibration.fault());
    }
}

void test_pulse_calibration_stops_sustained_opposite_motion_despite_jitter()
{
    PulseCalibrationSimulation sim;
    sim.start();
    sim.settle();
    double pan = 0;
    for (uint32_t tick = 1; tick <= 60 && sim.calibration.active(); ++tick)
    {
        sim.now += 50;
        pan += 0.2; // Positive motion opposes the requested leftward stage.
        const int32_t noise = static_cast<int32_t>(tick % 7) - 3;
        sim.appliedUs = sim.calibration.update(sim.now, true,
            static_cast<int32_t>(lround(pan)) + noise, sim.appliedUs, true, ++sim.sequence);
    }
    TEST_ASSERT_EQUAL(AatServo::PulseCalibration::Fault::WrongDirection, sim.calibration.fault());
    TEST_ASSERT_EQUAL_UINT16(0, sim.calibration.leftStartUs());
    TEST_ASSERT_EQUAL_UINT16(1520, sim.appliedUs);
}

void test_simulated_drone_positions_drive_loaded_pan_and_settle_inside_deadband()
{
    // Coordinates are aircraft GPS fixes (1e-7 degrees), not tracker-local GPS.
    const int32_t fixes[][2] = {
        {0, 100000}, {100000, 100000}, {100000, 0}, {100000, -100000},
        {0, -100000}, {-100000, 0}, {-100000, -100000}, {100000, 0}
    };
    AatServo::PanSafetyController safety;
    safety.updateReference(false, true, true, 0);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 0));
    double pan = 0, velocity = 0;
    for (const auto &fix : fixes)
    {
        uint32_t bearing = 0;
        AatNavigation::calculateDistanceAndAzimuth(0, 0, fix[0], fix[1], nullptr, &bearing);
        AatServo::PanCommand command = {};
        uint16_t pulse = 1520;
        for (unsigned tick = 0; tick < 600; ++tick)
        {
            const int32_t heading = static_cast<int32_t>(lround(pan));
            TEST_ASSERT_TRUE(safety.calculateCommand(true, heading, bearing, &command));
            TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
                safety.reference(), true, heading, bearing,
                1520, 1510, 1440, 1535, 1620, 3, 0, &pulse, &command));
            TEST_ASSERT_TRUE(pulse >= 1440 && pulse <= 1620);
            const double desired = pulse < 1520 ? -4.0 - (1510 - pulse) * 0.5
                : pulse > 1520 ? 4.0 + (pulse - 1535) * 0.4 : 0;
            velocity += (desired - velocity) / 4.0;
            pan += velocity * 0.1;
            char context[128];
            snprintf(context, sizeof(context), "bearing=%lu tick=%u pan=%.3f rate=%.3f pulse=%u",
                static_cast<unsigned long>(bearing), tick, pan, velocity, pulse);
            TEST_ASSERT_TRUE_MESSAGE(pan > -170.5 && pan < 170.5, context);
        }
        TEST_ASSERT_INT32_WITHIN(3, command.targetPanDegrees, static_cast<int32_t>(lround(pan)));
        TEST_ASSERT_EQUAL_UINT16(1520, pulse);
        TEST_ASSERT_TRUE(fabs(velocity) < 0.01);
    }
}

void test_continuous_pan_gps_freshness_caps_long_gaps_and_keeps_fast_timeout()
{
    TEST_ASSERT_FALSE(AatServo::isContinuousPanGpsFresh(false, 200, 0));
    TEST_ASSERT_FALSE(AatServo::isContinuousPanGpsFresh(true, 0, 0));
    TEST_ASSERT_TRUE(AatServo::isContinuousPanGpsFresh(true, 200, 600));
    TEST_ASSERT_FALSE(AatServo::isContinuousPanGpsFresh(true, 200, 601));
    TEST_ASSERT_TRUE(AatServo::isContinuousPanGpsFresh(true, 10000, 2999));
    TEST_ASSERT_FALSE(AatServo::isContinuousPanGpsFresh(true, 10000, 3000));
    TEST_ASSERT_FALSE(AatServo::isContinuousPanGpsFresh(true, 10000, 13142));
}

void test_simulated_aircraft_gps_loss_neutralizes_and_compass_loss_requires_reference()
{
    AatServo::PanSafetyController safety;
    safety.updateReference(false, true, true, 0);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 0));
    for (unsigned frame = 0; frame < 60; ++frame)
    {
        uint32_t bearing = 0;
        AatNavigation::calculateDistanceAndAzimuth(
            0, 0, 100000, 100000 + frame * 1000, nullptr, &bearing);
        const uint32_t gpsAgeMs = frame < 10 || frame >= 20 ? 0 : 3000 + (frame - 10) * 100;
        const bool gpsFresh = AatServo::isContinuousPanGpsFresh(true, 10000, gpsAgeMs);
        const bool compassFresh = frame < 30 || frame >= 40;
        AatServo::PanCommand command = {};
        const bool referenced = safety.updateReference(false, true, compassFresh, 0);
        const auto reason = AatServo::getContinuousPanBlockReason(
            false, true, compassFresh, gpsFresh, safety.state());
        uint16_t pulse = 1520;
        if (reason == AatServo::ContinuousPanBlockReason::None
            && AatServo::areContinuousPanCommandInputsValid(gpsFresh, compassFresh, referenced))
        {
            TEST_ASSERT_TRUE(safety.calculateCommand(compassFresh, 0, bearing, &command));
            TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
                safety.reference(), compassFresh, 0, bearing,
                1520, 1510, 1480, 1535, 1565, 3, 5, &pulse, &command));
        }
        if ((frame >= 10 && frame < 20) || frame >= 30)
            TEST_ASSERT_EQUAL_UINT16(1520, pulse);
        else
            TEST_ASSERT_TRUE(pulse > 1520);
    }
    TEST_ASSERT_FALSE(safety.reference().established);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 0));
    AatServo::PanCommand command = {};
    TEST_ASSERT_TRUE(safety.calculateCommand(true, 0, 45, &command));
}

void test_continuous_pan_pulse_handles_wrap_and_rejects_unsafe_state()
{
    const AatServo::PanReference reference = {350, true};
    AatServo::PanCommand command = {};
    uint16_t pulseUs = 0;

    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulse(reference, true,
        355, 5, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1565, pulseUs);
    TEST_ASSERT_EQUAL_INT32(10, command.errorDegrees);

    TEST_ASSERT_FALSE(AatServo::calculateContinuousPanPulse(reference, false,
        355, 5, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, pulseUs);

    TEST_ASSERT_FALSE(AatServo::calculateContinuousPanPulse(reference, true,
        161, 5, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, pulseUs);

    TEST_ASSERT_FALSE(AatServo::calculateContinuousPanPulse(reference, true,
        355, 5, 1520, 1520, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, pulseUs);
}

void test_compass_startup_alignment_requires_operator_without_overwriting_north()
{
    typedef AatServo::CompassStartupAlignmentAction Action;
    TEST_ASSERT_EQUAL(Action::WaitForCompass,
        AatServo::getCompassStartupAlignmentAction(false, false, false, false));
    TEST_ASSERT_EQUAL(Action::WaitForOperator,
        AatServo::getCompassStartupAlignmentAction(false, false, true, false));
    TEST_ASSERT_EQUAL(Action::CaptureNorthAndHome,
        AatServo::getCompassStartupAlignmentAction(false, false, true, true));
    TEST_ASSERT_EQUAL(Action::WaitForOperator,
        AatServo::getCompassStartupAlignmentAction(false, true, true, false));
    TEST_ASSERT_EQUAL(Action::CaptureHome,
        AatServo::getCompassStartupAlignmentAction(false, true, true, true));
    TEST_ASSERT_EQUAL(Action::Complete,
        AatServo::getCompassStartupAlignmentAction(true, true, true, false));
}

void test_compass_pan_setup_state_inhibits_tracking_until_fit_and_alignment()
{
    typedef AatServo::CompassPanOperatingState State;
    TEST_ASSERT_TRUE(AatServo::canResetCompassCalibration(true, false, false, 1520, 1520, 1520));
    TEST_ASSERT_FALSE(AatServo::canResetCompassCalibration(false, false, false, 1520, 1520, 1520));
    TEST_ASSERT_FALSE(AatServo::canResetCompassCalibration(true, true, false, 1520, 1520, 1520));
    TEST_ASSERT_FALSE(AatServo::canResetCompassCalibration(true, false, true, 1520, 1520, 1520));
    TEST_ASSERT_FALSE(AatServo::canResetCompassCalibration(true, false, false, 1480, 1520, 1520));
    TEST_ASSERT_FALSE(AatServo::canResetCompassCalibration(true, false, false, 1520, 1480, 1520));
    TEST_ASSERT_FALSE(AatServo::shouldEnterCompassPanSetup(false, false, true, true));
    TEST_ASSERT_TRUE(AatServo::shouldEnterCompassPanSetup(true, false, true, false));
    TEST_ASSERT_TRUE(AatServo::shouldEnterCompassPanSetup(true, true, false, false));
    TEST_ASSERT_TRUE(AatServo::shouldEnterCompassPanSetup(true, true, true, true));
    TEST_ASSERT_FALSE(AatServo::shouldEnterCompassPanSetup(true, true, true, false));
    TEST_ASSERT_EQUAL(State::Disabled,
        AatServo::getCompassPanOperatingState(false, false, false, false, false));
    TEST_ASSERT_EQUAL(State::Setup,
        AatServo::getCompassPanOperatingState(true, false, false, false, false));
    TEST_ASSERT_EQUAL(State::Setup,
        AatServo::getCompassPanOperatingState(true, true, true, true, true));
    TEST_ASSERT_EQUAL(State::StartupAlignment,
        AatServo::getCompassPanOperatingState(true, false, true, false, false));
    TEST_ASSERT_EQUAL(State::StartupAlignment,
        AatServo::getCompassPanOperatingState(true, false, true, true, false));
    TEST_ASSERT_EQUAL(State::Tracking,
        AatServo::getCompassPanOperatingState(true, false, true, true, true));
}

void test_gpio32_press_edge_requires_stable_new_press()
{
    TEST_ASSERT_FALSE(AatServo::hasDebouncedPressEdge(false, false, 50, 30));
    TEST_ASSERT_FALSE(AatServo::hasDebouncedPressEdge(false, true, 29, 30));
    TEST_ASSERT_TRUE(AatServo::hasDebouncedPressEdge(false, true, 30, 30));
    TEST_ASSERT_FALSE(AatServo::hasDebouncedPressEdge(true, true, 50, 30));
    TEST_ASSERT_FALSE(AatServo::hasDebouncedPressEdge(true, false, 50, 30));
}

void test_sweep_timeout_only_applies_after_rotation_starts()
{
    TEST_ASSERT_FALSE(AatServo::isSweepTimedOut(false, 40000, 40000));
    TEST_ASSERT_FALSE(AatServo::isSweepTimedOut(true, 39999, 40000));
    TEST_ASSERT_TRUE(AatServo::isSweepTimedOut(true, 40000, 40000));
}

void test_sweep_starts_on_hold_and_finishes_on_release()
{
    TEST_ASSERT_FALSE(AatServo::shouldStartSweepOnReferencePress(false, true));
    TEST_ASSERT_TRUE(AatServo::shouldStartSweepOnReferencePress(true, true));
    TEST_ASSERT_FALSE(AatServo::shouldStartSweepOnReferencePress(true, false));
    TEST_ASSERT_FALSE(AatServo::shouldCompleteSweepOnReferenceRelease(false, false));
    TEST_ASSERT_TRUE(AatServo::shouldCompleteSweepOnReferenceRelease(true, false));
    TEST_ASSERT_FALSE(AatServo::shouldCompleteSweepOnReferenceRelease(true, true));
}

void test_setup_sweep_wait_state_stays_neutral()
{
    TEST_ASSERT_EQUAL_UINT16(1520,
        AatServo::getCompassSetupSweepPulse(false, 1450, 1520));
    TEST_ASSERT_EQUAL_UINT16(1450,
        AatServo::getCompassSetupSweepPulse(true, 1450, 1520));
}

void test_compass_configuration_must_match_before_reference()
{
    TEST_ASSERT_TRUE(AatServo::isCompassConfigurationValid(true, 0x68, 0x68));
    TEST_ASSERT_FALSE(AatServo::isCompassConfigurationValid(true, 0x10, 0x68));
    TEST_ASSERT_FALSE(AatServo::isCompassConfigurationValid(false, 0x68, 0x68));
}

void test_compass_sample_release_requires_clear_post_trigger_lock()
{
    TEST_ASSERT_TRUE(AatCompass::isCompassSampleReleaseConfirmed(true, 0x01));
    TEST_ASSERT_FALSE(AatCompass::isCompassSampleReleaseConfirmed(true, 0x03));
    TEST_ASSERT_FALSE(AatCompass::isCompassSampleReleaseConfirmed(false, 0x01));
}

void test_compass_pan_sample_validity_requires_every_safety_gate()
{
    TEST_ASSERT_TRUE(AatServo::areCompassPanSampleInputsValid(
        true, true, true, true, true, true, true));
    TEST_ASSERT_FALSE(AatServo::areCompassPanSampleInputsValid(
        false, true, true, true, true, true, true));
    TEST_ASSERT_FALSE(AatServo::areCompassPanSampleInputsValid(
        true, false, true, true, true, true, true));
    TEST_ASSERT_FALSE(AatServo::areCompassPanSampleInputsValid(
        true, true, false, true, true, true, true));
    TEST_ASSERT_FALSE(AatServo::areCompassPanSampleInputsValid(
        true, true, true, false, true, true, true));
    TEST_ASSERT_FALSE(AatServo::areCompassPanSampleInputsValid(
        true, true, true, true, false, true, true));
    TEST_ASSERT_FALSE(AatServo::areCompassPanSampleInputsValid(
        true, true, true, true, true, false, true));
    TEST_ASSERT_FALSE(AatServo::areCompassPanSampleInputsValid(
        true, true, true, true, true, true, false));
}

void test_continuous_pan_command_gate_requires_gps_compass_and_reference()
{
    TEST_ASSERT_TRUE(AatServo::areContinuousPanCommandInputsValid(true, true, true));
    TEST_ASSERT_FALSE(AatServo::areContinuousPanCommandInputsValid(false, true, true));
    TEST_ASSERT_FALSE(AatServo::areContinuousPanCommandInputsValid(true, false, true));
    TEST_ASSERT_FALSE(AatServo::areContinuousPanCommandInputsValid(true, true, false));
    TEST_ASSERT_FALSE(AatServo::areContinuousPanCommandInputsValid(false, false, false));
}

void test_continuous_pan_block_reason_covers_each_fault_and_recovery()
{
    using Reason = AatServo::ContinuousPanBlockReason;
    using State = AatServo::PanSafetyState;

    TEST_ASSERT_EQUAL(Reason::Setup,
        AatServo::getContinuousPanBlockReason(true, false, false, false, State::AwaitButtonRelease));
    TEST_ASSERT_EQUAL(Reason::StartupAlignment,
        AatServo::getContinuousPanBlockReason(false, false, false, false, State::AwaitButtonRelease));
    TEST_ASSERT_EQUAL(Reason::CompassInvalid,
        AatServo::getContinuousPanBlockReason(false, true, false, false, State::Referenced));
    TEST_ASSERT_EQUAL(Reason::SafetyFault,
        AatServo::getContinuousPanBlockReason(false, true, false, false, State::Fault));
    TEST_ASSERT_EQUAL(Reason::ReferenceUnavailable,
        AatServo::getContinuousPanBlockReason(false, true, true, false, State::Unreferenced));
    TEST_ASSERT_EQUAL(Reason::GpsUnavailable,
        AatServo::getContinuousPanBlockReason(false, true, true, false, State::Referenced));
    TEST_ASSERT_EQUAL(Reason::None,
        AatServo::getContinuousPanBlockReason(false, true, true, true, State::Referenced));
}

void test_compass_north_offset_uses_shortest_wrap_safe_delta()
{
    int16_t offset = 0;
    TEST_ASSERT_TRUE(AatCompass::calculateCompassHeadingOffsetDegrees(359500, 2, &offset));
    TEST_ASSERT_EQUAL_INT16(2, offset);
    TEST_ASSERT_TRUE(AatCompass::calculateCompassHeadingOffsetDegrees(10000, 359, &offset));
    TEST_ASSERT_EQUAL_INT16(-11, offset);
    TEST_ASSERT_TRUE(AatCompass::calculateCompassHeadingOffsetDegrees(185000, 0, &offset));
    TEST_ASSERT_EQUAL_INT16(175, offset);
    TEST_ASSERT_FALSE(AatCompass::calculateCompassHeadingOffsetDegrees(0, 360, &offset));
    TEST_ASSERT_FALSE(AatCompass::calculateCompassHeadingOffsetDegrees(0, 0, nullptr));
}

void test_pan_safety_requires_neutral_actuator_and_fresh_compass()
{
    AatServo::PanSafetyController safety;
    safety.updateReference(false, true, true, 0);
    TEST_ASSERT_FALSE(safety.updateReference(true, false, true, 0));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
    TEST_ASSERT_FALSE(safety.updateReference(false, true, true, 0));
    TEST_ASSERT_FALSE(safety.updateReference(true, true, false, 0));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
    TEST_ASSERT_FALSE(safety.updateReference(false, true, true, 0));
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 0));
}

void test_pan_safety_latches_compass_and_travel_faults()
{
    AatServo::PanSafetyController safety;
    AatServo::PanCommand command = {};
    safety.updateReference(false, true, true, 350);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 350));
    TEST_ASSERT_FALSE(safety.calculateCommand(true, 5, 20, nullptr));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Referenced, safety.state());
    TEST_ASSERT_TRUE(safety.calculateCommand(true, 5, 20, &command));
    TEST_ASSERT_FALSE(safety.calculateCommand(false, 5, 20, &command));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
    TEST_ASSERT_EQUAL_INT32(0, command.errorDegrees);

    safety.updateReference(false, true, true, 350);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 350));
    TEST_ASSERT_FALSE(safety.calculateCommand(true, 161, 20, &command));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
    TEST_ASSERT_EQUAL_INT32(0, command.errorDegrees);
}

void test_pan_safety_invalidation_requires_release_and_new_reference()
{
    AatServo::PanSafetyController safety;
    safety.updateReference(false, true, true, 10);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 10));

    safety.invalidate();
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
    TEST_ASSERT_FALSE(safety.updateReference(true, true, true, 20));
    TEST_ASSERT_FALSE(safety.updateReference(false, true, true, 20));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Unreferenced, safety.state());
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 20));
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Referenced, safety.state());
}

void test_pan_safety_rejects_skipped_rear_boundary_in_both_directions()
{
    for (int32_t direction = -1; direction <= 1; direction += 2)
    {
        AatServo::PanSafetyController safety;
        AatServo::PanCommand command = {};
        safety.updateReference(false, true, true, 0);
        TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 0));
        TEST_ASSERT_TRUE(safety.calculateCommand(true, direction * 160, 0, &command));
        TEST_ASSERT_FALSE(safety.calculateCommand(true, direction * 200, 0, &command));
        TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
        TEST_ASSERT_EQUAL_INT32(0, command.errorDegrees);
    }
}

void test_pan_safety_checks_rear_crossing_during_reference_updates()
{
    for (int32_t direction = -1; direction <= 1; direction += 2)
    {
        AatServo::PanSafetyController safety;
        safety.updateReference(false, true, true, 350);
        TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 350));
        TEST_ASSERT_TRUE(safety.updateReference(false, false, true,
            350 + direction * 160));
        TEST_ASSERT_FALSE(safety.updateReference(false, false, true,
            350 + direction * 200));
        TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Fault, safety.state());
        TEST_ASSERT_FALSE(safety.reference().established);
    }
}

void test_pan_feedback_freshness_uses_output_time_after_stall()
{
    AatServo::PanSafetyController safety;
    safety.updateReference(false, true, true, 0);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 0));
    TEST_ASSERT_TRUE(AatServo::isPanFeedbackFresh(1000U, 1500U, 500U));
    TEST_ASSERT_FALSE(AatServo::isPanFeedbackFresh(1000U, 1501U, 500U));
    TEST_ASSERT_FALSE(safety.updateReference(false, false,
        AatServo::isPanFeedbackFresh(1000U, 2000U, 500U), 0));
    uint16_t pulseUs = 1565U;
    AatServo::PanCommand command = {};
    TEST_ASSERT_FALSE(AatServo::calculateContinuousPanPulse(safety.reference(),
        false, 0, 90, 1520, 1480, 1565, 3, &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1520, pulseUs);
    TEST_ASSERT_FALSE(safety.reference().established);
    TEST_ASSERT_TRUE(AatServo::isPanFeedbackFresh(UINT32_MAX - 100U, 399U, 500U));
    TEST_ASSERT_FALSE(AatServo::isPanFeedbackFresh(UINT32_MAX - 100U, 400U, 500U));
}

void test_pan_safety_allows_direct_travel_and_north_wrap()
{
    AatServo::PanSafetyController safety;
    AatServo::PanCommand command = {};
    safety.updateReference(false, true, true, 350);
    TEST_ASSERT_TRUE(safety.updateReference(true, true, true, 350));
    for (int32_t panDegrees = -160; panDegrees <= 160; panDegrees += 80)
    {
        TEST_ASSERT_TRUE(safety.calculateCommand(true, 350 + panDegrees, 0, &command));
        TEST_ASSERT_EQUAL_INT32(panDegrees, command.currentPanDegrees);
    }
    TEST_ASSERT_EQUAL(AatServo::PanSafetyState::Referenced, safety.state());
}

void test_compass_heading_uses_cardinal_directions()
{
    const AatCompass::Calibration calibration = {0, 0, 1, 1, 0, true};
    int32_t heading = -1;

    TEST_ASSERT_TRUE(AatCompass::calculateHeading(100, 0, calibration, &heading));
    TEST_ASSERT_EQUAL_INT32(0, heading);
    TEST_ASSERT_TRUE(AatCompass::calculateHeading(0, 100, calibration, &heading));
    TEST_ASSERT_EQUAL_INT32(270, heading);
    TEST_ASSERT_TRUE(AatCompass::calculateHeading(-100, 0, calibration, &heading));
    TEST_ASSERT_EQUAL_INT32(180, heading);
    TEST_ASSERT_TRUE(AatCompass::calculateHeading(0, -100, calibration, &heading));
    TEST_ASSERT_EQUAL_INT32(90, heading);
}

void test_frd_compass_clockwise_quarter_turn_increases_heading()
{
    const AatCompass::Calibration calibration = {345.5f, -264.5f, 1, 1, 0, true};
    int32_t startHeading = 0;
    int32_t turnedHeading = 0;

    TEST_ASSERT_TRUE(AatCompass::calculateHeading(487, -180, calibration, &startHeading));
    TEST_ASSERT_TRUE(AatCompass::calculateHeading(416, -397, calibration, &turnedHeading));

    int32_t headingChange = turnedHeading - startHeading;
    if (headingChange > 180)
        headingChange -= 360;
    else if (headingChange < -180)
        headingChange += 360;

    TEST_ASSERT_TRUE(headingChange >= 70 && headingChange <= 110);
}

void test_compass_heading_applies_calibration_and_mounting_offset()
{
    const AatCompass::Calibration calibration = {10, 20, 2, 1, 30, true};
    int32_t heading = -1;
    TEST_ASSERT_TRUE(AatCompass::calculateHeading(110, 20, calibration, &heading));
    TEST_ASSERT_EQUAL_INT32(30, heading);
}

void test_compass_heading_normalizes_wraparound()
{
    const AatCompass::Calibration calibration = {0, 0, 1, 1, 45, true};
    int32_t heading = -1;
    TEST_ASSERT_TRUE(AatCompass::calculateHeading(100, -1, calibration, &heading));
    TEST_ASSERT_EQUAL_INT32(46, heading);
}

void test_compass_heading_millidegrees_preserves_calibrated_fraction()
{
    const AatCompass::Calibration calibration = {0, 0, 1, 1, -1, true};
    int32_t headingMilliDegrees = -1;

    TEST_ASSERT_TRUE(AatCompass::calculateHeadingMilliDegrees(
        100, 1, calibration, &headingMilliDegrees));
    TEST_ASSERT_EQUAL_INT32(358427, headingMilliDegrees);
}

void test_compass_mounting_reversal_corrects_heading_before_north_offset()
{
    const AatCompass::Calibration fit = {25, -40, 2, 1, 15, true};
    int32_t start = 0, end = 0;
    TEST_ASSERT_TRUE(AatCompass::calculateHeadingMilliDegrees(75, -40, fit, &start));
    TEST_ASSERT_TRUE(AatCompass::calculateHeadingMilliDegrees(25, 60, fit, &end));
    TEST_ASSERT_EQUAL_INT32(15000, start);
    TEST_ASSERT_EQUAL_INT32(285000, end);
    TEST_ASSERT_TRUE(AatCompass::isQuarterTurn(-AatCompass::mountingTurnDegrees(start / 1000, end / 1000)));
    TEST_ASSERT_TRUE(AatCompass::calculateHeadingMilliDegrees(25, 60, fit, &end, true));
    TEST_ASSERT_EQUAL_INT32(105000, end);
    TEST_ASSERT_TRUE(AatCompass::isQuarterTurn(AatCompass::mountingTurnDegrees(start / 1000, end / 1000)));
    TEST_ASSERT_EQUAL_INT32(90, AatCompass::mountingTurnDegrees(350, 80));
    TEST_ASSERT_FALSE(AatCompass::isQuarterTurn(AatCompass::mountingTurnDegrees(350, 15)));
}

void test_compass_heading_rate_handles_wrap_and_zero_interval()
{
    int32_t rateMilliDegreesPerSecond = -1;
    TEST_ASSERT_TRUE(AatCompass::calculateHeadingRateMilliDegreesPerSecond(
        359900, 100, 100, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(2000, rateMilliDegreesPerSecond);

    TEST_ASSERT_FALSE(AatCompass::calculateHeadingRateMilliDegreesPerSecond(
        0, 100, 0, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(0, rateMilliDegreesPerSecond);
}

void test_compass_heading_rate_window_averages_sample_jitter()
{
    AatCompass::HeadingRateWindow window(500, 1000);
    int32_t rateMilliDegreesPerSecond = -1;

    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(0, 164358, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(100, 163301, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(200, 164505, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(300, 163900, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(400, 164300, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Updated,
        window.update(500, 164358, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(0, rateMilliDegreesPerSecond);
}

void test_compass_heading_rate_window_restarts_after_pulse_transition()
{
    AatCompass::HeadingRateWindow window(500, 1000);
    int32_t rateMilliDegreesPerSecond = 0;

    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(0, 0, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Updated,
        window.update(500, 5000, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(10000, rateMilliDegreesPerSecond);

    window.reset();
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(600, 10000, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(1099, 14990, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Updated,
        window.update(1100, 15000, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(10000, rateMilliDegreesPerSecond);
}

void test_compass_heading_rate_window_handles_wrap_and_stale_gaps()
{
    AatCompass::HeadingRateWindow window(500, 1000);
    int32_t rateMilliDegreesPerSecond = 0;

    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Waiting,
        window.update(1000, 359500, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Updated,
        window.update(1500, 500, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(2000, rateMilliDegreesPerSecond);

    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Reset,
        window.update(3001, 10000, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(0, rateMilliDegreesPerSecond);
    TEST_ASSERT_EQUAL(AatCompass::RateWindowUpdate::Updated,
        window.update(3501, 20000, &rateMilliDegreesPerSecond));
    TEST_ASSERT_EQUAL_INT32(20000, rateMilliDegreesPerSecond);
}

void test_compass_heading_rejects_invalid_calibration_and_zero_field()
{
    const AatCompass::Calibration invalidCalibration = {0, 0, 1, 1, 0, false};
    const AatCompass::Calibration zeroScale = {0, 0, 0, 1, 0, true};
    const AatCompass::Calibration validCalibration = {0, 0, 1, 1, 0, true};
    int32_t heading = 123;

    TEST_ASSERT_FALSE(AatCompass::calculateHeading(100, 0, invalidCalibration, &heading));
    TEST_ASSERT_EQUAL_INT32(0, heading);
    TEST_ASSERT_FALSE(AatCompass::calculateHeading(100, 0, zeroScale, &heading));
    TEST_ASSERT_FALSE(AatCompass::calculateHeading(0, 0, validCalibration, &heading));
    TEST_ASSERT_FALSE(AatCompass::calculateHeading(100, 0, validCalibration, nullptr));
}

void test_compass_calibration_fits_elliptical_sweep()
{
    AatCompassCalibration::Sweep sweep;
    for (int32_t angleDegrees = 0; angleDegrees < 360; angleDegrees += 5)
    {
        const float angle = angleDegrees * (3.14159265358979323846f / 180.0f);
        TEST_ASSERT_TRUE(sweep.addSample(
            static_cast<int16_t>(lroundf(120.0f + 300.0f * cosf(angle))),
            static_cast<int16_t>(lroundf(-80.0f + 150.0f * sinf(angle)))));
    }

    AatCompassCalibration::Result result = {};
    TEST_ASSERT_TRUE(sweep.fit(&result, true));
    TEST_ASSERT_EQUAL_UINT16(72, result.sampleCount);
    TEST_ASSERT_EQUAL_UINT8(12, result.coveredSectors);
    TEST_ASSERT_TRUE(result.returnClosureErrorDegrees <=
        AatCompassCalibration::MAX_RETURN_CLOSURE_ERROR_DEGREES);
    TEST_ASSERT_FALSE(result.fieldMagnitudeWarning);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 120.0f, result.calibration.offsetX);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -80.0f, result.calibration.offsetY);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.75f, result.calibration.scaleX);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, result.calibration.scaleY);
    TEST_ASSERT_TRUE(result.fieldMagnitudeMin > 0.0f);
    TEST_ASSERT_TRUE(result.fieldMagnitudeMax >= result.fieldMagnitudeMin);
}

void test_compass_calibration_retains_raw_xyz_and_elapsed_time()
{
    AatCompassCalibration::Sweep sweep;
    TEST_ASSERT_TRUE(sweep.addSample(120, -80, 35, 100));
    TEST_ASSERT_TRUE(sweep.addSample(-42, 91, -17, 650));

    AatCompassCalibration::Sample sample = {};
    TEST_ASSERT_TRUE(sweep.getSample(0, &sample));
    TEST_ASSERT_EQUAL_INT16(120, sample.x);
    TEST_ASSERT_EQUAL_INT16(-80, sample.y);
    TEST_ASSERT_EQUAL_INT16(35, sample.z);
    TEST_ASSERT_EQUAL_UINT32(100, sample.elapsedMs);

    TEST_ASSERT_TRUE(sweep.getSample(1, &sample));
    TEST_ASSERT_EQUAL_INT16(-42, sample.x);
    TEST_ASSERT_EQUAL_INT16(91, sample.y);
    TEST_ASSERT_EQUAL_INT16(-17, sample.z);
    TEST_ASSERT_EQUAL_UINT32(650, sample.elapsedMs);
    TEST_ASSERT_FALSE(sweep.getSample(2, &sample));
    TEST_ASSERT_FALSE(sweep.getSample(0, nullptr));
}

void test_compass_calibration_rejects_bad_return_closure()
{
    AatCompassCalibration::Sweep sweep;
    for (int32_t angleDegrees = 0; angleDegrees < 360; angleDegrees += 5)
    {
        const int32_t sampleAngle = angleDegrees == 355 ? 60 : angleDegrees;
        const float angle = sampleAngle * (3.14159265358979323846f / 180.0f);
        TEST_ASSERT_TRUE(sweep.addSample(
            static_cast<int16_t>(lroundf(300.0f * cosf(angle))),
            static_cast<int16_t>(lroundf(300.0f * sinf(angle)))));
    }

    AatCompassCalibration::Result result = {};
    TEST_ASSERT_FALSE(sweep.fit(&result, true));
    TEST_ASSERT_EQUAL_UINT8(12, result.coveredSectors);
    TEST_ASSERT_TRUE(result.returnClosureErrorDegrees >
        AatCompassCalibration::MAX_RETURN_CLOSURE_ERROR_DEGREES);
    TEST_ASSERT_EQUAL(AatCompassCalibration::Failure::ReturnClosureMismatch, result.failure);
}

void test_compass_calibration_warns_on_field_magnitude_spread()
{
    TEST_ASSERT_FALSE(AatCompassCalibration::hasFieldMagnitudeSpreadWarning(100.0f, 140.0f));
    TEST_ASSERT_TRUE(AatCompassCalibration::hasFieldMagnitudeSpreadWarning(100.0f, 151.0f));
    TEST_ASSERT_TRUE(AatCompassCalibration::hasFieldMagnitudeSpreadWarning(0.0f, 200.0f));
}

void test_compass_sweep_buffer_supports_full_turn_sample_rate()
{
    AatCompassCalibration::Sweep sweep;
    for (int16_t sample = 0; sample < 1500; ++sample)
        TEST_ASSERT_TRUE(sweep.addSample(sample, -sample));
    TEST_ASSERT_EQUAL_UINT16(1500, sweep.sampleCount());
}

void test_compass_calibration_rejects_missing_axis_variation()
{
    AatCompassCalibration::Sweep sweep;
    for (int32_t angleDegrees = 0; angleDegrees < 360; angleDegrees += 4)
    {
        const float angle = angleDegrees * (3.14159265358979323846f / 180.0f);
        TEST_ASSERT_TRUE(sweep.addSample(
            300,
            static_cast<int16_t>(lroundf(300.0f * sinf(angle)))));
    }

    AatCompassCalibration::Result result = {};
    TEST_ASSERT_FALSE(sweep.fit(&result, true));
    TEST_ASSERT_FALSE(result.valid);
    TEST_ASSERT_EQUAL_UINT16(90, result.sampleCount);
    TEST_ASSERT_EQUAL(AatCompassCalibration::Failure::NoXVariation, result.failure);
    TEST_ASSERT_EQUAL_UINT16(0, result.spanX);
    TEST_ASSERT_TRUE(result.spanY > 0);
    result = {};
    TEST_ASSERT_FALSE(sweep.fit(&result, false));
    TEST_ASSERT_EQUAL(AatCompassCalibration::Failure::FullRotationNotConfirmed, result.failure);
}

void test_site_compass_refresh_changes_offsets_but_preserves_scales()
{
    struct CalibrationFixture {
        int16_t offsetX;
        int16_t offsetY;
        uint16_t scaleXPermille;
        uint16_t scaleYPermille;
        int16_t headingOffsetDegrees;
        uint8_t valid;
    } saved = {10, 20, 800, 1200, 15, 1};
    AatCompassCalibration::Result siteFit = {};
    siteFit.valid = true;
    siteFit.calibration.offsetX = 34.0f;
    siteFit.calibration.offsetY = -47.0f;

    TEST_ASSERT_TRUE(AatCompassCalibration::refreshLocalOffset(saved, siteFit));
    TEST_ASSERT_EQUAL_INT16(34, saved.offsetX);
    TEST_ASSERT_EQUAL_INT16(-47, saved.offsetY);
    TEST_ASSERT_EQUAL_UINT16(800, saved.scaleXPermille);
    TEST_ASSERT_EQUAL_UINT16(1200, saved.scaleYPermille);
    TEST_ASSERT_EQUAL_INT16(15, saved.headingOffsetDegrees);
}

void test_two_to_one_servo_mode_maps_full_bearing_range()
{
    const int32_t servoLow[2] = {500, 1000};
    const int32_t servoHigh[2] = {2500, 2000};
    int32_t servoPosition[2] = {};
    AatServo::applyMode(-180, 90, AatServo::TwoToOne, servoLow, servoHigh, servoPosition);

    TEST_ASSERT_EQUAL_INT32(500, servoPosition[0]);
    TEST_ASSERT_EQUAL_INT32(2000, servoPosition[1]);

    AatServo::applyMode(179, 0, AatServo::TwoToOne, servoLow, servoHigh, servoPosition);
    TEST_ASSERT_EQUAL_INT32(2500, servoPosition[0]);
    TEST_ASSERT_EQUAL_INT32(1000, servoPosition[1]);
}

void test_continuous_pan_mode_maps_elevation_without_changing_pan()
{
    const int32_t servoLow[2] = {500, 1000};
    const int32_t servoHigh[2] = {2500, 2000};
    int32_t servoPosition[2] = {1520, 0};

    AatServo::applyMode(120, 0, AatServo::CompassContinuous, servoLow, servoHigh, servoPosition);
    TEST_ASSERT_EQUAL_INT32(1520, servoPosition[0]);
    TEST_ASSERT_EQUAL_INT32(1000, servoPosition[1]);

    AatServo::applyMode(-120, 90, AatServo::CompassContinuous, servoLow, servoHigh, servoPosition);
    TEST_ASSERT_EQUAL_INT32(1520, servoPosition[0]);
    TEST_ASSERT_EQUAL_INT32(2000, servoPosition[1]);
}

void test_clip180_servo_mode_clamps_bearing()
{
    const int32_t servoLow[2] = {500, 1000};
    const int32_t servoHigh[2] = {2500, 2000};
    int32_t servoPosition[2] = {};
    AatServo::applyMode(-120, 45, AatServo::Clip180, servoLow, servoHigh, servoPosition);

    TEST_ASSERT_EQUAL_INT32(500, servoPosition[0]);
    TEST_ASSERT_EQUAL_INT32(1500, servoPosition[1]);

    AatServo::applyMode(120, 45, AatServo::Clip180, servoLow, servoHigh, servoPosition);
    TEST_ASSERT_EQUAL_INT32(2500, servoPosition[0]);
}

void test_flip180_servo_mode_folds_bearing_and_elevation()
{
    const int32_t servoLow[2] = {500, 1000};
    const int32_t servoHigh[2] = {2500, 2000};
    int32_t servoPosition[2] = {};
    AatServo::applyMode(-120, 45, AatServo::Flip180, servoLow, servoHigh, servoPosition);

    TEST_ASSERT_EQUAL_INT32(833, servoPosition[0]);
    TEST_ASSERT_EQUAL_INT32(1750, servoPosition[1]);

    AatServo::applyMode(120, 45, AatServo::Flip180, servoLow, servoHigh, servoPosition);
    TEST_ASSERT_EQUAL_INT32(2166, servoPosition[0]);
    TEST_ASSERT_EQUAL_INT32(1750, servoPosition[1]);
}

void test_projection_caps_elapsed_time_to_one_update_interval()
{
    const int32_t projectedAzimuth = AatProjection::calculateProjectedAzimuth(
        true, 1000, 2100, 100, 10, 100, 350);

    TEST_ASSERT_EQUAL_INT32(90, projectedAzimuth);
}

void test_projection_limits_positive_and_negative_rotation_rate()
{
    const int32_t positiveProjection = AatProjection::calculateProjectedAzimuth(
        true, 1000, 1000, 0, 1, 100, 10);
    const int32_t negativeProjection = AatProjection::calculateProjectedAzimuth(
        true, 1000, 1000, 0, -1, 100, 10);

    TEST_ASSERT_EQUAL_INT32(110, positiveProjection);
    TEST_ASSERT_EQUAL_INT32(270, negativeProjection);
}

void test_projection_returns_current_azimuth_when_prerequisites_are_missing()
{
    TEST_ASSERT_EQUAL_INT32(123, AatProjection::calculateProjectedAzimuth(
        false, 1000, 1000, 0, 10, 100, 123));
    TEST_ASSERT_EQUAL_INT32(123, AatProjection::calculateProjectedAzimuth(
        true, 1000, 1000, 0, 10, 3, 123));
    TEST_ASSERT_EQUAL_INT32(123, AatProjection::calculateProjectedAzimuth(
        true, 0, 1000, 0, 10, 100, 123));
}

void test_servo_endpoints_require_nonzero_ranges_for_both_axes()
{
    TEST_ASSERT_TRUE(AatConfigValidation::areServoEndpointsValid(500, 2500, 1000, 2000));
    TEST_ASSERT_FALSE(AatConfigValidation::areServoEndpointsValid(500, 500, 1000, 2000));
    TEST_ASSERT_FALSE(AatConfigValidation::areServoEndpointsValid(500, 2500, 1000, 1000));
}

void test_pan_servo_neutral_validation()
{
    TEST_ASSERT_TRUE(AatConfigValidation::isPanServoNeutralValid(1450));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanServoNeutralValid(1550));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanServoNeutralValid(1449));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanServoNeutralValid(1551));
}

void test_pan_deadband_validation()
{
    TEST_ASSERT_TRUE(AatConfigValidation::isPanDeadbandValid(1));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanDeadbandValid(3));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanDeadbandValid(30));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanDeadbandValid(0));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanDeadbandValid(31));
}

void test_pan_pulse_calibration_validation()
{
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseCalibrationValid(
        1520, 1505, 1530, 40, 45));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseCalibrationValid(
        1520, 1480, 1565, 40, 45));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseCalibrationValid(
        1520, 1520, 1530, 40, 45));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseCalibrationValid(
        1520, 1479, 1530, 40, 45));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseCalibrationValid(
        1520, 1505, 1566, 40, 45));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseCalibrationValid(
        1520, 1450, 1600, 80, 90));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseCalibrationValid(
        1450, 1000, 1500, 450, 100));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseCalibrationValid(
        1550, 1500, 2000, 100, 450));
}

void test_pan_pulse_envelope_rejects_out_of_range_edits_before_narrowing()
{
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseEnvelopeValid(1450, 450, 550));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseEnvelopeValid(1520, 520, 480));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseEnvelopeValid(1550, 550, 450));
    TEST_ASSERT_TRUE(AatConfigValidation::isPanPulseEnvelopeValid(1520, 7, 12));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1520, 521, 480));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1520, 520, 481));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1520, -65529, 45));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1520, 65543, 45));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1520, 40, UINT32_MAX));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1520, 4, 45));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1449, 40, 45));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanPulseEnvelopeValid(1551, 40, 45));
}

void test_continuous_pan_profile_can_use_configured_absolute_endpoints()
{
    const AatServo::PanReference reference = {0, true};
    AatServo::PanCommand command = {};
    uint16_t pulseUs = 0;
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 270, 1520, 1480, 1000, 1565, 2000, 3, 0,
        &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(1000, pulseUs);
    TEST_ASSERT_TRUE(AatServo::calculateContinuousPanPulseProfile(
        reference, true, 0, 90, 1520, 1480, 1000, 1565, 2000, 3, 0,
        &pulseUs, &command));
    TEST_ASSERT_EQUAL_UINT16(2000, pulseUs);
}

void test_pan_compass_calibration_validation()
{
    struct CalibrationFixture {
        int16_t offsetX;
        int16_t offsetY;
        uint16_t scaleXPermille;
        uint16_t scaleYPermille;
        int16_t headingOffsetDegrees;
        uint8_t valid;
    } calibration = {120, -80, 750, 1500, 0, 1};

    TEST_ASSERT_TRUE(AatConfigValidation::isPanCompassCalibrationValid(calibration));
    calibration.scaleXPermille = 249;
    TEST_ASSERT_FALSE(AatConfigValidation::isPanCompassCalibrationValid(calibration));
    calibration.scaleXPermille = 750;
    calibration.headingOffsetDegrees = 181;
    TEST_ASSERT_FALSE(AatConfigValidation::isPanCompassCalibrationValid(calibration));
    calibration.headingOffsetDegrees = 0;
    calibration.valid = 0;
    TEST_ASSERT_FALSE(AatConfigValidation::isPanCompassCalibrationValid(calibration));
}

void test_pan_compass_north_alignment_requires_valid_axis_calibration()
{
    TEST_ASSERT_TRUE(AatConfigValidation::isPanCompassNorthAlignmentValid(true));
    TEST_ASSERT_FALSE(AatConfigValidation::isPanCompassNorthAlignmentValid(false));
}

void test_v5_aat_migration_preserves_values_and_defaults_neutral()
{
    struct CalibrationFixture {
        int16_t offsetX;
        int16_t offsetY;
        uint16_t scaleXPermille;
        uint16_t scaleYPermille;
        int16_t headingOffsetDegrees;
        uint8_t valid;
    };
    struct ConfigFixture {
        uint32_t version;
        uint8_t bootCount;
        uint8_t project;
        uint16_t azimuthLow;
        uint16_t panServoNeutralUs;
        CalibrationFixture panCompassCalibration;
        uint8_t panCompassNorthAligned;
    } config = {};
    config.version = 5;
    config.bootCount = 23;
    config.project = 0xff;
    config.azimuthLow = 500;

    TEST_ASSERT_TRUE(AatConfigMigration::migrateAatVrxV5(config, 5, 8,
        AatConfigDefaults::PAN_SERVO_NEUTRAL_US));
    TEST_ASSERT_EQUAL_UINT32(8, config.version);
    TEST_ASSERT_EQUAL_UINT8(23, config.bootCount);
    TEST_ASSERT_EQUAL_UINT8(0xff, config.project);
    TEST_ASSERT_EQUAL_UINT16(500, config.azimuthLow);
    TEST_ASSERT_EQUAL_UINT16(1520, config.panServoNeutralUs);
    TEST_ASSERT_EQUAL_UINT16(1000, config.panCompassCalibration.scaleXPermille);
    TEST_ASSERT_EQUAL_UINT8(0, config.panCompassCalibration.valid);
    TEST_ASSERT_EQUAL_UINT8(0, config.panCompassNorthAligned);
    TEST_ASSERT_FALSE(AatConfigMigration::migrateAatVrxV5(config, 5, 8,
        AatConfigDefaults::PAN_SERVO_NEUTRAL_US));
}

void test_v6_aat_migration_preserves_neutral_and_invalidates_compass_calibration()
{
    struct CalibrationFixture {
        int16_t offsetX;
        int16_t offsetY;
        uint16_t scaleXPermille;
        uint16_t scaleYPermille;
        int16_t headingOffsetDegrees;
        uint8_t valid;
    };
    struct ConfigFixture {
        uint32_t version;
        uint16_t panServoNeutralUs;
        CalibrationFixture panCompassCalibration;
        uint8_t panCompassNorthAligned;
    } config = {6, 1524, {10, 20, 800, 1200, 30, 1}, 1};

    TEST_ASSERT_TRUE(AatConfigMigration::migrateAatVrxV6(config, 6, 8));
    TEST_ASSERT_EQUAL_UINT32(8, config.version);
    TEST_ASSERT_EQUAL_UINT16(1524, config.panServoNeutralUs);
    TEST_ASSERT_EQUAL_UINT16(1000, config.panCompassCalibration.scaleXPermille);
    TEST_ASSERT_EQUAL_UINT8(0, config.panCompassCalibration.valid);
    TEST_ASSERT_EQUAL_UINT8(0, config.panCompassNorthAligned);

    config.panCompassCalibration = {10, 20, 800, 1200, 30, 1};
    config.panCompassNorthAligned = 1;
    AatConfigMigration::resetAatCompassSetup(config);
    TEST_ASSERT_EQUAL_UINT32(8, config.version);
    TEST_ASSERT_EQUAL_UINT16(1524, config.panServoNeutralUs);
    TEST_ASSERT_EQUAL_INT16(0, config.panCompassCalibration.offsetX);
    TEST_ASSERT_EQUAL_INT16(0, config.panCompassCalibration.offsetY);
    TEST_ASSERT_EQUAL_UINT16(1000, config.panCompassCalibration.scaleXPermille);
    TEST_ASSERT_EQUAL_UINT16(1000, config.panCompassCalibration.scaleYPermille);
    TEST_ASSERT_EQUAL_INT16(0, config.panCompassCalibration.headingOffsetDegrees);
    TEST_ASSERT_EQUAL_UINT8(0, config.panCompassCalibration.valid);
    TEST_ASSERT_EQUAL_UINT8(0, config.panCompassNorthAligned);
}

void test_v7_aat_migration_preserves_compass_fit_but_requires_north_alignment()
{
    struct CalibrationFixture {
        int16_t offsetX;
        int16_t offsetY;
        uint16_t scaleXPermille;
        uint16_t scaleYPermille;
        int16_t headingOffsetDegrees;
        uint8_t valid;
    };
    struct ConfigFixture {
        uint32_t version;
        uint16_t panServoNeutralUs;
        CalibrationFixture panCompassCalibration;
        uint8_t panCompassNorthAligned;
    } config = {7, 1520, {151, -59, 1109, 911, 0, 1}, 1};

    TEST_ASSERT_TRUE(AatConfigMigration::migrateAatVrxV7(config, 7, 8));
    TEST_ASSERT_EQUAL_UINT32(8, config.version);
    TEST_ASSERT_EQUAL_UINT16(1520, config.panServoNeutralUs);
    TEST_ASSERT_EQUAL_INT16(151, config.panCompassCalibration.offsetX);
    TEST_ASSERT_EQUAL_UINT16(1109, config.panCompassCalibration.scaleXPermille);
    TEST_ASSERT_EQUAL_UINT8(1, config.panCompassCalibration.valid);
    TEST_ASSERT_EQUAL_UINT8(0, config.panCompassNorthAligned);
}

void test_v8_aat_migration_preserves_calibration_and_defaults_deadband()
{
    struct CalibrationFixture {
        int16_t offsetX;
        int16_t offsetY;
        uint16_t scaleXPermille;
        uint16_t scaleYPermille;
        int16_t headingOffsetDegrees;
        uint8_t valid;
    };
    struct ConfigFixture {
        uint32_t version;
        uint16_t panServoNeutralUs;
        CalibrationFixture panCompassCalibration;
        uint8_t panCompassNorthAligned;
        uint8_t panDeadbandDegrees;
    } config = {8, 1520, {151, -59, 1109, 911, 34, 1}, 1, 0};

    TEST_ASSERT_TRUE(AatConfigMigration::migrateAatVrxV8(config, 8, 9,
        AatConfigDefaults::PAN_DEADBAND_DEFAULT_DEGREES));
    TEST_ASSERT_EQUAL_UINT32(9, config.version);
    TEST_ASSERT_EQUAL_UINT16(1520, config.panServoNeutralUs);
    TEST_ASSERT_EQUAL_INT16(151, config.panCompassCalibration.offsetX);
    TEST_ASSERT_EQUAL_INT16(-59, config.panCompassCalibration.offsetY);
    TEST_ASSERT_EQUAL_UINT16(1109, config.panCompassCalibration.scaleXPermille);
    TEST_ASSERT_EQUAL_UINT16(911, config.panCompassCalibration.scaleYPermille);
    TEST_ASSERT_EQUAL_INT16(34, config.panCompassCalibration.headingOffsetDegrees);
    TEST_ASSERT_EQUAL_UINT8(1, config.panCompassCalibration.valid);
    TEST_ASSERT_EQUAL_UINT8(1, config.panCompassNorthAligned);
    TEST_ASSERT_EQUAL_UINT8(3, config.panDeadbandDegrees);
}

void test_v8_non_aat_migration_preserves_values()
{
    struct ConfigFixture {
        uint32_t version;
        uint8_t bootCount;
        uint8_t setting;
    } config = {8, 17, 91};

    TEST_ASSERT_TRUE(AatConfigMigration::migrateVrxV8(config, 8, 9));
    TEST_ASSERT_EQUAL_UINT32(9, config.version);
    TEST_ASSERT_EQUAL_UINT8(17, config.bootCount);
    TEST_ASSERT_EQUAL_UINT8(91, config.setting);
}

void test_v9_aat_migration_preserves_values_and_requires_pulse_calibration()
{
    struct ConfigFixture {
        uint32_t version;
        uint8_t bootCount;
        uint8_t deadband;
        uint16_t panLeftStartPulseUs;
        uint16_t panRightStartPulseUs;
        uint8_t panPulseCalibrationValid;
        uint16_t panLeftMaxOffsetUs;
        uint16_t panRightMaxOffsetUs;
    } config = {9, 23, 7, 0, 0, 1, 0, 0};

    TEST_ASSERT_TRUE(AatConfigMigration::migrateAatVrxV9(
        config, 9, 10, 1520, AatConfigDefaults::PAN_LEFT_MAX_OFFSET_US,
        AatConfigDefaults::PAN_RIGHT_MAX_OFFSET_US));
    TEST_ASSERT_EQUAL_UINT32(10, config.version);
    TEST_ASSERT_EQUAL_UINT8(23, config.bootCount);
    TEST_ASSERT_EQUAL_UINT8(7, config.deadband);
    TEST_ASSERT_EQUAL_UINT16(1480, config.panLeftStartPulseUs);
    TEST_ASSERT_EQUAL_UINT16(1565, config.panRightStartPulseUs);
    TEST_ASSERT_EQUAL_UINT16(40, config.panLeftMaxOffsetUs);
    TEST_ASSERT_EQUAL_UINT16(45, config.panRightMaxOffsetUs);
    TEST_ASSERT_EQUAL_UINT8(0, config.panPulseCalibrationValid);
}

void test_v5_non_aat_migration_preserves_values()
{
    struct ConfigFixture {
        uint32_t version;
        uint8_t bootCount;
        uint8_t setting;
    } config = {5, 7, 42};

    TEST_ASSERT_TRUE(AatConfigMigration::migrateVrxV5(config, 5, 8));
    TEST_ASSERT_EQUAL_UINT32(8, config.version);
    TEST_ASSERT_EQUAL_UINT8(7, config.bootCount);
    TEST_ASSERT_EQUAL_UINT8(42, config.setting);
}

void test_legacy_aat_defaults_preserve_projection_and_servo_settings()
{
    struct ServoEndpoints { uint16_t low; uint16_t high; };
    struct AatSettings {
        uint8_t satelliteHomeMin;
        uint8_t project;
        uint8_t servoSmooth;
        uint8_t centerDir;
        uint8_t azimuthServoFastFlip;
        ServoEndpoints servoEndpoints[2];
    } aat = {};
    struct BatterySettings { uint16_t scale; int16_t offset; } battery = {};

    AatConfigDefaults::apply(aat, battery);

    TEST_ASSERT_EQUAL_UINT8(5, aat.satelliteHomeMin);
    TEST_ASSERT_EQUAL_UINT8(0xff, aat.project);
    TEST_ASSERT_EQUAL_UINT8(5, aat.servoSmooth);
    TEST_ASSERT_EQUAL_UINT8(0, aat.centerDir);
    TEST_ASSERT_EQUAL_UINT8(1, aat.azimuthServoFastFlip);
    TEST_ASSERT_EQUAL_UINT16(500, aat.servoEndpoints[0].low);
    TEST_ASSERT_EQUAL_UINT16(2500, aat.servoEndpoints[0].high);
    TEST_ASSERT_EQUAL_UINT16(1000, aat.servoEndpoints[1].low);
    TEST_ASSERT_EQUAL_UINT16(2000, aat.servoEndpoints[1].high);
    TEST_ASSERT_EQUAL_UINT16(292, battery.scale);
    TEST_ASSERT_EQUAL_INT16(-2, battery.offset);
}

int main(int argumentCount, char **arguments)
{
    UNITY_BEGIN();
    RUN_TEST(test_eastward_bearing_and_distance);
    RUN_TEST(test_westward_bearing_is_normalized);
    RUN_TEST(test_antimeridian_distance_uses_wide_longitude_delta);
    RUN_TEST(test_elevation);
    RUN_TEST(test_continuous_pan_uses_shortest_path_across_zero);
    RUN_TEST(test_continuous_pan_uses_positive_direction_for_half_turn_tie);
    RUN_TEST(test_active_servo_sweep_bypasses_manual_pulse_timeout);
    RUN_TEST(test_manual_pulse_timeout_handles_future_and_wrapped_timestamps);
    RUN_TEST(test_manual_pulse_uses_three_second_timeout);
    RUN_TEST(test_manual_pulse_clear_preserves_a_newer_request);
    RUN_TEST(test_manual_pulse_clear_retires_unchanged_timed_out_request);
    RUN_TEST(test_manual_pan_pulse_requires_reference_and_fresh_compass);
    RUN_TEST(test_manual_pan_pulse_is_limited_to_a_safe_test_window);
    RUN_TEST(test_pan_command_converts_compass_headings_relative_to_home);
    RUN_TEST(test_pan_command_clamps_target_to_soft_limits);
    RUN_TEST(test_pan_command_uses_linear_error_within_limited_sweep);
    RUN_TEST(test_pan_command_rejects_unhomed_or_invalid_compass);
    RUN_TEST(test_pan_command_rejects_current_position_outside_soft_limits);
    RUN_TEST(test_pan_reference_requires_confirmation_neutral_and_fresh_compass);
    RUN_TEST(test_pan_reference_is_required_and_commands_stay_within_soft_limits);
    RUN_TEST(test_failed_pan_reference_clears_previous_reference);
    RUN_TEST(test_pan_safety_requires_button_release_after_boot);
    RUN_TEST(test_continuous_pan_pulse_maps_error_and_deadband);
    RUN_TEST(test_continuous_pan_uses_configured_deadband);
    RUN_TEST(test_continuous_pan_profile_tapers_approach_and_preserves_direction);
    RUN_TEST(test_continuous_pan_profile_applies_existing_speed_setting);
    RUN_TEST(test_continuous_pan_profile_scales_approach_once_and_is_monotonic);
    RUN_TEST(test_tracking_pan_median_requires_distinct_samples_and_resets);
    RUN_TEST(test_tracking_pan_median_suppresses_single_outlier_across_north);
    RUN_TEST(test_filtered_pan_pulse_respects_raw_direction_and_deadband);
    RUN_TEST(test_tracking_pan_filter_does_not_bypass_raw_travel_guard);
    RUN_TEST(test_pulse_calibration_requires_fresh_heading_windows_and_neutral_reversal);
    RUN_TEST(test_pulse_calibration_clamps_non_multiple_step_endpoints);
    RUN_TEST(test_pulse_calibration_aborts_on_feedback_direction_travel_and_cancel);
    RUN_TEST(test_pulse_calibration_handles_missing_headings_settle_timeout_and_clock_wrap);
    RUN_TEST(test_pulse_calibration_rejects_output_loss_and_duplicate_start);
    RUN_TEST(test_pulse_calibration_simulates_dead_zone_inertia_and_complete_restart);
    RUN_TEST(test_pulse_calibration_tolerates_measured_stationary_jitter_without_inventing_motion);
    RUN_TEST(test_pulse_calibration_rejects_duplicate_sparse_and_invalid_heading_evidence);
    RUN_TEST(test_pulse_calibration_simulates_inertia_with_bounded_jitter_in_both_directions);
    RUN_TEST(test_pulse_calibration_keeps_raw_travel_guards_ahead_of_heading_filter);
    RUN_TEST(test_pulse_calibration_stops_sustained_opposite_motion_despite_jitter);
    RUN_TEST(test_simulated_drone_positions_drive_loaded_pan_and_settle_inside_deadband);
    RUN_TEST(test_continuous_pan_gps_freshness_caps_long_gaps_and_keeps_fast_timeout);
    RUN_TEST(test_simulated_aircraft_gps_loss_neutralizes_and_compass_loss_requires_reference);
    RUN_TEST(test_continuous_pan_pulse_handles_wrap_and_rejects_unsafe_state);
    RUN_TEST(test_compass_startup_alignment_requires_operator_without_overwriting_north);
    RUN_TEST(test_compass_pan_setup_state_inhibits_tracking_until_fit_and_alignment);
    RUN_TEST(test_gpio32_press_edge_requires_stable_new_press);
    RUN_TEST(test_sweep_timeout_only_applies_after_rotation_starts);
    RUN_TEST(test_sweep_starts_on_hold_and_finishes_on_release);
    RUN_TEST(test_setup_sweep_wait_state_stays_neutral);
    RUN_TEST(test_compass_configuration_must_match_before_reference);
    RUN_TEST(test_compass_sample_release_requires_clear_post_trigger_lock);
    RUN_TEST(test_compass_pan_sample_validity_requires_every_safety_gate);
    RUN_TEST(test_continuous_pan_command_gate_requires_gps_compass_and_reference);
    RUN_TEST(test_continuous_pan_block_reason_covers_each_fault_and_recovery);
    RUN_TEST(test_compass_north_offset_uses_shortest_wrap_safe_delta);
    RUN_TEST(test_pan_safety_requires_neutral_actuator_and_fresh_compass);
    RUN_TEST(test_pan_safety_latches_compass_and_travel_faults);
    RUN_TEST(test_pan_safety_invalidation_requires_release_and_new_reference);
    RUN_TEST(test_pan_safety_rejects_skipped_rear_boundary_in_both_directions);
    RUN_TEST(test_pan_safety_checks_rear_crossing_during_reference_updates);
    RUN_TEST(test_pan_safety_allows_direct_travel_and_north_wrap);
    RUN_TEST(test_pan_feedback_freshness_uses_output_time_after_stall);
    RUN_TEST(test_compass_heading_uses_cardinal_directions);
    RUN_TEST(test_frd_compass_clockwise_quarter_turn_increases_heading);
    RUN_TEST(test_compass_heading_applies_calibration_and_mounting_offset);
    RUN_TEST(test_compass_heading_normalizes_wraparound);
    RUN_TEST(test_compass_heading_millidegrees_preserves_calibrated_fraction);
    RUN_TEST(test_compass_mounting_reversal_corrects_heading_before_north_offset);
    RUN_TEST(test_compass_heading_rate_handles_wrap_and_zero_interval);
    RUN_TEST(test_compass_heading_rate_window_averages_sample_jitter);
    RUN_TEST(test_compass_heading_rate_window_restarts_after_pulse_transition);
    RUN_TEST(test_compass_heading_rate_window_handles_wrap_and_stale_gaps);
    RUN_TEST(test_compass_heading_rejects_invalid_calibration_and_zero_field);
    RUN_TEST(test_compass_calibration_fits_elliptical_sweep);
    RUN_TEST(test_compass_calibration_retains_raw_xyz_and_elapsed_time);
    RUN_TEST(test_compass_calibration_rejects_bad_return_closure);
    RUN_TEST(test_compass_calibration_warns_on_field_magnitude_spread);
    RUN_TEST(test_compass_sweep_buffer_supports_full_turn_sample_rate);
    RUN_TEST(test_compass_calibration_rejects_missing_axis_variation);
    RUN_TEST(test_site_compass_refresh_changes_offsets_but_preserves_scales);
    RUN_TEST(test_two_to_one_servo_mode_maps_full_bearing_range);
    RUN_TEST(test_continuous_pan_mode_maps_elevation_without_changing_pan);
    RUN_TEST(test_clip180_servo_mode_clamps_bearing);
    RUN_TEST(test_flip180_servo_mode_folds_bearing_and_elevation);
    RUN_TEST(test_projection_caps_elapsed_time_to_one_update_interval);
    RUN_TEST(test_projection_limits_positive_and_negative_rotation_rate);
    RUN_TEST(test_projection_returns_current_azimuth_when_prerequisites_are_missing);
    RUN_TEST(test_servo_endpoints_require_nonzero_ranges_for_both_axes);
    RUN_TEST(test_pan_servo_neutral_validation);
    RUN_TEST(test_pan_deadband_validation);
    RUN_TEST(test_pan_pulse_calibration_validation);
    RUN_TEST(test_pan_pulse_envelope_rejects_out_of_range_edits_before_narrowing);
    RUN_TEST(test_continuous_pan_profile_can_use_configured_absolute_endpoints);
    RUN_TEST(test_pan_compass_calibration_validation);
    RUN_TEST(test_pan_compass_north_alignment_requires_valid_axis_calibration);
    RUN_TEST(test_v5_aat_migration_preserves_values_and_defaults_neutral);
    RUN_TEST(test_v6_aat_migration_preserves_neutral_and_invalidates_compass_calibration);
    RUN_TEST(test_v7_aat_migration_preserves_compass_fit_but_requires_north_alignment);
    RUN_TEST(test_v8_aat_migration_preserves_calibration_and_defaults_deadband);
    RUN_TEST(test_v8_non_aat_migration_preserves_values);
    RUN_TEST(test_v9_aat_migration_preserves_values_and_requires_pulse_calibration);
    RUN_TEST(test_v5_non_aat_migration_preserves_values);
    RUN_TEST(test_legacy_aat_defaults_preserve_projection_and_servo_settings);
    return UNITY_END();
}