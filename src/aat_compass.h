#pragma once

#include <math.h>
#include <stdint.h>

namespace AatCompass
{
inline bool isCompassSampleReleaseConfirmed(bool statusAfterTriggerValid, uint8_t statusAfterTrigger)
{
    constexpr uint8_t STATUS_LOCK = 0x02;
    return statusAfterTriggerValid && (statusAfterTrigger & STATUS_LOCK) == 0;
}

struct Calibration
{
    float offsetX;
    float offsetY;
    float scaleX;
    float scaleY;
    int32_t headingOffsetDegrees;
    bool valid;
};

inline int32_t normalizeHeading(int32_t headingDegrees)
{
    headingDegrees %= 360;
    if (headingDegrees < 0)
        headingDegrees += 360;
    return headingDegrees;
}

inline int32_t normalizeHeadingMilliDegrees(int64_t headingMilliDegrees)
{
    headingMilliDegrees %= 360000;
    if (headingMilliDegrees < 0)
        headingMilliDegrees += 360000;
    return static_cast<int32_t>(headingMilliDegrees);
}

inline float headingFromForwardRightXY(float x, float y)
{
    // The AAT compass is mounted in a forward-right-down (FRD) sensor frame.
    return atan2f(-y, x) * (180.0f / 3.14159265358979323846f);
}

inline bool calculateCompassHeadingOffsetDegrees(int32_t sensorHeadingMilliDegrees,
    int32_t referenceBearingDegrees, int16_t *headingOffsetDegrees)
{
    if (headingOffsetDegrees == nullptr)
        return false;
    *headingOffsetDegrees = 0;
    if (referenceBearingDegrees < 0 || referenceBearingDegrees >= 360)
        return false;

    const int32_t sensorHeadingDegrees = normalizeHeadingMilliDegrees(
        static_cast<int64_t>(sensorHeadingMilliDegrees) + 500) / 1000;
    int32_t offset = referenceBearingDegrees - sensorHeadingDegrees;
    if (offset > 180)
        offset -= 360;
    else if (offset < -180)
        offset += 360;
    else if (offset == -180)
        offset = 180;

    *headingOffsetDegrees = static_cast<int16_t>(offset);
    return true;
}

inline bool calculateHeadingMilliDegrees(int16_t rawX, int16_t rawY,
    const Calibration &calibration, int32_t *headingMilliDegrees, bool reversed = false)
{
    if (headingMilliDegrees == nullptr)
        return false;
    *headingMilliDegrees = 0;

    if (!calibration.valid ||
        !isfinite(calibration.offsetX) || !isfinite(calibration.offsetY) ||
        !isfinite(calibration.scaleX) || !isfinite(calibration.scaleY) ||
        !isfinite(static_cast<float>(calibration.headingOffsetDegrees)) ||
        calibration.scaleX <= 0.0f || calibration.scaleY <= 0.0f)
        return false;

    const float x = (rawX - calibration.offsetX) * calibration.scaleX;
    const float y = (rawY - calibration.offsetY) * calibration.scaleY;
    if (!isfinite(x) || !isfinite(y) || (x == 0.0f && y == 0.0f))
        return false;

    const float heading = headingFromForwardRightXY(x, reversed ? -y : y) * 1000.0f;
    const int64_t correctedHeading = static_cast<int64_t>(lroundf(heading))
        + static_cast<int64_t>(calibration.headingOffsetDegrees) * 1000;
    *headingMilliDegrees = normalizeHeadingMilliDegrees(correctedHeading);
    return true;
}

inline int32_t mountingTurnDegrees(int32_t startHeading, int32_t endHeading)
{
    return normalizeHeading(endHeading - startHeading + 180) - 180;
}

inline bool isQuarterTurn(int32_t signedTurnDegrees)
{
    return signedTurnDegrees >= 70 && signedTurnDegrees <= 110;
}

inline bool calculateHeadingRateMilliDegreesPerSecond(int32_t previousHeadingMilliDegrees,
    int32_t currentHeadingMilliDegrees, uint32_t intervalMs, int32_t *rateMilliDegreesPerSecond)
{
    if (rateMilliDegreesPerSecond == nullptr)
        return false;
    *rateMilliDegreesPerSecond = 0;
    if (intervalMs == 0)
        return false;

    const int32_t previous = normalizeHeadingMilliDegrees(previousHeadingMilliDegrees);
    const int32_t current = normalizeHeadingMilliDegrees(currentHeadingMilliDegrees);
    int32_t delta = current - previous;
    if (delta > 180000)
        delta -= 360000;
    else if (delta < -180000)
        delta += 360000;
    else if (delta == -180000)
        delta = 180000;

    *rateMilliDegreesPerSecond = static_cast<int32_t>(
        (static_cast<int64_t>(delta) * 1000) / intervalMs);
    return true;
}

enum class RateWindowUpdate : uint8_t
{
    Waiting,
    Updated,
    Reset
};

class HeadingRateWindow
{
public:
    HeadingRateWindow(uint32_t windowMs, uint32_t maxIntervalMs)
        : _windowMs(windowMs), _maxIntervalMs(maxIntervalMs)
    {
    }

    void reset()
    {
        _hasHeading = false;
        _startAtMs = 0;
        _startHeadingMilliDegrees = 0;
    }

    RateWindowUpdate update(uint32_t nowMs, int32_t headingMilliDegrees,
        int32_t *rateMilliDegreesPerSecond)
    {
        if (rateMilliDegreesPerSecond == nullptr)
            return RateWindowUpdate::Reset;
        *rateMilliDegreesPerSecond = 0;
        const int32_t heading = normalizeHeadingMilliDegrees(headingMilliDegrees);
        if (!_hasHeading)
        {
            setBaseline(nowMs, heading);
            return RateWindowUpdate::Waiting;
        }

        const uint32_t intervalMs = nowMs - _startAtMs;
        if (intervalMs < _windowMs)
            return RateWindowUpdate::Waiting;
        if (intervalMs > _maxIntervalMs
            || !calculateHeadingRateMilliDegreesPerSecond(_startHeadingMilliDegrees,
                heading, intervalMs, rateMilliDegreesPerSecond))
        {
            setBaseline(nowMs, heading);
            return RateWindowUpdate::Reset;
        }

        setBaseline(nowMs, heading);
        return RateWindowUpdate::Updated;
    }

private:
    void setBaseline(uint32_t nowMs, int32_t headingMilliDegrees)
    {
        _startAtMs = nowMs;
        _startHeadingMilliDegrees = headingMilliDegrees;
        _hasHeading = true;
    }

    uint32_t _windowMs;
    uint32_t _maxIntervalMs;
    uint32_t _startAtMs = 0;
    int32_t _startHeadingMilliDegrees = 0;
    bool _hasHeading = false;
};

inline bool calculateHeading(int16_t rawX, int16_t rawY,
    const Calibration &calibration, int32_t *headingDegrees)
{
    if (headingDegrees == nullptr)
        return false;
    *headingDegrees = 0;

    if (!calibration.valid ||
        !isfinite(calibration.offsetX) || !isfinite(calibration.offsetY) ||
        !isfinite(calibration.scaleX) || !isfinite(calibration.scaleY) ||
        !isfinite(static_cast<float>(calibration.headingOffsetDegrees)) ||
        calibration.scaleX <= 0.0f || calibration.scaleY <= 0.0f)
        return false;

    const float x = (rawX - calibration.offsetX) * calibration.scaleX;
    const float y = (rawY - calibration.offsetY) * calibration.scaleY;
    if (!isfinite(x) || !isfinite(y) || (x == 0.0f && y == 0.0f))
        return false;

    const float heading = headingFromForwardRightXY(x, y);
    *headingDegrees = normalizeHeading(
        static_cast<int32_t>(lroundf(heading)) + calibration.headingOffsetDegrees);
    return true;
}
}