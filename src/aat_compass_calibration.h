#pragma once

#include <math.h>
#include <stdint.h>

#include "aat_compass.h"

namespace AatCompassCalibration
{
static constexpr uint16_t MAX_SAMPLES = 2048;
static constexpr uint16_t MIN_SAMPLES = 36;
static constexpr uint8_t SECTOR_COUNT = 12;
static constexpr uint8_t MIN_COVERED_SECTORS = 10;
static constexpr uint16_t MAX_RETURN_CLOSURE_ERROR_DEGREES = 20;
static constexpr float MAX_FIELD_MAGNITUDE_RATIO_WARNING = 1.5f;

inline bool hasFieldMagnitudeSpreadWarning(float minimum, float maximum)
{
    return !isfinite(minimum) || !isfinite(maximum) || minimum <= 0.0f
        || maximum / minimum > MAX_FIELD_MAGNITUDE_RATIO_WARNING;
}

enum class Failure : uint8_t
{
    None,
    FullRotationNotConfirmed,
    TooFewSamples,
    NoXVariation,
    NoYVariation,
    InsufficientCoverage,
    ReturnClosureMismatch
};

struct Result
{
    AatCompass::Calibration calibration;
    uint16_t sampleCount;
    uint16_t spanX;
    uint16_t spanY;
    uint16_t sectorMask;
    uint8_t coveredSectors;
    float fieldMagnitudeMin;
    float fieldMagnitudeMax;
    float fieldMagnitudeRatio;
    uint16_t returnClosureErrorDegrees;
    bool fieldMagnitudeWarning;
    Failure failure;
    bool valid;
};

struct Sample
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint32_t elapsedMs;
};

class Sweep
{
public:
    void reset() { _sampleCount = 0; }
    uint16_t sampleCount() const { return _sampleCount; }

    bool addSample(int16_t x, int16_t y, int16_t z = 0, uint32_t elapsedMs = 0)
    {
        if (_sampleCount >= MAX_SAMPLES)
            return false;
        _x[_sampleCount] = x;
        _y[_sampleCount] = y;
        _z[_sampleCount] = z;
        _elapsedMs[_sampleCount] = elapsedMs;
        ++_sampleCount;
        return true;
    }

    bool getSample(uint16_t index, Sample *sample) const
    {
        if (sample == nullptr || index >= _sampleCount)
            return false;
        sample->x = _x[index];
        sample->y = _y[index];
        sample->z = _z[index];
        sample->elapsedMs = _elapsedMs[index];
        return true;
    }

    bool fit(Result *result, bool fullRotationConfirmed) const
    {
        if (result == nullptr)
            return false;
        *result = {};
        result->sampleCount = _sampleCount;
        if (!fullRotationConfirmed)
        {
            result->failure = Failure::FullRotationNotConfirmed;
            return false;
        }
        if (_sampleCount < MIN_SAMPLES)
        {
            result->failure = Failure::TooFewSamples;
            return false;
        }

        int16_t minX = INT16_MAX;
        int16_t maxX = INT16_MIN;
        int16_t minY = INT16_MAX;
        int16_t maxY = INT16_MIN;
        for (uint16_t index = 0; index < _sampleCount; ++index)
        {
            if (_x[index] < minX) minX = _x[index];
            if (_x[index] > maxX) maxX = _x[index];
            if (_y[index] < minY) minY = _y[index];
            if (_y[index] > maxY) maxY = _y[index];
        }

        result->spanX = static_cast<uint16_t>(maxX - minX);
        result->spanY = static_cast<uint16_t>(maxY - minY);
        const float radiusX = result->spanX * 0.5f;
        const float radiusY = result->spanY * 0.5f;
        if (radiusX <= 0.0f)
        {
            result->failure = Failure::NoXVariation;
            return false;
        }
        if (radiusY <= 0.0f)
        {
            result->failure = Failure::NoYVariation;
            return false;
        }

        const float averageRadius = (radiusX + radiusY) * 0.5f;
        result->calibration.offsetX = (maxX + minX) * 0.5f;
        result->calibration.offsetY = (maxY + minY) * 0.5f;
        result->calibration.scaleX = averageRadius / radiusX;
        result->calibration.scaleY = averageRadius / radiusY;
        result->calibration.headingOffsetDegrees = 0;
        result->calibration.valid = true;
        result->fieldMagnitudeMin = INFINITY;
        result->fieldMagnitudeMax = 0.0f;

        for (uint16_t index = 0; index < _sampleCount; ++index)
        {
            const float x = (_x[index] - result->calibration.offsetX) * result->calibration.scaleX;
            const float y = (_y[index] - result->calibration.offsetY) * result->calibration.scaleY;
            const float magnitude = sqrtf(x * x + y * y);
            if (magnitude < result->fieldMagnitudeMin)
                result->fieldMagnitudeMin = magnitude;
            if (magnitude > result->fieldMagnitudeMax)
                result->fieldMagnitudeMax = magnitude;

            float heading = AatCompass::headingFromForwardRightXY(x, y);
            if (heading < 0.0f)
                heading += 360.0f;
            uint8_t sector = static_cast<uint8_t>(heading / (360.0f / SECTOR_COUNT));
            if (sector >= SECTOR_COUNT)
                sector = SECTOR_COUNT - 1;
            result->sectorMask |= static_cast<uint16_t>(1U << sector);
        }

        for (uint8_t sector = 0; sector < SECTOR_COUNT; ++sector)
            if ((result->sectorMask & (1U << sector)) != 0)
                ++result->coveredSectors;

        const float startX = (_x[0] - result->calibration.offsetX) * result->calibration.scaleX;
        const float startY = (_y[0] - result->calibration.offsetY) * result->calibration.scaleY;
        const float endX = (_x[_sampleCount - 1] - result->calibration.offsetX)
            * result->calibration.scaleX;
        const float endY = (_y[_sampleCount - 1] - result->calibration.offsetY)
            * result->calibration.scaleY;
        const float startHeading = AatCompass::headingFromForwardRightXY(startX, startY);
        const float endHeading = AatCompass::headingFromForwardRightXY(endX, endY);
        float closureError = fabsf(endHeading - startHeading);
        if (closureError > 180.0f)
            closureError = 360.0f - closureError;
        result->returnClosureErrorDegrees = static_cast<uint16_t>(lroundf(closureError));
        result->fieldMagnitudeRatio = result->fieldMagnitudeMin > 0.0f
            ? result->fieldMagnitudeMax / result->fieldMagnitudeMin : INFINITY;
        result->fieldMagnitudeWarning = hasFieldMagnitudeSpreadWarning(
            result->fieldMagnitudeMin, result->fieldMagnitudeMax);

        result->valid = result->coveredSectors >= MIN_COVERED_SECTORS
            && result->returnClosureErrorDegrees <= MAX_RETURN_CLOSURE_ERROR_DEGREES;
        result->calibration.valid = result->valid;
        result->failure = result->coveredSectors < MIN_COVERED_SECTORS
            ? Failure::InsufficientCoverage
            : result->valid ? Failure::None : Failure::ReturnClosureMismatch;
        return result->valid;
    }

private:
    int16_t _x[MAX_SAMPLES] = {};
    int16_t _y[MAX_SAMPLES] = {};
    int16_t _z[MAX_SAMPLES] = {};
    uint32_t _elapsedMs[MAX_SAMPLES] = {};
    uint16_t _sampleCount = 0;
};

template <typename Calibration>
inline bool refreshLocalOffset(Calibration &saved, const Result &siteFit)
{
    if (!saved.valid || !siteFit.valid)
        return false;

    saved.offsetX = static_cast<decltype(saved.offsetX)>(lroundf(siteFit.calibration.offsetX));
    saved.offsetY = static_cast<decltype(saved.offsetY)>(lroundf(siteFit.calibration.offsetY));
    return true;
}
}