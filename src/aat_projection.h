#pragma once

#include <stdint.h>

namespace AatProjection
{
inline int32_t calculateProjectedAzimuth(bool enabled, uint32_t averageUpdateIntervalMs,
    uint32_t now, uint32_t lastUpdateMs, int32_t millisecondsPerDegree,
    uint32_t targetDistance, int32_t targetAzimuth)
{
    if (!enabled || !averageUpdateIntervalMs || !millisecondsPerDegree || targetDistance <= 3)
        return targetAzimuth;

    uint32_t elapsed = now - lastUpdateMs;
    if (elapsed > averageUpdateIntervalMs)
        elapsed = averageUpdateIntervalMs;

    // Limit projected rotation to 100 degrees per second.
    if (millisecondsPerDegree > -10 && millisecondsPerDegree < 10)
        millisecondsPerDegree = millisecondsPerDegree > 0 ? 10 : -10;

    return (((int32_t)elapsed / millisecondsPerDegree) + targetAzimuth + 360) % 360;
}
}