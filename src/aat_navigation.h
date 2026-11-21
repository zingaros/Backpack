#pragma once

#include <math.h>
#include <stdint.h>

namespace AatNavigation
{
constexpr double PI_VALUE = 3.14159265358979323846;

inline double degreesToRadians(double degrees)
{
    return degrees * PI_VALUE / 180.0;
}

inline double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI_VALUE;
}

inline void calculateDistanceAndAzimuth(int32_t srcLat, int32_t srcLon, int32_t dstLat, int32_t dstLon,
    uint32_t *outDistance, uint32_t *outAzimuth)
{
    const double deltaLon = degreesToRadians(((double)dstLon - srcLon) / 1e7);
    const double thetaA = degreesToRadians((double)srcLat / 1e7);
    const double thetaB = degreesToRadians((double)dstLat / 1e7);
    const double cosThetaA = cos(thetaA);
    const double cosThetaB = cos(thetaB);
    const double sinThetaA = sin(thetaA);
    const double sinThetaB = sin(thetaB);
    const double cosDeltaLon = cos(deltaLon);
    const double sinDeltaLon = sin(deltaLon);

    if (outDistance)
    {
        const double earthRadius = 6371e3;
        const double distance = acos(sinThetaA * sinThetaB + cosThetaA * cosThetaB * cosDeltaLon) * earthRadius;
        *outDistance = (uint32_t)distance;
    }

    if (outAzimuth)
    {
        const double eastComponent = cosThetaB * sinDeltaLon;
        const double northComponent = cosThetaA * sinThetaB - sinThetaA * cosThetaB * cosDeltaLon;
        int32_t heading = (int32_t)radiansToDegrees(atan2(eastComponent, northComponent));
        heading = (heading + 360) % 360;
        *outAzimuth = (uint32_t)heading;
    }
}

inline int32_t calculateElevation(uint32_t distance, int32_t altitude)
{
    return (int32_t)radiansToDegrees(atan2(altitude, distance));
}
}