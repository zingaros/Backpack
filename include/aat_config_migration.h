#pragma once

#include <stdint.h>

namespace AatConfigMigration
{
template <typename Config>
inline bool migrateVrxVersion(Config &config, uint32_t versionFrom, uint32_t versionTo)
{
    if (config.version != versionFrom)
        return false;

    config.version = versionTo;
    return true;
}

template <typename Config>
inline void resetAatCompassCalibration(Config &config)
{
    config.panCompassCalibration.offsetX = 0;
    config.panCompassCalibration.offsetY = 0;
    config.panCompassCalibration.scaleXPermille = 1000;
    config.panCompassCalibration.scaleYPermille = 1000;
    config.panCompassCalibration.headingOffsetDegrees = 0;
    config.panCompassCalibration.valid = 0;
}

template <typename Config>
inline void resetAatCompassSetup(Config &config)
{
    resetAatCompassCalibration(config);
    config.panCompassNorthAligned = 0;
}

template <typename Config>
inline bool migrateAatVrxV5(Config &config, uint32_t versionV5, uint32_t versionV8,
    uint16_t panServoNeutralUs)
{
    if (!migrateVrxVersion(config, versionV5, versionV8))
        return false;

    config.panServoNeutralUs = panServoNeutralUs;
    resetAatCompassCalibration(config);
    config.panCompassNorthAligned = 0;
    return true;
}

template <typename Config>
inline bool migrateAatVrxV6(Config &config, uint32_t versionV6, uint32_t versionV8)
{
    if (!migrateVrxVersion(config, versionV6, versionV8))
        return false;

    resetAatCompassCalibration(config);
    config.panCompassNorthAligned = 0;
    return true;
}

template <typename Config>
inline bool migrateAatVrxV7(Config &config, uint32_t versionV7, uint32_t versionV8)
{
    if (!migrateVrxVersion(config, versionV7, versionV8))
        return false;

    config.panCompassNorthAligned = 0;
    return true;
}

template <typename Config>
inline bool migrateAatVrxV8(Config &config, uint32_t versionV8, uint32_t versionV9,
    uint8_t defaultPanDeadbandDegrees)
{
    if (!migrateVrxVersion(config, versionV8, versionV9))
        return false;

    config.panDeadbandDegrees = defaultPanDeadbandDegrees;
    return true;
}

template <typename Config>
inline bool migrateAatVrxV9(Config &config, uint32_t versionV9, uint32_t versionV10,
    uint16_t panNeutralUs, uint16_t leftMaxOffsetUs, uint16_t rightMaxOffsetUs)
{
    if (!migrateVrxVersion(config, versionV9, versionV10))
        return false;

    config.panLeftStartPulseUs = panNeutralUs - leftMaxOffsetUs;
    config.panRightStartPulseUs = panNeutralUs + rightMaxOffsetUs;
    config.panLeftMaxOffsetUs = leftMaxOffsetUs;
    config.panRightMaxOffsetUs = rightMaxOffsetUs;
    config.panPulseCalibrationValid = 0;
    return true;
}

template <typename Config>
inline bool migrateVrxV5(Config &config, uint32_t versionV5, uint32_t versionV7)
{
    return migrateVrxVersion(config, versionV5, versionV7);
}

template <typename Config>
inline bool migrateVrxV6(Config &config, uint32_t versionV6, uint32_t versionV7)
{
    return migrateVrxVersion(config, versionV6, versionV7);
}

template <typename Config>
inline bool migrateVrxV7(Config &config, uint32_t versionV7, uint32_t versionV8)
{
    return migrateVrxVersion(config, versionV7, versionV8);
}

template <typename Config>
inline bool migrateVrxV8(Config &config, uint32_t versionV8, uint32_t versionV9)
{
    return migrateVrxVersion(config, versionV8, versionV9);
}

template <typename Config>
inline bool migrateVrxV9(Config &config, uint32_t versionV9, uint32_t versionV10)
{
    return migrateVrxVersion(config, versionV9, versionV10);
}

}