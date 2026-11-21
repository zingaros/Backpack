#include "config.h"
#include "aat_config_defaults.h"
#include "aat_config_migration.h"
#include "aat_config_validation.h"
#include "logging.h"

#if defined(TARGET_TX_BACKPACK)

void
TxBackpackConfig::Load()
{
    m_eeprom->Get(0, m_config);
    m_modified = 0;

    // Check if version number matches
    if (m_config.version != (uint32_t)(TX_BACKPACK_CONFIG_VERSION | TX_BACKPACK_CONFIG_MAGIC))
    {
        // If not, revert to defaults for this version
        DBGLN("EEPROM version mismatch! Resetting to defaults...");
        SetDefaults();
    }
}

void
TxBackpackConfig::Commit()
{
    if (!m_modified)
    {
        // No changes
        return;
    }
    // Write the struct to eeprom
    m_eeprom->Put(0, m_config);
    m_eeprom->Commit();

    m_modified = false;
}

// Setters
void
TxBackpackConfig::SetStorageProvider(ELRS_EEPROM *eeprom)
{
    if (eeprom)
    {
        m_eeprom = eeprom;
    }
}

void
TxBackpackConfig::SetDefaults()
{
    m_config.version = TX_BACKPACK_CONFIG_VERSION | TX_BACKPACK_CONFIG_MAGIC;
    m_config.startWiFi = false;
    m_config.ssid[0] = 0;
    m_config.password[0] = 0;
    memset(m_config.address, 0, 6);
    m_config.wifiService = WIFI_SERVICE_UPDATE;
    m_config.mavlinkListenPort = 14555;  // Default MavLink listen port
    m_config.mavlinkSendPort = 14550;    // Default MavLink send port
    m_modified = true;
    Commit();
}

void
TxBackpackConfig::SetStartWiFiOnBoot(bool startWifi)
{
    m_config.startWiFi = startWifi;
    m_modified = true;
}

void
TxBackpackConfig::SetSSID(const char *ssid)
{
    strcpy(m_config.ssid, ssid);
    m_modified = true;
}

void
TxBackpackConfig::SetPassword(const char *password)
{
    strcpy(m_config.password, password);
    m_modified = true;
}

void
TxBackpackConfig::SetGroupAddress(const uint8_t address[6])
{
    memcpy(m_config.address, address, 6);
    m_modified = true;
}

void
TxBackpackConfig::SetWiFiService(wifi_service_t service)
{
    m_config.wifiService = service;
    m_modified = true;
}

void
TxBackpackConfig::SetTelemMode(telem_mode_t mode)
{
    m_config.telemMode = mode;
    m_modified = true;
}
void
TxBackpackConfig::SetMavlinkListenPort(uint16_t port)
{
    m_config.mavlinkListenPort = port;
    m_modified = true;
}

void
TxBackpackConfig::SetMavlinkSendPort(uint16_t port)
{
    m_config.mavlinkSendPort = port;
    m_modified = true;
}
#endif

/////////////////////////////////////////////////////

#if defined(TARGET_VRX_BACKPACK)

#define CONFIG_MOD_CHECK(fld, val) {    \
        if (val == fld)                 \
            return;                     \
        fld = val;                      \
        m_modified = true;              \
    }

void
VrxBackpackConfig::Load()
{
    // Populate the struct from eeprom
    m_eeprom->Get(0, m_config);
    m_modified = false;

    const uint32_t versionV5 = VRX_BACKPACK_CONFIG_VERSION_V5 | VRX_BACKPACK_CONFIG_MAGIC;
    const uint32_t versionV6 = VRX_BACKPACK_CONFIG_VERSION_V6 | VRX_BACKPACK_CONFIG_MAGIC;
    const uint32_t versionV7 = VRX_BACKPACK_CONFIG_VERSION_V7 | VRX_BACKPACK_CONFIG_MAGIC;
    const uint32_t versionV8 = VRX_BACKPACK_CONFIG_VERSION_V8 | VRX_BACKPACK_CONFIG_MAGIC;
    const uint32_t versionV9 = VRX_BACKPACK_CONFIG_VERSION_V9 | VRX_BACKPACK_CONFIG_MAGIC;
#if defined(AAT_BACKPACK)
    const uint32_t versionV10 = VRX_BACKPACK_CONFIG_VERSION_V10 | VRX_BACKPACK_CONFIG_MAGIC;
#endif
    const uint32_t versionCurrent = VRX_BACKPACK_CONFIG_VERSION | VRX_BACKPACK_CONFIG_MAGIC;
    bool migrated = false;
#if defined(AAT_BACKPACK)
    migrated = AatConfigMigration::migrateAatVrxV5(m_config, versionV5, versionV8,
        AatConfigDefaults::PAN_SERVO_NEUTRAL_US);
    if (!migrated)
        migrated = AatConfigMigration::migrateAatVrxV6(m_config, versionV6, versionV8);
    if (!migrated)
        migrated = AatConfigMigration::migrateAatVrxV7(m_config, versionV7, versionV8);
#else
    migrated = AatConfigMigration::migrateVrxV5(m_config, versionV5, versionV8);
    if (!migrated)
        migrated = AatConfigMigration::migrateVrxV6(m_config, versionV6, versionV8);
    if (!migrated)
        migrated = AatConfigMigration::migrateVrxV7(m_config, versionV7, versionV8);
#endif
    const bool migratedFromV8 =
#if defined(AAT_BACKPACK)
        AatConfigMigration::migrateAatVrxV8(m_config, versionV8, versionV9,
            AatConfigDefaults::PAN_DEADBAND_DEFAULT_DEGREES);
#else
        AatConfigMigration::migrateVrxV8(m_config, versionV8, versionV9);
#endif
    migrated = migrated || migratedFromV8;
    const bool migratedFromV9 =
#if defined(AAT_BACKPACK)
        AatConfigMigration::migrateAatVrxV9(m_config, versionV9, versionV10,
            m_config.panServoNeutralUs,
            AatConfigDefaults::PAN_LEFT_MAX_OFFSET_US,
            AatConfigDefaults::PAN_RIGHT_MAX_OFFSET_US);
#else
        AatConfigMigration::migrateVrxV9(m_config, versionV9, versionCurrent);
#endif
    migrated = migrated || migratedFromV9;
#if defined(AAT_BACKPACK)
    if (AatConfigMigration::migrateVrxVersion(m_config, versionV10, versionCurrent))
    {
        m_config.panCompassMounting = AatPanCompassMounting::Standard;
        migrated = true;
    }
#endif
    if (migrated)
    {
        DBGLN("Migrating VRX config to current version...");
        m_modified = true;
        Commit();
    }
    else if (m_config.version != versionCurrent)
    {
        // If not, revert to defaults for this version
        DBGLN("EEPROM version mismatch! Resetting to defaults...");
        SetDefaults();
    }
#if defined(AAT_BACKPACK)
    if (!AatConfigValidation::isPanPulseEnvelopeValid(m_config.panServoNeutralUs,
        m_config.panLeftMaxOffsetUs, m_config.panRightMaxOffsetUs))
    {
        DBGLN("Invalid continuous-pan pulse envelope; restoring neutral and default offsets");
        SetAatPanPulseEnvelope(AatConfigDefaults::PAN_SERVO_NEUTRAL_US,
            AatConfigDefaults::PAN_LEFT_MAX_OFFSET_US,
            AatConfigDefaults::PAN_RIGHT_MAX_OFFSET_US);
        Commit();
    }
#endif
}

void
VrxBackpackConfig::Commit()
{
    if (!m_modified)
    {
        // No changes
        return;
    }

    // Write the struct to eeprom
    m_eeprom->Put(0, m_config);
    m_eeprom->Commit();

    m_modified = false;
}

// Setters
void
VrxBackpackConfig::SetStorageProvider(ELRS_EEPROM *eeprom)
{
    if (eeprom)
    {
        m_eeprom = eeprom;
    }
}

void
VrxBackpackConfig::SetDefaults()
{
    memset(&m_config, 0, sizeof(m_config));
    m_config.version = VRX_BACKPACK_CONFIG_VERSION | VRX_BACKPACK_CONFIG_MAGIC;

#if defined(AAT_BACKPACK)
    AatConfigDefaults::apply(m_config.aat, m_config.vbat);
    m_config.panServoNeutralUs = AatConfigDefaults::PAN_SERVO_NEUTRAL_US;
    AatConfigMigration::resetAatCompassCalibration(m_config);
    m_config.panCompassNorthAligned = 0;
    m_config.panDeadbandDegrees = AatConfigDefaults::PAN_DEADBAND_DEFAULT_DEGREES;
    m_config.panLeftMaxOffsetUs = AatConfigDefaults::PAN_LEFT_MAX_OFFSET_US;
    m_config.panRightMaxOffsetUs = AatConfigDefaults::PAN_RIGHT_MAX_OFFSET_US;
    m_config.panLeftStartPulseUs =
        m_config.panServoNeutralUs - m_config.panLeftMaxOffsetUs;
    m_config.panRightStartPulseUs =
        m_config.panServoNeutralUs + m_config.panRightMaxOffsetUs;
    m_config.panPulseCalibrationValid = 0;
#endif

    m_modified = true;
    Commit();
}

void
VrxBackpackConfig::SetSSID(const char *ssid)
{
    strcpy(m_config.ssid, ssid);
    m_modified = true;
}

void
VrxBackpackConfig::SetPassword(const char *password)
{
    strcpy(m_config.password, password);
    m_modified = true;
}

void
VrxBackpackConfig::SetGroupAddress(const uint8_t address[6])
{
    memcpy(m_config.address, address, 6);
    m_modified = true;
}

void
VrxBackpackConfig::SetBootCount(uint8_t count)
{
    m_config.bootCount = count;
    m_modified = true;
}

void
VrxBackpackConfig::SetStartWiFiOnBoot(bool startWifi)
{
    m_config.startWiFi = startWifi;
    m_modified = true;
}

#if defined(HAS_HEADTRACKING)
void
VrxBackpackConfig::SetCompassCalibration(const int calibrationData[3][2])
{
    memcpy(m_config.compassCalibration, calibrationData, sizeof(m_config.compassCalibration));
    m_modified = true;
}

void
VrxBackpackConfig::SetIMUCalibration(const float calibration[3])
{
    memcpy(m_config.imuCalibration, calibration, sizeof(m_config.imuCalibration));
    m_modified = true;
}

void
VrxBackpackConfig::SetBoardOrientation(const float orientation[3])
{
    memcpy(m_config.boardOrientation, orientation, sizeof(m_config.boardOrientation));
    m_modified = true;
}
#endif

#if defined(AAT_BACKPACK)

void
VrxBackpackConfig::SetAatServoSmooth(uint8_t val)
{
    CONFIG_MOD_CHECK(m_config.aat.servoSmooth, val);
}

void
VrxBackpackConfig::SetAatAzimuthServoFastFlip(uint8_t val)
{
    CONFIG_MOD_CHECK(m_config.aat.azimuthServoFastFlip, val);
}

void
VrxBackpackConfig::SetAatServoLow(uint8_t idx, uint16_t val)
{
    CONFIG_MOD_CHECK(m_config.aat.servoEndpoints[idx].low, val);
}

void
VrxBackpackConfig::SetAatServoHigh(uint8_t idx, uint16_t val)
{
    CONFIG_MOD_CHECK(m_config.aat.servoEndpoints[idx].high, val);
}

void
VrxBackpackConfig::SetVbatScale(uint16_t val)
{
    CONFIG_MOD_CHECK(m_config.vbat.scale, val);
}

void
VrxBackpackConfig::SetVbatOffset(int16_t val)
{
    CONFIG_MOD_CHECK(m_config.vbat.offset, val);
}

void
VrxBackpackConfig::SetAatCenterDir(uint8_t val)
{
    CONFIG_MOD_CHECK(m_config.aat.centerDir, val);
}

void
VrxBackpackConfig::SetAatServoMode(uint8_t val)
{
    CONFIG_MOD_CHECK(m_config.aat.servoMode, val);
}

void
VrxBackpackConfig::SetAatSatelliteHomeMin(uint8_t val)
{
    CONFIG_MOD_CHECK(m_config.aat.satelliteHomeMin, val);
}

bool
VrxBackpackConfig::SetAatPanServoNeutral(uint16_t val)
{
    return SetAatPanPulseEnvelope(val,
        m_config.panLeftMaxOffsetUs, m_config.panRightMaxOffsetUs);
}

bool
VrxBackpackConfig::SetAatPanDeadbandDegrees(uint8_t val)
{
    if (!AatConfigValidation::isPanDeadbandValid(val))
        return false;
    if (m_config.panDeadbandDegrees != val)
    {
        m_config.panDeadbandDegrees = val;
        m_modified = true;
    }
    return true;
}

bool
VrxBackpackConfig::GetAatPanPulseCalibrationValid() const
{
    return m_config.panPulseCalibrationValid == 1
        && AatConfigValidation::isPanPulseCalibrationValid(
            m_config.panServoNeutralUs, m_config.panLeftStartPulseUs,
            m_config.panRightStartPulseUs, m_config.panLeftMaxOffsetUs,
            m_config.panRightMaxOffsetUs);
}

bool
VrxBackpackConfig::SetAatPanPulseCalibration(
    uint16_t leftStartPulseUs, uint16_t rightStartPulseUs)
{
    if (!AatConfigValidation::isPanPulseCalibrationValid(
        m_config.panServoNeutralUs, leftStartPulseUs, rightStartPulseUs,
        m_config.panLeftMaxOffsetUs, m_config.panRightMaxOffsetUs))
        return false;

    if (m_config.panLeftStartPulseUs != leftStartPulseUs
        || m_config.panRightStartPulseUs != rightStartPulseUs
        || m_config.panPulseCalibrationValid != 1)
    {
        m_config.panLeftStartPulseUs = leftStartPulseUs;
        m_config.panRightStartPulseUs = rightStartPulseUs;
        m_config.panPulseCalibrationValid = 1;
        m_modified = true;
    }
    return true;
}

bool
VrxBackpackConfig::SetAatPanMaximumOffsets(
    uint16_t leftMaxOffsetUs, uint16_t rightMaxOffsetUs)
{
    return SetAatPanPulseEnvelope(m_config.panServoNeutralUs,
        leftMaxOffsetUs, rightMaxOffsetUs);
}

bool
VrxBackpackConfig::SetAatPanPulseEnvelope(uint16_t neutralUs,
    uint16_t leftMaxOffsetUs, uint16_t rightMaxOffsetUs)
{
    if (!AatConfigValidation::isPanPulseEnvelopeValid(
        neutralUs, leftMaxOffsetUs, rightMaxOffsetUs))
        return false;

    if (m_config.panServoNeutralUs != neutralUs
        || m_config.panLeftMaxOffsetUs != leftMaxOffsetUs
        || m_config.panRightMaxOffsetUs != rightMaxOffsetUs)
    {
        m_config.panServoNeutralUs = neutralUs;
        m_config.panLeftMaxOffsetUs = leftMaxOffsetUs;
        m_config.panRightMaxOffsetUs = rightMaxOffsetUs;
        m_config.panLeftStartPulseUs = m_config.panServoNeutralUs - leftMaxOffsetUs;
        m_config.panRightStartPulseUs = m_config.panServoNeutralUs + rightMaxOffsetUs;
        m_config.panPulseCalibrationValid = 0;
        m_modified = true;
    }
    return true;
}

bool
VrxBackpackConfig::SetAatPanCompassCalibration(const aat_pan_compass_calibration_t &calibration)
{
    if (!AatConfigValidation::isPanCompassCalibrationValid(calibration))
        return false;
    if (memcmp(&m_config.panCompassCalibration, &calibration, sizeof(calibration)) != 0)
    {
        m_config.panCompassCalibration = calibration;
        m_config.panCompassNorthAligned = 0;
        m_modified = true;
    }
    return true;
}

bool
VrxBackpackConfig::SetAatPanCompassMountingReversed()
{
    if (IsAatPanCompassReversed() || m_config.panCompassCalibration.valid != 1)
        return false;
    m_config.panCompassNorthAligned = 0;
    m_config.panCompassCalibration.headingOffsetDegrees = 0;
    m_config.panPulseCalibrationValid = 0;
    m_config.panCompassMounting = AatPanCompassMounting::ReversedUnverified;
    m_modified = true;
    return true;
}

bool
VrxBackpackConfig::VerifyAatPanCompassMounting()
{
    if (!IsAatPanCompassMountingPending() || m_config.panCompassCalibration.valid != 1)
        return false;
    m_config.panCompassMounting = AatPanCompassMounting::ReversedVerified;
    m_modified = true;
    return true;
}

bool
VrxBackpackConfig::SetAatPanCompassNorthAligned(bool aligned)
{
    if (aligned && (IsAatPanCompassMountingPending() ||
        !AatConfigValidation::isPanCompassNorthAlignmentValid(m_config.panCompassCalibration.valid == 1)))
        return false;
    const uint8_t value = aligned ? 1 : 0;
    if (m_config.panCompassNorthAligned != value)
    {
        m_config.panCompassNorthAligned = value;
        m_modified = true;
    }
    return true;
}

void
VrxBackpackConfig::ResetAatPanCompassSetup()
{
    AatConfigMigration::resetAatCompassSetup(m_config);
    if (IsAatPanCompassReversed())
        m_config.panPulseCalibrationValid = 0;
    m_config.panCompassMounting = AatPanCompassMounting::Standard;
    m_modified = true;
}

/**
 * @brief: Validate that the endpoints have a valid range, i.e. low/high not the same
*/
bool
VrxBackpackConfig::GetAatServoEndpointsValid() const
{
    return AatConfigValidation::areServoEndpointsValid(
        m_config.aat.servoEndpoints[0].low,
        m_config.aat.servoEndpoints[0].high,
        m_config.aat.servoEndpoints[1].low,
        m_config.aat.servoEndpoints[1].high);
}

#endif /* defined(AAT_BACKPACK) */

#endif /* TARGET_VRX_BACKPACK */

/////////////////////////////////////////////////////

#if defined(TARGET_TIMER_BACKPACK)

void
TimerBackpackConfig::Load()
{
    m_eeprom->Get(0, m_config);
    m_modified = 0;

    // Check if version number matches
    if (m_config.version != (uint32_t)(TIMER_BACKPACK_CONFIG_VERSION | TIMER_BACKPACK_CONFIG_MAGIC))
    {
        // If not, revert to defaults for this version
        DBGLN("EEPROM version mismatch! Resetting to defaults...");
        SetDefaults();
    }

    m_modified = false;
}

void
TimerBackpackConfig::Commit()
{
    if (!m_modified)
    {
        // No changes
        DBGLN("No Config Changes Detected")
        return;
    }
    // Write the struct to eeprom
    m_eeprom->Put(0, m_config);
    m_eeprom->Commit();

    m_modified = false;
}

// Setters
void
TimerBackpackConfig::SetStorageProvider(ELRS_EEPROM *eeprom)
{
    if (eeprom)
    {
        m_eeprom = eeprom;
    }
}

void
TimerBackpackConfig::SetDefaults()
{
    m_config.version = TIMER_BACKPACK_CONFIG_VERSION | TIMER_BACKPACK_CONFIG_MAGIC;
    m_config.bootCount = 0;
    m_config.startWiFi = false;
    m_config.ssid[0] = 0;
    m_config.password[0] = 0;
    memset(m_config.address, 0, 6);
    m_modified = true;
    Commit();
}

void
TimerBackpackConfig::SetStartWiFiOnBoot(bool startWifi)
{
    m_config.startWiFi = startWifi;
    m_modified = true;
}

void
TimerBackpackConfig::SetSSID(const char *ssid)
{
    strcpy(m_config.ssid, ssid);
    m_modified = true;
}

void
TimerBackpackConfig::SetPassword(const char *password)
{
    strcpy(m_config.password, password);
    m_modified = true;
}

void
TimerBackpackConfig::SetGroupAddress(const uint8_t address[6])
{
    memcpy(m_config.address, address, 6);
    m_modified = true;
}

void
TimerBackpackConfig::SetBootCount(uint8_t count)
{
    m_config.bootCount = count;
    m_modified = true;
}
#endif
