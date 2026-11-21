#pragma once

#include <Wire.h>
#include "aat_compass.h"

class AatHmc5883l
{
public:
    struct ConfigurationReadback
    {
        uint8_t configAAfterWrite;
        bool afterWriteValid;
        uint8_t configAAfterConfigBWrite;
        bool afterConfigBWriteValid;
        uint8_t configAAfterTrigger;
        bool afterTriggerValid;
    };

    struct RawSample
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t status;
        uint8_t statusAfterRead;
        bool statusAfterReadValid;
        uint8_t registers[6];
    };

    struct SampleConfigReadback
    {
        uint8_t configAAtDataReady = 0;
        bool configAAtDataReadyValid = false;
    };

    explicit AatHmc5883l(TwoWire &wire) : _wire(wire) {}

    bool readIdentity(uint8_t identity[3])
    {
        return readRegisters(0x0A, identity, 3);
    }

    bool hasHmcIdentity(const uint8_t identity[3]) const
    {
        return identity[0] == 'H' && identity[1] == '4' && identity[2] == '3';
    }

    bool configure(uint8_t configA = 0x70)
    {
        return writeRegister(0x00, configA) &&
            writeRegister(0x01, 0x20) &&
            writeRegister(0x02, 0x00);
    }

    bool configureSingleShot(uint8_t configA = 0x68,
        ConfigurationReadback *readback = nullptr)
    {
        if (readback != nullptr)
            *readback = {};
        if (!writeRegister(0x00, configA))
            return false;
        if (readback != nullptr)
            readback->afterWriteValid = readRegisters(0x00, &readback->configAAfterWrite, 1);
        if (!writeRegister(0x01, 0x20))
            return false;
        if (readback != nullptr)
            readback->afterConfigBWriteValid = readRegisters(
                0x00, &readback->configAAfterConfigBWrite, 1);
        if (!triggerSingleShot())
            return false;
        if (readback != nullptr)
            readback->afterTriggerValid = readRegisters(0x00, &readback->configAAfterTrigger, 1);
        return true;
    }

    bool triggerSingleShot()
    {
        return writeRegister(0x02, 0x01);
    }

    bool readStatus(uint8_t *status)
    {
        return status != nullptr && readRegisters(0x09, status, 1);
    }

    bool readRaw(RawSample *sample, uint8_t *statusOut = nullptr,
        SampleConfigReadback *configReadback = nullptr)
    {
        uint8_t status = 0;
        if (sample == nullptr || !readStatus(&status))
            return false;
        sample->status = status;
        if (statusOut != nullptr)
            *statusOut = status;
        if ((status & 0x01) == 0)
            return false;

        if (configReadback != nullptr)
            configReadback->configAAtDataReadyValid = readRegisters(
                0x00, &configReadback->configAAtDataReady, 1);

        uint8_t data[6] = {};
        if (!readRegisters(0x03, data, sizeof(data)))
            return false;

        for (uint8_t index = 0; index < sizeof(data); ++index)
            sample->registers[index] = data[index];
        sample->x = decodeAxis(data[0], data[1]);
        sample->z = decodeAxis(data[2], data[3]);
        sample->y = decodeAxis(data[4], data[5]);

        uint8_t finalStatus = status;
        sample->statusAfterReadValid = readStatus(&finalStatus);
        if (sample->statusAfterReadValid)
            sample->statusAfterRead = finalStatus;
        else
            sample->statusAfterRead = status;
        return sample->statusAfterReadValid;
    }

    bool readConfiguration(uint8_t settings[3])
    {
        return settings != nullptr && readRegisters(0x00, settings, 3);
    }

private:
    static constexpr uint8_t ADDRESS = 0x1E;

    static int16_t decodeAxis(uint8_t high, uint8_t low)
    {
        return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
    }

    bool writeRegister(uint8_t reg, uint8_t value)
    {
        _wire.beginTransmission(ADDRESS);
        _wire.write(reg);
        _wire.write(value);
        const uint32_t startedAtUs = micros();
        const uint8_t error = _wire.endTransmission();
    #if defined(AAT_I2C_TRACE)
        Serial.printf("I2C t=%lu op=write addr=1e reg=%02x value=%02x err=%u us=%lu\n",
            static_cast<unsigned long>(startedAtUs), reg, value, error,
            static_cast<unsigned long>(micros() - startedAtUs));
    #endif
        return error == 0;
    }

    bool readRegisters(uint8_t reg, uint8_t *data, uint8_t length)
    {
        _wire.beginTransmission(ADDRESS);
        _wire.write(reg);
        const uint32_t startedAtUs = micros();
    #if defined(AAT_I2C_TRACE_STOP_READ)
        const uint8_t error = _wire.endTransmission(true);
    #else
        const uint8_t error = _wire.endTransmission(false);
    #endif
        const uint8_t received = error == 0 ? _wire.requestFrom(ADDRESS, length) : 0;
    #if defined(AAT_I2C_TRACE)
        Serial.printf("I2C t=%lu op=read addr=1e reg=%02x err=%u want=%u got=%u us=%lu",
            static_cast<unsigned long>(startedAtUs), reg, error, length, received,
            static_cast<unsigned long>(micros() - startedAtUs));
    #endif
        if (error != 0 || received != length)
        {
    #if defined(AAT_I2C_TRACE)
            Serial.println();
    #endif
            return false;
        }

        for (uint8_t index = 0; index < length; ++index)
            data[index] = _wire.read();
    #if defined(AAT_I2C_TRACE)
        Serial.print(" data=");
        for (uint8_t index = 0; index < length; ++index)
            Serial.printf("%02x", data[index]);
        Serial.println();
    #endif
        return true;
    }

    TwoWire &_wire;
};