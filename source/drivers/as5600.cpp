#include "drivers/as5600.hpp"

namespace drivers
{
    CAs5600::CAs5600(PinName sda, PinName scl, int frequency_hz, PinName dir_pin)
        : m_i2c(sda, scl)
        , m_addr_8bit(kI2cAddress << 1)
        , m_dir_pin(dir_pin, 0)
    {
        m_i2c.frequency(frequency_hz);
    }

    CAs5600::~CAs5600()
    {
    }

    bool CAs5600::init()
    {
        // CONF high byte (0x07): SF[1:0], FTH[4:2], WD[5]
        uint8_t conf_high = (kSlowFilterFastest & 0x03)
            | ((kFastFilterTh6 & 0x07) << 2)
            | ((kWatchdogOff & 0x01) << 5);

        // CONF low byte (0x08): PM[1:0], HYST[3:2], OUTMODE[5:4], PWMF[7:6]
        uint8_t conf_low = (kPowerModeNominal & 0x03)
            | ((kHysteresisOff & 0x03) << 2);

        return writeRegister(kRegConfHigh, conf_high) && writeRegister(kRegConfLow, conf_low);
    }

    bool CAs5600::readRawAngle(uint16_t& raw_angle)
    {
        uint16_t value = 0;
        if (!readRegister16(kRegRawAngle, value)) {
            return false;
        }
        raw_angle = value & 0x0FFF;
        return true;
    }

    bool CAs5600::readStatus(uint8_t& status)
    {
        return readRegister(kRegStatus, status);
    }

    bool CAs5600::readAgc(uint8_t& agc)
    {
        return readRegister(kRegAgc, agc);
    }

    bool CAs5600::magnetDetected()
    {
        uint8_t status = 0;
        if (!readStatus(status)) {
            return false;
        }
        return (status & kStatusMagnetDetected) != 0;
    }

    bool CAs5600::agcInRange(uint8_t min_val, uint8_t max_val)
    {
        uint8_t agc = 0;
        if (!readAgc(agc)) {
            return false;
        }
        return (agc >= min_val) && (agc <= max_val);
    }

    bool CAs5600::writeRegister(uint8_t reg, uint8_t value)
    {
        char data[2] = { static_cast<char>(reg), static_cast<char>(value) };
        return m_i2c.write(m_addr_8bit, data, sizeof(data)) == 0;
    }

    bool CAs5600::readRegister(uint8_t reg, uint8_t& value)
    {
        char reg_buf = static_cast<char>(reg);
        if (m_i2c.write(m_addr_8bit, &reg_buf, 1) != 0) {
            return false;
        }
        char out = 0;
        if (m_i2c.read(m_addr_8bit, &out, 1) != 0) {
            return false;
        }
        value = static_cast<uint8_t>(out);
        return true;
    }

    bool CAs5600::readRegister16(uint8_t reg, uint16_t& value)
    {
        char reg_buf = static_cast<char>(reg);
        if (m_i2c.write(m_addr_8bit, &reg_buf, 1) != 0) {
            return false;
        }
        char out[2] = { 0, 0 };
        if (m_i2c.read(m_addr_8bit, out, 2) != 0) {
            return false;
        }
        value = (static_cast<uint16_t>(static_cast<uint8_t>(out[0])) << 8)
              | static_cast<uint16_t>(static_cast<uint8_t>(out[1]));
        return true;
    }

    CAs5600Tracker::CAs5600Tracker(CAs5600& sensor, float sample_hz)
        : m_sensor(sensor)
        , m_sample_hz(sample_hz)
        , m_total_ticks(0)
        , m_last_delta(0)
        , m_last_raw(0)
        , m_has_last(false)
    {
    }

    bool CAs5600Tracker::update()
    {
        uint16_t raw = 0;
        if (!m_sensor.readRawAngle(raw)) {
            return false;
        }

        if (!m_has_last) {
            m_last_raw = raw;
            m_has_last = true;
            m_last_delta = 0;
            return true;
        }

        int32_t delta = static_cast<int32_t>(raw) - static_cast<int32_t>(m_last_raw);
        if (delta > 2048) {
            delta -= 4096;
        } else if (delta < -2048) {
            delta += 4096;
        }

        m_total_ticks += delta;
        m_last_delta = delta;
        m_last_raw = raw;
        return true;
    }

    int32_t CAs5600Tracker::getTotalTicks() const
    {
        return m_total_ticks;
    }

    int32_t CAs5600Tracker::getVelocityTicksPerSec() const
    {
        return static_cast<int32_t>(m_last_delta * m_sample_hz);
    }

    uint16_t CAs5600Tracker::getLastRaw() const
    {
        return m_last_raw;
    }

    void CAs5600Tracker::reset(int32_t total_ticks)
    {
        m_total_ticks = total_ticks;
        m_last_delta = 0;
        m_has_last = false;
    }

    void CAs5600Tracker::setSampleHz(float sample_hz)
    {
        m_sample_hz = sample_hz;
    }
}; // namespace drivers