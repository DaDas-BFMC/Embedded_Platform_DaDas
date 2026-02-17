#ifndef AS5600_HPP
#define AS5600_HPP

#include <mbed.h>
#include <cstdint>

namespace drivers
{
    class CAs5600
    {
        public:
            /* Constructor */
            CAs5600(PinName sda = PC_9, PinName scl = D7, int frequency_hz = 100000, PinName dir_pin = D2);
            /* Destructor */
            ~CAs5600();

            /* Configure sensor according to datasheet requirements */
            bool init();

            /* Data acquisition */
            bool readRawAngle(uint16_t& raw_angle);

            /* Diagnostics */
            bool readStatus(uint8_t& status);
            bool readAgc(uint8_t& agc);
            bool magnetDetected();
            bool agcInRange(uint8_t min_val = 0, uint8_t max_val = 128);

        private:
            static constexpr uint8_t kI2cAddress = 0x36;
            static constexpr uint8_t kRegConfHigh = 0x07;
            static constexpr uint8_t kRegConfLow = 0x08;
            static constexpr uint8_t kRegStatus = 0x0B;
            static constexpr uint8_t kRegRawAngle = 0x0C;
            static constexpr uint8_t kRegAgc = 0x1A;

            static constexpr uint8_t kStatusMagnetDetected = 0x20;

            static constexpr uint8_t kSlowFilterFastest = 0x03;   // SF = 11
            static constexpr uint8_t kFastFilterTh6 = 0x01;       // FTH = 001
            static constexpr uint8_t kWatchdogOff = 0x00;         // WD = 0
            static constexpr uint8_t kHysteresisOff = 0x00;       // HYST = 00
            static constexpr uint8_t kPowerModeNominal = 0x00;    // PM = 00

            bool writeRegister(uint8_t reg, uint8_t value);
            bool readRegister(uint8_t reg, uint8_t& value);
            bool readRegister16(uint8_t reg, uint16_t& value);

            I2C m_i2c;
            int m_addr_8bit;
            DigitalOut m_dir_pin;
    };

    class CAs5600Tracker
    {
        public:
            CAs5600Tracker(CAs5600& sensor, float sample_hz);

            bool update();
            int32_t getTotalTicks() const;
            int32_t getVelocityTicksPerSec() const;
            uint16_t getLastRaw() const;

            void reset(int32_t total_ticks = 0);
            void setSampleHz(float sample_hz);

        private:
            CAs5600& m_sensor;
            float m_sample_hz;
            int32_t m_total_ticks;
            int32_t m_last_delta;
            uint16_t m_last_raw;
            bool m_has_last;
    };
}; // namespace drivers

#endif // AS5600_HPP
