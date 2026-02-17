#include <periodics/as5600_encoder.hpp>

namespace periodics
{
    CAs5600Encoder::CAs5600Encoder(std::chrono::milliseconds f_period, UnbufferedSerial& f_serial)
        : utils::CTask(f_period)
        , m_sensor() // defaults to I2C3: PC_9 (SDA), D7/PA_8 (SCL), DIR=D2
        , m_tracker(m_sensor, f_period.count() > 0 ? (1000.0f / f_period.count()) : 1.0f)
        , m_serial(f_serial)
        , m_isOk(false)
    {
        m_isOk = m_sensor.init();
    }

    CAs5600Encoder::~CAs5600Encoder()
    {
    }

    void CAs5600Encoder::_run()
    {
        if (!m_isOk) return;
        if (!m_tracker.update()) return;

        uint16_t raw = m_tracker.getLastRaw();
        int32_t total = m_tracker.getTotalTicks();
        int32_t velocity = m_tracker.getVelocityTicksPerSec();
        bool md = m_sensor.magnetDetected();

        uint8_t agc = 0;
        if (!m_sensor.readAgc(agc)) {
            agc = 0;
        }

        char buffer[96];
        int len = snprintf(
            buffer,
            sizeof(buffer),
            "@as5600:%u;%ld;%ld;%u;%u;;\r\n",
            static_cast<unsigned>(raw),
            static_cast<long>(total),
            static_cast<long>(velocity),
            static_cast<unsigned>(md ? 1 : 0),
            static_cast<unsigned>(agc)
        );

        if (len <= 0 || len >= static_cast<int>(sizeof(buffer))) return;
        m_serial.write(buffer, len);
    }
}; // namespace periodics
