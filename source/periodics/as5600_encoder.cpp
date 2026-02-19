#include <periodics/as5600_encoder.hpp>

// Wheel: D=62mm, 32768 ticks/wheel rev. mm/tick = (pi*62)/32768.
static constexpr int32_t kTicksPerWheelRev = 32768;
static constexpr int32_t kWheelDiameterMm = 62;
// (pi*62)*1000 for integer mm/s: speed_mm_s = (velocity * 194778) / 32768 / 1000
static constexpr int64_t kMmPerTickNum = 194778;  // (pi*62)*1000
static constexpr int32_t kMmPerTickDen = 32768;

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
        static float ema_speed = 0.0f;
        constexpr float alpha = 0.2f; // EMA smoothing factor (adjust as needed)

        if (!m_isOk) return;
        if (!m_tracker.update()) return;

        int32_t total = m_tracker.getTotalTicks();
        int32_t velocity = m_tracker.getVelocityTicksPerSec();
        int32_t speed_mm_s = (int32_t)((int64_t)velocity * kMmPerTickNum / kMmPerTickDen / 1000);
        bool md = m_sensor.magnetDetected();

        // Apply EMA filter to computed speed
        ema_speed = alpha * speed_mm_s + (1.0f - alpha) * ema_speed;

        uint8_t agc = 0;
        if (!m_sensor.readAgc(agc)) {
            agc = 0;
        }

        char buffer[96];
        int len = snprintf(
            buffer,
            sizeof(buffer),
            "@as5600:%ld;%ld;%u;%u;%ld;;\r\n",
            static_cast<long>(speed_mm_s),
            static_cast<long>(total),
            static_cast<unsigned>(md ? 1 : 0),
            static_cast<unsigned>(agc),
            static_cast<long>(ema_speed)
        );

        if (len <= 0 || len >= static_cast<int>(sizeof(buffer))) return;
        m_serial.write(buffer, len);
    }
}; // namespace periodics
