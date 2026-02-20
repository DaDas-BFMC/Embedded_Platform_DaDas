#include <periodics/imu_encoder.hpp>
#include <brain/globalsv.hpp>
#include <cstdio>
#include <cstdlib>

// Wheel: D=62mm, 32768 ticks/wheel rev. mm/tick = (pi*62)/32768.
static constexpr int32_t kMmPerTickDen = 32768;
static constexpr int64_t kMmPerTickNum = 194778;  // (pi*62)*1000

#define _FMT_SIGN(v, div)  (((v) < 0 && (v) / (div) == 0) ? "-" : "")

namespace periodics
{
    float CImuEncoder::s_speedCalibScale = 1.0f;

    void CImuEncoder::serialCallbackSpeedCalib(char const* message, char* response)
    {
        float scale = 1.0f;
        int n = std::sscanf(message, "%f", &scale);
        if (n != 1 || scale < 0.5f || scale > 2.0f)
        {
            std::sprintf(response, "syntax error");
            return;
        }
        s_speedCalibScale = scale;
        std::sprintf(response, "%.4f", scale);
    }

    void CImuEncoder::serialCallbackCalibOutput(char const* message, char* response)
    {
        unsigned val = 0;
        int n = std::sscanf(message, "%u", &val);
        if (n != 1)
        {
            std::sprintf(response, "syntax error");
            return;
        }
        bool_globalsV_calibOutput = (val != 0);
        std::sprintf(response, "%d", bool_globalsV_calibOutput ? 1 : 0);
    }

    CImuEncoder::CImuEncoder(
        std::chrono::milliseconds f_period,
        UnbufferedSerial& f_serial,
        periodics::CImu& f_imu,
        PinName encoder_sda,
        PinName encoder_scl,
        PinName encoder_dir
    )
        : utils::CTask(f_period)
        , m_serial(f_serial)
        , m_imu(f_imu)
        , m_encoderSensor(encoder_sda, encoder_scl, 100000, encoder_dir)
        , m_encoderTracker(m_encoderSensor, f_period.count() > 0 ? (1000.0f / f_period.count()) : 1.0f)
        , m_encoderOk(false)
        , m_emaSpeed(0.0f)
    {
        m_encoderOk = m_encoderSensor.init();
    }

    CImuEncoder::~CImuEncoder()
    {
    }

    void CImuEncoder::_run()
    {
        if (!bool_globalsV_imu_isActive)
            return;

        int32_t totalTick = 0;
        int32_t speed_mm_s = 0;
        unsigned md = 0;
        unsigned agc = 0;

        if (m_encoderOk && m_encoderTracker.update()) {
            totalTick = m_encoderTracker.getTotalTicks();
            int32_t velocity = m_encoderTracker.getVelocityTicksPerSec();
            speed_mm_s = (int32_t)((int64_t)velocity * kMmPerTickNum / kMmPerTickDen / 1000);
            speed_mm_s = (int32_t)(static_cast<float>(speed_mm_s) * s_speedCalibScale);
            m_emaSpeed = kEmaAlpha * static_cast<float>(speed_mm_s) + (1.0f - kEmaAlpha) * m_emaSpeed;
            md = m_encoderSensor.magnetDetected() ? 1 : 0;
            uint8_t agcVal = 0;
            if (!m_encoderSensor.readAgc(agcVal))
                agcVal = 0;
            agc = agcVal;
        }

        int32_t speed_ema_int = (int32_t)m_emaSpeed;

        if (bool_globalsV_calibOutput) {
            /* Calibration test: send only encoder data to reduce serial load and latency. */
            char buffer[64];
            int len = snprintf(buffer, sizeof(buffer), "@enc:%ld;%ld;%u;%u;;\r\n",
                (long)speed_ema_int, (long)totalTick, md, agc);
            if (len > 0 && len < static_cast<int>(sizeof(buffer)))
                m_serial.write(buffer, len);
            return;
        }

        periodics::ImuSnapshot snap;
        if (!m_imu.readAndFillSnapshot(snap))
            return;

        char buffer[256];
        int len = snprintf(buffer, sizeof(buffer),
            "@imu:%s%d.%01d;%s%d.%01d;%s%d.%01d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%03d;%s%d.%02d;%s%d.%02d;%s%d.%02d;%ld;%ld;%u;%u;;\r\n",
            _FMT_SIGN(snap.euler_r_deg, 10),  snap.euler_r_deg/10, abs(snap.euler_r_deg%10),
            _FMT_SIGN(snap.euler_p_deg, 10),  snap.euler_p_deg/10, abs(snap.euler_p_deg%10),
            _FMT_SIGN(snap.euler_h_deg, 10),  snap.euler_h_deg/10, abs(snap.euler_h_deg%10),
            _FMT_SIGN(snap.velocityX, 1000),     snap.velocityX/1000, abs(snap.velocityX%1000),
            _FMT_SIGN(snap.velocityY, 1000),     snap.velocityY/1000, abs(snap.velocityY%1000),
            _FMT_SIGN(snap.velocityZ, 1000),     snap.velocityZ/1000, abs(snap.velocityZ%1000),
            _FMT_SIGN(snap.linear_accel_x_msq, 1000), snap.linear_accel_x_msq/1000, abs(snap.linear_accel_x_msq%1000),
            _FMT_SIGN(snap.linear_accel_y_msq, 1000), snap.linear_accel_y_msq/1000, abs(snap.linear_accel_y_msq%1000),
            _FMT_SIGN(snap.linear_accel_z_msq, 1000), snap.linear_accel_z_msq/1000, abs(snap.linear_accel_z_msq%1000),
            _FMT_SIGN(snap.gyro_x_dps, 100),   snap.gyro_x_dps/100, abs(snap.gyro_x_dps%100),
            _FMT_SIGN(snap.gyro_y_dps, 100),   snap.gyro_y_dps/100, abs(snap.gyro_y_dps%100),
            _FMT_SIGN(snap.gyro_z_dps, 100),   snap.gyro_z_dps/100, abs(snap.gyro_z_dps%100),
            (long)speed_ema_int,
            (long)totalTick,
            md,
            agc
        );

        if (len <= 0 || len >= static_cast<int>(sizeof(buffer)))
            return;

        m_serial.write(buffer, len);
    }
}
