#ifndef IMU_ENCODER_HPP
#define IMU_ENCODER_HPP

#include <mbed.h>
#include <chrono>
#include <utils/task.hpp>
#include <periodics/imu.hpp>
#include <drivers/as5600.hpp>

namespace periodics
{
    /**
     * Single periodic task that samples both IMU and AS5600 encoder,
     * computes EMA-filtered linear speed from the encoder, and sends
     * one combined serial frame per period. Activation is via #imu:1
     * (handled by CImu; this task checks bool_globalsV_imu_isActive).
     */
    class CImuEncoder : public utils::CTask
    {
    public:
        CImuEncoder(
            std::chrono::milliseconds f_period,
            UnbufferedSerial& f_serial,
            periodics::CImu& f_imu,
            PinName encoder_sda = PC_9,
            PinName encoder_scl = D7,
            PinName encoder_dir = D2
        );
        ~CImuEncoder();

    private:
        virtual void _run();

        UnbufferedSerial& m_serial;
        periodics::CImu& m_imu;
        drivers::CAs5600 m_encoderSensor;
        drivers::CAs5600Tracker m_encoderTracker;
        bool m_encoderOk;
        float m_emaSpeed;
        static constexpr float kEmaAlpha = 0.2f;
    };
}

#endif
