#ifndef AS5600_ENCODER_HPP
#define AS5600_ENCODER_HPP

#include <mbed.h>
#include <chrono>
#include <utils/task.hpp>
#include <drivers/as5600.hpp>

namespace periodics
{
    class CAs5600Encoder : public utils::CTask
    {
        public:
            CAs5600Encoder(std::chrono::milliseconds f_period, UnbufferedSerial& f_serial);
            ~CAs5600Encoder();

        private:
            virtual void _run();

            drivers::CAs5600 m_sensor;
            drivers::CAs5600Tracker m_tracker;
            UnbufferedSerial& m_serial;
            bool m_isOk;
    };
}; // namespace periodics

#endif // AS5600_ENCODER_HPP
