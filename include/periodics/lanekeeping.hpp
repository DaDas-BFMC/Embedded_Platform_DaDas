/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */

#ifndef LANEKEEPING_HPP
#define LANEKEEPING_HPP

#include <mbed.h>
#include <utils/taskmanager.hpp>
#include <utils/pidcontroller.hpp>
#include <brain/laneperception.hpp>
#include <drivers/steeringmotor.hpp>
#include <drivers/speedingmotor.hpp>
#include <brain/globalsv.hpp>

namespace periodics
{
    /**
     * @brief Lane keeping controller
     * 
     * Periodic task that reads lane position from perception module,
     * calculates steering correction using PID control, and commands
     * the steering motor to keep the vehicle centered in the lane.
     */
    class CLaneKeeping : public utils::CTask
    {
        public:
            /* Constructor */
            CLaneKeeping(
                std::chrono::milliseconds f_period,
                brain::CLanePerception& f_perception,
                drivers::ISteeringCommand& f_steering,
                drivers::ISpeedingCommand& f_speeding,
                UnbufferedSerial& f_serial
            );
            
            /* Destructor */
            ~CLaneKeeping();
            
            /**
             * @brief Enable lane keeping
             */
            void enable();
            
            /**
             * @brief Disable lane keeping
             */
            void disable();
            
            /**
             * @brief Check if lane keeping is enabled
             * @return true if enabled
             */
            bool isEnabled();
            
            /**
             * @brief Set target speed for lane keeping
             * @param f_speed Speed in mm/s
             */
            void setSpeed(int f_speed);
            
            /**
             * @brief Set PID gains
             * @param f_kp Proportional gain
             * @param f_ki Integral gain
             * @param f_kd Derivative gain
             */
            void setPIDGains(float f_kp, float f_ki, float f_kd);
            
            /**
             * @brief Serial callback for lane keeping command
             * @param a Input command string
             * @param b Output response string
             */
            void serialCallbackLANEKEEPINGcommand(const char* a, char* b);
            
        private:
            /** @brief Main control loop execution */
            virtual void _run();
            
            /** @brief Reference to lane perception */
            brain::CLanePerception& m_perception;
            
            /** @brief Reference to steering control */
            drivers::ISteeringCommand& m_steering;
            
            /** @brief Reference to speed control */
            drivers::ISpeedingCommand& m_speeding;
            
            /** @brief Reference to serial port */
            UnbufferedSerial& m_serial;
            
            /** @brief PID controller for steering */
            utils::CPIDController m_pid;
            
            /** @brief Lane keeping enabled flag */
            bool m_enabled;
            
            /** @brief Target speed (mm/s) */
            int m_target_speed;
            
            /** @brief Period in seconds for PID calculation */
            float m_dt;
            
            /** @brief Steering angle scale factor */
            float m_steering_scale;
    };
}

#endif // LANEKEEPING_HPP
