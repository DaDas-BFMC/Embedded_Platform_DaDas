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

#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include <mbed.h>

namespace utils
{
    /**
     * @brief PID Controller implementation
     * 
     * Generic PID controller for closed-loop control systems.
     * Calculates control output based on proportional, integral, and derivative terms.
     */
    class CPIDController
    {
        public:
            /* Constructor */
            CPIDController(
                float f_kp,
                float f_ki,
                float f_kd,
                float f_output_min,
                float f_output_max
            );
            
            /* Destructor */
            ~CPIDController();
            
            /**
             * @brief Calculate PID control output
             * @param f_setpoint Desired value (target)
             * @param f_process_value Current value (measurement)
             * @param f_dt Time step in seconds
             * @return Control output value
             */
            float compute(float f_setpoint, float f_process_value, float f_dt);
            
            /**
             * @brief Reset PID controller state
             */
            void reset();
            
            /**
             * @brief Set PID gains
             * @param f_kp Proportional gain
             * @param f_ki Integral gain
             * @param f_kd Derivative gain
             */
            void setGains(float f_kp, float f_ki, float f_kd);
            
            /**
             * @brief Set output limits
             * @param f_min Minimum output value
             * @param f_max Maximum output value
             */
            void setOutputLimits(float f_min, float f_max);
            
            /**
             * @brief Get last error value
             * @return Last error
             */
            float getLastError();
            
        private:
            /** @brief Proportional gain */
            float m_kp;
            
            /** @brief Integral gain */
            float m_ki;
            
            /** @brief Derivative gain */
            float m_kd;
            
            /** @brief Output minimum limit */
            float m_output_min;
            
            /** @brief Output maximum limit */
            float m_output_max;
            
            /** @brief Previous error for derivative calculation */
            float m_prev_error;
            
            /** @brief Accumulated integral term */
            float m_integral;
            
            /** @brief Clamp value to output limits */
            float clamp(float value);
    };
}

#endif // PIDCONTROLLER_HPP
