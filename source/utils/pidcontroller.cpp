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

#include "utils/pidcontroller.hpp"

namespace utils
{
    /**
     * @brief CPIDController constructor
     * 
     * @param f_kp Proportional gain
     * @param f_ki Integral gain
     * @param f_kd Derivative gain
     * @param f_output_min Minimum output limit
     * @param f_output_max Maximum output limit
     */
    CPIDController::CPIDController(
        float f_kp,
        float f_ki,
        float f_kd,
        float f_output_min,
        float f_output_max
    )
        : m_kp(f_kp)
        , m_ki(f_ki)
        , m_kd(f_kd)
        , m_output_min(f_output_min)
        , m_output_max(f_output_max)
        , m_prev_error(0.0f)
        , m_integral(0.0f)
    {
    }

    /**
     * @brief CPIDController destructor
     */
    CPIDController::~CPIDController()
    {
    }

    /**
     * @brief Compute PID control output
     * 
     * Calculates PID output using the formula:
     * output = Kp*error + Ki*integral + Kd*derivative
     * 
     * @param f_setpoint Desired value (target)
     * @param f_process_value Current measured value
     * @param f_dt Time step in seconds
     * @return Clamped control output
     */
    float CPIDController::compute(float f_setpoint, float f_process_value, float f_dt)
    {
        // Calculate error
        float error = f_setpoint - f_process_value;
        
        // Proportional term
        float p_term = m_kp * error;
        
        // Integral term (with anti-windup)
        m_integral += error * f_dt;
        float i_term = m_ki * m_integral;
        
        // Derivative term
        float derivative = (error - m_prev_error) / f_dt;
        float d_term = m_kd * derivative;
        
        // Calculate total output
        float output = p_term + i_term + d_term;
        
        // Clamp output to limits
        output = clamp(output);
        
        // Anti-windup: prevent integral windup when output is saturated
        if (output == m_output_max || output == m_output_min)
        {
            m_integral -= error * f_dt;  // Undo integral accumulation
        }
        
        // Store error for next iteration
        m_prev_error = error;
        
        return output;
    }

    /**
     * @brief Reset PID controller state
     * 
     * Clears integral and previous error terms
     */
    void CPIDController::reset()
    {
        m_prev_error = 0.0f;
        m_integral = 0.0f;
    }

    /**
     * @brief Set PID gains
     * 
     * @param f_kp Proportional gain
     * @param f_ki Integral gain
     * @param f_kd Derivative gain
     */
    void CPIDController::setGains(float f_kp, float f_ki, float f_kd)
    {
        m_kp = f_kp;
        m_ki = f_ki;
        m_kd = f_kd;
    }

    /**
     * @brief Set output limits
     * 
     * @param f_min Minimum output value
     * @param f_max Maximum output value
     */
    void CPIDController::setOutputLimits(float f_min, float f_max)
    {
        m_output_min = f_min;
        m_output_max = f_max;
    }

    /**
     * @brief Get last error value
     * 
     * @return Previous error value
     */
    float CPIDController::getLastError()
    {
        return m_prev_error;
    }

    /**
     * @brief Clamp value to output limits
     * 
     * @param value Value to clamp
     * @return Clamped value
     */
    float CPIDController::clamp(float value)
    {
        if (value > m_output_max)
        {
            return m_output_max;
        }
        else if (value < m_output_min)
        {
            return m_output_min;
        }
        
        return value;
    }
}
