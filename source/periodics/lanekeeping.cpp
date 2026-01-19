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

#include "periodics/lanekeeping.hpp"

namespace periodics
{
    /**
     * @brief CLaneKeeping constructor
     * 
     * @param f_period Control loop period
     * @param f_perception Reference to lane perception module
     * @param f_steering Reference to steering motor control
     * @param f_speeding Reference to speed motor control
     * @param f_serial Reference to serial communication
     */
    CLaneKeeping::CLaneKeeping(
        std::chrono::milliseconds f_period,
        brain::CLanePerception& f_perception,
        drivers::ISteeringCommand& f_steering,
        drivers::ISpeedingCommand& f_speeding,
        UnbufferedSerial& f_serial
    )
        : utils::CTask(f_period)
        , m_perception(f_perception)
        , m_steering(f_steering)
        , m_speeding(f_speeding)
        , m_serial(f_serial)
        , m_pid(1.5f, 0.01f, 0.5f, -250.0f, 250.0f)  // Initial PID gains and steering limits
        , m_enabled(false)
        , m_target_speed(0)
        , m_dt(f_period.count() / 1000.0f)  // Convert ms to seconds
        , m_steering_scale(2.0f)  // Scale factor for lane error to steering angle
    {
    }

    /**
     * @brief CLaneKeeping destructor
     */
    CLaneKeeping::~CLaneKeeping()
    {
    }

    /**
     * @brief Main control loop execution
     * 
     * Called periodically by task manager. Reads lane perception,
     * calculates PID control output, and commands steering and speed.
     */
    void CLaneKeeping::_run()
    {
        if (!m_enabled)
        {
            return;
        }
        
        // Update lane perception
        bool lane_detected = m_perception.update();
        
        if (lane_detected)
        {
            // Get filtered lane error (-100 to +100)
            int lane_error = m_perception.getFilteredLaneError();
            
            // PID controller: setpoint = 0 (centered), process_value = lane_error
            // Negative error means line is to left, need to steer left (negative angle)
            // So we invert the error for PID calculation
            float steering_output = m_pid.compute(0.0f, static_cast<float>(-lane_error), m_dt);
            
            // Apply steering scale
            int steering_angle = static_cast<int>(steering_output * m_steering_scale);
            
            // Command steering
            m_steering.setAngle(steering_angle);
            
            // Command speed
            m_speeding.setSpeed(m_target_speed);
            
            // Send status message
            char buffer[100];
            snprintf(buffer, sizeof(buffer), "@lk:1;%d;%d;;\r\n", lane_error, steering_angle);
            m_serial.write(buffer, strlen(buffer));
        }
        else
        {
            // Lane not detected - stop or maintain last command
            m_speeding.setSpeed(0);
            
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "@lk:0;0;0;;\r\n");
            m_serial.write(buffer, strlen(buffer));
        }
    }

    /**
     * @brief Enable lane keeping
     */
    void CLaneKeeping::enable()
    {
        m_enabled = true;
        m_pid.reset();
        m_perception.reset();
    }

    /**
     * @brief Disable lane keeping
     */
    void CLaneKeeping::disable()
    {
        m_enabled = false;
        m_steering.setAngle(0);
        m_speeding.setSpeed(0);
    }

    /**
     * @brief Check if lane keeping is enabled
     * 
     * @return true if enabled
     */
    bool CLaneKeeping::isEnabled()
    {
        return m_enabled;
    }

    /**
     * @brief Set target speed for lane keeping
     * 
     * @param f_speed Speed in mm/s
     */
    void CLaneKeeping::setSpeed(int f_speed)
    {
        m_target_speed = f_speed;
    }

    /**
     * @brief Set PID gains
     * 
     * @param f_kp Proportional gain
     * @param f_ki Integral gain
     * @param f_kd Derivative gain
     */
    void CLaneKeeping::setPIDGains(float f_kp, float f_ki, float f_kd)
    {
        m_pid.setGains(f_kp, f_ki, f_kd);
    }

    /**
     * @brief Serial callback for lane keeping command
     * 
     * Command format: "enable;speed" or "disable" or "pid;kp;ki;kd"
     * Examples: 
     *   "1;200" - enable with 200mm/s speed
     *   "0" - disable
     *   "pid;1.5;0.01;0.5" - set PID gains
     * 
     * @param a Input command string
     * @param b Output response string
     */
    void CLaneKeeping::serialCallbackLANEKEEPINGcommand(const char* a, char* b)
    {
        char cmd[20];
        int enable_flag, speed;
        float kp, ki, kd;
        
        // Check if PID configuration command
        if (sscanf(a, "%[^;];%f;%f;%f", cmd, &kp, &ki, &kd) == 4)
        {
            if (strcmp(cmd, "pid") == 0)
            {
                setPIDGains(kp, ki, kd);
                sprintf(b, "PID gains set: Kp=%.2f Ki=%.4f Kd=%.2f", kp, ki, kd);
                return;
            }
        }
        
        // Check if enable/disable with speed command
        if (sscanf(a, "%d;%d", &enable_flag, &speed) == 2)
        {
            if (enable_flag == 1)
            {
                setSpeed(speed);
                enable();
                sprintf(b, "Lane keeping enabled, speed=%d", speed);
            }
            else
            {
                disable();
                sprintf(b, "Lane keeping disabled");
            }
            return;
        }
        
        // Check if simple enable/disable
        if (sscanf(a, "%d", &enable_flag) == 1)
        {
            if (enable_flag == 1)
            {
                enable();
                sprintf(b, "Lane keeping enabled");
            }
            else
            {
                disable();
                sprintf(b, "Lane keeping disabled");
            }
            return;
        }
        
        sprintf(b, "syntax error");
    }
}
