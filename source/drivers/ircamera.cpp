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

#include "drivers/ircamera.hpp"

namespace drivers
{
    /**
     * @brief CIRCamera constructor
     * 
     * @param f_sensor1 Pin for left-most sensor
     * @param f_sensor2 Pin for left-center sensor
     * @param f_sensor3 Pin for center sensor
     * @param f_sensor4 Pin for right-center sensor
     * @param f_sensor5 Pin for right-most sensor
     */
    CIRCamera::CIRCamera(
        PinName f_sensor1,
        PinName f_sensor2,
        PinName f_sensor3,
        PinName f_sensor4,
        PinName f_sensor5
    )
        : m_sensor1(f_sensor1)
        , m_sensor2(f_sensor2)
        , m_sensor3(f_sensor3)
        , m_sensor4(f_sensor4)
        , m_sensor5(f_sensor5)
        , m_threshold(0.5f)  // Default threshold at 50%
    {
    }

    /**
     * @brief CIRCamera destructor
     */
    CIRCamera::~CIRCamera()
    {
    }

    /**
     * @brief Read all IR sensor values
     * 
     * @param f_values Array to store 5 sensor readings (0.0-1.0)
     */
    void CIRCamera::readSensors(int* f_values)
    {
        f_values[0] = (m_sensor1.read() > m_threshold) ? 1 : 0;
        f_values[1] = (m_sensor2.read() > m_threshold) ? 1 : 0;
        f_values[2] = (m_sensor3.read() > m_threshold) ? 1 : 0;
        f_values[3] = (m_sensor4.read() > m_threshold) ? 1 : 0;
        f_values[4] = (m_sensor5.read() > m_threshold) ? 1 : 0;
    }

    /**
     * @brief Calculate line position from IR sensor array
     * 
     * Uses weighted average of sensor positions to determine line position.
     * Position scale: -100 (far left) to +100 (far right), 0 = centered
     * 
     * @return Position error value
     */
    int CIRCamera::getLinePosition()
    {
        int sensor_values[5];
        readSensors(sensor_values);
        
        return calculateWeightedPosition(sensor_values);
    }

    /**
     * @brief Check if any sensor detects a line
     * 
     * @return true if at least one sensor is active
     */
    bool CIRCamera::isLineDetected()
    {
        int sensor_values[5];
        readSensors(sensor_values);
        
        for (int i = 0; i < 5; i++)
        {
            if (sensor_values[i] == 1)
            {
                return true;
            }
        }
        
        return false;
    }

    /**
     * @brief Calibrate sensor threshold
     * 
     * Reads current sensor values and sets threshold to midpoint
     * between detected white and black surfaces
     */
    void CIRCamera::calibrate()
    {
        // Simple calibration: average all sensor readings
        float sum = 0.0f;
        sum += m_sensor1.read();
        sum += m_sensor2.read();
        sum += m_sensor3.read();
        sum += m_sensor4.read();
        sum += m_sensor5.read();
        
        m_threshold = sum / 5.0f;
    }

    /**
     * @brief Calculate weighted position from sensor array
     * 
     * Weights: [-100, -50, 0, 50, 100] for sensors [0, 1, 2, 3, 4]
     * 
     * @param f_values Array of 5 binary sensor values (0 or 1)
     * @return Weighted position (-100 to +100)
     */
    int CIRCamera::calculateWeightedPosition(int* f_values)
    {
        const int weights[5] = {-100, -50, 0, 50, 100};
        
        int weighted_sum = 0;
        int active_count = 0;
        
        for (int i = 0; i < 5; i++)
        {
            if (f_values[i] == 1)
            {
                weighted_sum += weights[i];
                active_count++;
            }
        }
        
        // Return weighted average if sensors are active, otherwise 0
        if (active_count > 0)
        {
            return weighted_sum / active_count;
        }
        
        return 0;  // No line detected, assume centered
    }
}
