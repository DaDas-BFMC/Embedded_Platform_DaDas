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

#ifndef IRCAMERA_HPP
#define IRCAMERA_HPP

#include <mbed.h>

namespace drivers
{
    /**
     * @brief IR Camera sensor driver for line detection
     * 
     * Reads IR sensor array to detect lane lines for lane keeping functionality.
     * Supports multiple IR sensors arranged in a line to detect the position
     * of a line relative to the center of the sensor array.
     */
    class CIRCamera
    {
        public:
            /* Constructor */
            CIRCamera(
                PinName f_sensor1,
                PinName f_sensor2,
                PinName f_sensor3,
                PinName f_sensor4,
                PinName f_sensor5
            );
            
            /* Destructor */
            ~CIRCamera();
            
            /**
             * @brief Read all sensor values
             * @param f_values Array to store sensor readings (should have 5 elements)
             */
            void readSensors(int* f_values);
            
            /**
             * @brief Calculate line position from sensor readings
             * @return Position error: negative = line to left, positive = line to right, 0 = centered
             */
            int getLinePosition();
            
            /**
             * @brief Check if line is detected by any sensor
             * @return true if line is detected, false otherwise
             */
            bool isLineDetected();
            
            /**
             * @brief Calibrate sensors for white/black threshold
             */
            void calibrate();
            
        private:
            /** @brief Analog inputs for IR sensors */
            AnalogIn m_sensor1;
            AnalogIn m_sensor2;
            AnalogIn m_sensor3;
            AnalogIn m_sensor4;
            AnalogIn m_sensor5;
            
            /** @brief Threshold for line detection (0.0 to 1.0) */
            float m_threshold;
            
            /** @brief Calculate weighted position from sensor array */
            int calculateWeightedPosition(int* f_values);
    };
}

#endif // IRCAMERA_HPP
