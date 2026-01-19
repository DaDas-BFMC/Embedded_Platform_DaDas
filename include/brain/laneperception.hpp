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

#ifndef LANEPERCEPTION_HPP
#define LANEPERCEPTION_HPP

#include <mbed.h>
#include <drivers/ircamera.hpp>

namespace brain
{
    /**
     * @brief Lane perception module
     * 
     * Processes IR camera data to extract lane position information
     * and provides filtered/smoothed lane position for control loop
     */
    class CLanePerception
    {
        public:
            /* Constructor */
            CLanePerception(drivers::CIRCamera& f_camera);
            
            /* Destructor */
            ~CLanePerception();
            
            /**
             * @brief Update lane perception with latest camera data
             * @return true if lane is detected, false otherwise
             */
            bool update();
            
            /**
             * @brief Get current lane position error
             * @return Position error: negative = left, positive = right, 0 = centered
             */
            int getLaneError();
            
            /**
             * @brief Get filtered (smoothed) lane position error
             * @return Filtered position error
             */
            int getFilteredLaneError();
            
            /**
             * @brief Check if lane is currently detected
             * @return true if lane is detected
             */
            bool isLaneDetected();
            
            /**
             * @brief Reset perception state
             */
            void reset();
            
        private:
            /** @brief Reference to IR camera */
            drivers::CIRCamera& m_camera;
            
            /** @brief Current lane position error */
            int m_lane_error;
            
            /** @brief Filtered lane position error */
            int m_filtered_error;
            
            /** @brief Lane detection status */
            bool m_lane_detected;
            
            /** @brief Filter coefficient for smoothing (0.0-1.0) */
            float m_filter_alpha;
            
            /** @brief Apply exponential moving average filter */
            void applyFilter(int new_value);
    };
}

#endif // LANEPERCEPTION_HPP
