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

#include "brain/laneperception.hpp"

namespace brain
{
    /**
     * @brief CLanePerception constructor
     * 
     * @param f_camera Reference to IR camera object
     */
    CLanePerception::CLanePerception(drivers::CIRCamera& f_camera)
        : m_camera(f_camera)
        , m_lane_error(0)
        , m_filtered_error(0)
        , m_lane_detected(false)
        , m_filter_alpha(0.3f)  // 30% new value, 70% old value for smoothing
    {
    }

    /**
     * @brief CLanePerception destructor
     */
    CLanePerception::~CLanePerception()
    {
    }

    /**
     * @brief Update lane perception with latest camera data
     * 
     * Reads camera, calculates lane position, and applies filtering
     * 
     * @return true if lane is detected
     */
    bool CLanePerception::update()
    {
        // Check if line is detected
        m_lane_detected = m_camera.isLineDetected();
        
        if (m_lane_detected)
        {
            // Get current lane position
            m_lane_error = m_camera.getLinePosition();
            
            // Apply filtering for smoother output
            applyFilter(m_lane_error);
        }
        else
        {
            // No lane detected - maintain last filtered value
            m_lane_error = 0;
        }
        
        return m_lane_detected;
    }

    /**
     * @brief Get current lane position error
     * 
     * @return Raw position error from camera
     */
    int CLanePerception::getLaneError()
    {
        return m_lane_error;
    }

    /**
     * @brief Get filtered lane position error
     * 
     * @return Filtered position error (smoothed over time)
     */
    int CLanePerception::getFilteredLaneError()
    {
        return m_filtered_error;
    }

    /**
     * @brief Check if lane is currently detected
     * 
     * @return Lane detection status
     */
    bool CLanePerception::isLaneDetected()
    {
        return m_lane_detected;
    }

    /**
     * @brief Reset perception state
     * 
     * Clears all error values and detection status
     */
    void CLanePerception::reset()
    {
        m_lane_error = 0;
        m_filtered_error = 0;
        m_lane_detected = false;
    }

    /**
     * @brief Apply exponential moving average filter
     * 
     * Formula: filtered = alpha * new_value + (1 - alpha) * old_filtered
     * 
     * @param new_value New lane error value to filter
     */
    void CLanePerception::applyFilter(int new_value)
    {
        m_filtered_error = static_cast<int>(
            m_filter_alpha * new_value + (1.0f - m_filter_alpha) * m_filtered_error
        );
    }
}
