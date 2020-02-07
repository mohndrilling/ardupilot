/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_StereoVision.h"

class AP_StereoVision_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_StereoVision_Backend(AP_StereoVision &frontend);

    // mavlink message callbacks
    virtual void handle_stereo_vision_msg(const mavlink_message_t *msg) {};
    virtual void handle_net_inspection_msg(const mavlink_message_t *msg) {};
    virtual void handle_phase_correlation_msg(const mavlink_message_t *msg) {};
    virtual void handle_marker_detection_msg(const mavlink_message_t *msg) {};

protected:

    // fill the data structs
    void set_stereovision_odometry(float distance, float delta_pitch, float delta_yaw, uint64_t time_delta_usec, float confidence);

    void set_net_inspection_data(uint32_t mesh_count, float mesh_distribution, uint64_t time_delta_usec);

    void set_phase_corr_data(float phase_shift_x, float phase_shift_y, float phase_shift_sum_x, float phase_shift_sum_y, uint64_t time_delta_usec);

    void set_marker_detection_data(uint8_t marker_visible, uint8_t terminate, float horizontal_pos, uint64_t time_delta_usec);

private:

    // references
    AP_StereoVision &_frontend;
};
