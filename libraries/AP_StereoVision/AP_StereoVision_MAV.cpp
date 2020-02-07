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

#include <AP_HAL/AP_HAL.h>
#include "AP_StereoVision_MAV.h"
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_StereoVision_MAV::AP_StereoVision_MAV(AP_StereoVision &frontend) :
    AP_StereoVision_Backend(frontend)
{
}

// consume STEREO_VISION_ODOM MAVLink message
void AP_StereoVision_MAV::handle_stereo_vision_msg(const mavlink_message_t *msg)
{
    // decode message
    mavlink_stereo_vision_odom_t packet;
    mavlink_msg_stereo_vision_odom_decode(msg, &packet);

    set_stereovision_odometry(packet.distance, packet.delta_pitch, packet.delta_yaw, packet.time_delta_usec, packet.confidence);
}

// consume STEREO_VISION_ODOM MAVLink message
void AP_StereoVision_MAV::handle_net_inspection_msg(const mavlink_message_t *msg)
{
    // decode message
    mavlink_net_inspection_t packet;
    mavlink_msg_net_inspection_decode(msg, &packet);

    set_net_inspection_data(packet.mesh_count, packet.mesh_distribution, packet.time_delta_usec);
}

// consume STEREO_VISION_ODOM MAVLink message
void AP_StereoVision_MAV::handle_phase_correlation_msg(const mavlink_message_t *msg)
{
    // decode message
    mavlink_phase_corr_t packet;
    mavlink_msg_phase_corr_decode(msg, &packet);

    set_phase_corr_data(packet.phase_shift_x, packet.phase_shift_y, packet.phase_shift_sum_x, packet.phase_shift_sum_y, packet.time_delta_usec);
}

// consume STEREO_VISION_ODOM MAVLink message
void AP_StereoVision_MAV::handle_marker_detection_msg(const mavlink_message_t *msg)
{
    // decode message
    mavlink_nettracking_marker_t packet;
    mavlink_msg_nettracking_marker_decode(msg, &packet);

    set_marker_detection_data(packet.marker_visible, packet.terminate, packet.horizontal_pos, packet.time_delta_usec);
}
