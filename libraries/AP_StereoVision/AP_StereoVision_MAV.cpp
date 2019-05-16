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

// consume VISIOIN_POSITION_DELTA MAVLink message
void AP_StereoVision_MAV::handle_msg(const mavlink_message_t *msg)
{
    // decode message
    mavlink_stereo_vision_odom_t packet;
    mavlink_msg_stereo_vision_odom_decode(msg, &packet);

    const Vector3f lin_velocity(packet.lin_velocity[0], packet.lin_velocity[1], packet.lin_velocity[2]);

    set_stereovision_odometry(lin_velocity, packet.distance, packet.delta_pitch, packet.delta_yaw, packet.time_delta_usec, packet.confidence);
}
