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

    // consume VISION_POSITION_DELTA MAVLink message
    virtual void handle_msg(mavlink_message_t &msg) {};

protected:

    // set deltas (used by backend to update state)
    void set_stereovision_odometry(const Vector3f &lin_velocity, float distance, float delta_pitch, float delta_yaw, uint32_t mesh_count, uint64_t time_delta_usec, float confidence);

private:

    // references
    AP_StereoVision &_frontend;
};
