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
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

class AP_StereoVision_Backend;

#define AP_STEREOVISION_TIMEOUT_MS 300

class AP_StereoVision
{
public:
    friend class AP_StereoVision_Backend;

    AP_StereoVision();

    // get singleton instance
    static AP_StereoVision *get_singleton() {
        return _singleton;
    }

    // external position backend types (used by _TYPE parameter)
    enum AP_StereoVision_Type {
        AP_StereoVision_Type_None   = 0,
        AP_StereoVision_Type_MAV    = 1
    };

    // The AP_StereoVisionState structure is filled in by the backend driver
    struct AP_StereoVisionState {
        Vector3f lin_velocity;       // linear velocity with regard to body frame
        float distance;    // distance to nearest poincloud in front
        float delta_pitch; // difference pitch angle (centi-degrees) between the vehicle's heading and the targeted object plane
        float delta_yaw; // difference yaw angle (centi-degrees) between the vehicle's heading and the targeted object plane
        uint32_t mesh_count; // amount of currently visible net meshes
        uint64_t time_delta_usec;   // time delta (in usec) between previous and most recent update
        float confidence;           // confidence expressed as a value from 0 (no confidence) to 100 (very confident)
        uint32_t last_sensor_update_ms;    // system time (in milliseconds) of last update from sensor
        uint32_t last_processed_sensor_update_ms; // timestamp of last sensor update that was processed

    };

    // detect and initialise any sensors
    void init(uint32_t log_bit);     // bitmask bit which indicates if we should log.  -1 means we always log

    // should be called really, really often.  The faster you call
    // this the lower the latency of the data fed to the estimator.
    void update();

    // return true if sensor is enabled
    bool enabled() const;

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() const;

    // return a 3D vector defining the position offset of the camera in meters relative to the body frame origin
    const Vector3f &get_pos_offset(void) const { return _pos_offset; }

    // consume data from MAVLink messages
    void handle_msg(const mavlink_message_t *msg);

    static const struct AP_Param::GroupInfo var_info[];

    // state accessors
    const Vector3f &get_lin_velocity() const { return _state.lin_velocity; }
    const float &get_distance() const { return _state.distance; }
    const float &get_delta_pitch() const { return _state.delta_pitch; }
    const float &get_delta_yaw() const { return _state.delta_yaw; }
    const uint32_t &get_mesh_count() const { return _state.mesh_count; }
    const uint64_t &get_time_delta_usec() const { return _state.time_delta_usec; }
    const float &get_confidence() const { return _state.confidence; }
    const uint32_t &get_last_update_ms() const { return _state.last_sensor_update_ms; }

private:

    static AP_StereoVision *_singleton;   

    // parameters
    AP_Int8 _type;
    AP_Vector3f _pos_offset;    // position offset of the camera in the body frame
    AP_Int8 _orientation;       // camera orientation on vehicle frame


    uint32_t _log_bit = -1;     // bitmask bit which indicates if we should log.  -1 means we always log

    // reference to backends
    AP_StereoVision_Backend *_driver;

    // state of backend
    AP_StereoVisionState _state;
};

namespace AP {
    AP_StereoVision *stereovision();
};
