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
    struct StereoVisionState {
        float distance;    // distance to nearest poincloud in front
        float delta_pitch; // difference pitch angle (centi-degrees) between the vehicle's heading and the targeted object plane
        float delta_yaw; // difference yaw angle (centi-degrees) between the vehicle's heading and the targeted object plane
        uint64_t time_delta_usec;   // time delta (in usec) between previous and most recent update
        float confidence;           // confidence expressed as a value from 0 (no confidence) to 100 (very confident)
        uint32_t last_update_ms;    // system time (in milliseconds) of last update
        uint32_t last_processed_sensor_update_ms; // timestamp of last sensor update that was processed

    };

    struct NetInspectionState {
        uint64_t time_delta_usec; // time delta (in usec) between previous and most recent update
        uint32_t last_update_ms;  // system time (in milliseconds) of last update from net inspector
        uint32_t last_processed_sensor_update_ms; // timestamp of last net inspection update that was processed
        float mesh_distribution; // ratio of net meshes on left image side
        uint32_t mesh_count; // absolute amount of net meshes
    };

    struct PhaseCorrState {
        uint64_t time_delta_usec; // time delta (in usec) between previous and most recent update
        uint32_t last_update_ms;  // system time (in milliseconds) of last update from net inspector
        uint32_t last_processed_sensor_update_ms; // timestamp of last net inspection update that was processed
        Vector2f phase_shift; // translational shift between current and previous image
        Vector2f phase_shift_sum; // accumulated shift between current and first received image
    };

    struct MarkerDetectionState {
        uint64_t time_delta_usec; // time delta (in usec) between previous and most recent update
        uint32_t last_update_ms;  // system time (in milliseconds) of last update from net inspector
        uint32_t last_processed_sensor_update_ms; // timestamp of last net inspection update that was processed
        bool marker_visible; // whether a marker is currently detected
        bool terminate; // whether a termination marker is currently detected
        float horizontal_pos; // horizontal position of the upper marker related to the image width (-1 ... 1)
    };

    // detect and initialise any sensors
    void init(uint32_t log_bit);     // bitmask bit which indicates if we should log.  -1 means we always log

    // should be called really, really often.  The faster you call
    // this the lower the latency of the data fed to the estimator.
    void update();

    // return true if sensor is enabled
    bool enabled() const;

    // return true if sensor is basically healthy (we are receiving data)
    bool stereo_vision_healthy() const;
    bool net_inspection_healthy() const;
    bool phase_corr_healthy() const;
    bool marker_detection_healthy() const;

    // return a 3D vector defining the position offset of the camera in meters relative to the body frame origin
    const Vector3f &get_pos_offset(void) const { return _pos_offset; }

    // consume data from MAVLink messages
    void handle_stereo_vision_msg(const mavlink_message_t *msg);
    void handle_net_inspection_msg(const mavlink_message_t *msg);
    void handle_phase_correlation_msg(const mavlink_message_t *msg);
    void handle_marker_detection_msg(const mavlink_message_t *msg);

    static const struct AP_Param::GroupInfo var_info[];

    // state accessors
    // Stereo Vision
    const float &get_distance() const { return _stv_state.distance; }
    const float &get_delta_pitch() const { return _stv_state.delta_pitch; }
    const float &get_delta_yaw() const { return _stv_state.delta_yaw; }
    const float &get_stv_confidence() const { return _stv_state.confidence; }
    const uint64_t &get_stv_time_delta_usec() const { return _stv_state.time_delta_usec; }
    const uint32_t &get_last_stv_update_ms() const { return _stv_state.last_update_ms; }

    // net inspection
    const uint32_t &get_mesh_count() const { return _ni_state.mesh_count; }
    const float &get_mesh_distr() const { return _ni_state.mesh_distribution; }
    const uint64_t &get_ni_time_delta_usec() const { return _ni_state.time_delta_usec; }
    const uint32_t &get_last_ni_update_ms() const { return _ni_state.last_update_ms; }

    //phase correlation
    const Vector2f &get_cur_transl_shift() const { return _pc_state.phase_shift; }
    const Vector2f &get_acc_transl_shift() const { return _pc_state.phase_shift_sum; }
    const uint64_t &get_pc_time_delta_usec() const { return _pc_state.time_delta_usec; }
    const uint32_t &get_last_pc_update_ms() const { return _pc_state.last_update_ms; }

    // marker detection
    const bool &marker_visible() const { return _md_state.marker_visible; }
    const bool &marker_terminate() const { return _md_state.terminate; }
    const float &marker_horizontal_pos() const { return _md_state.horizontal_pos; }
    const uint64_t &get_md_time_delta_usec() const { return _md_state.time_delta_usec; }
    const uint32_t &get_last_md_update_ms() const { return _md_state.last_update_ms; }


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
    StereoVisionState _stv_state;
    NetInspectionState _ni_state;
    PhaseCorrState _pc_state;
    MarkerDetectionState _md_state;
};

namespace AP {
    AP_StereoVision *stereovision();
};
