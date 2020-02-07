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

#include "AP_StereoVision.h"
#include "AP_StereoVision_Backend.h"
#include "AP_StereoVision_MAV.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_StereoVision::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: Stereo camera camera connection type
    // @Description: Stereo camera camera connection type
    // @Values: 0:None,1:MAV
    // @User: Advanced
    AP_GROUPINFO("_TYPE", 0, AP_StereoVision, _type, 1),

    // @Param: _OFFS
    // @DisplayName: Stereo camera position offset with regard to body frame
    // @Description: Stereo camera position offset with regard to body frame
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("_OFFS", 1, AP_StereoVision, _pos_offset, 0.0f),

    // @Param: _ORIENT
    // @DisplayName: Stereo camera orientation
    // @Description: Stereo camera orientation
    // @Values: 0:Forward, 2:Right, 4:Back, 6:Left, 24:Up, 25:Down
    // @User: Advanced
    AP_GROUPINFO("_ORIENT", 2, AP_StereoVision, _orientation, ROTATION_NONE),

    AP_GROUPEND
};

AP_StereoVision::AP_StereoVision()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("StereoVision must be singleton");
    }
#endif
    _singleton = this;
}

// detect and initialise any sensors
void AP_StereoVision::init(uint32_t log_bit)
{
     _log_bit = log_bit;

    // create backend
    if (_type == AP_StereoVision_Type_MAV) {
        _driver = new AP_StereoVision_MAV(*this);
    }
}

// return true if sensor is enabled
bool AP_StereoVision::enabled() const
{
    return ((_type != AP_StereoVision_Type_None) && (_driver != nullptr));
}

// update stereo camera sensor data
void AP_StereoVision::update()
{
    if (!enabled()) {
        return;
    }

    // stereo vision
    if (_stv_state.last_processed_sensor_update_ms != _stv_state.last_update_ms) {
        _stv_state.last_processed_sensor_update_ms = _stv_state.last_update_ms;
    }

    // net inspection
    if (_ni_state.last_processed_sensor_update_ms != _ni_state.last_update_ms) {
        _ni_state.last_processed_sensor_update_ms = _ni_state.last_update_ms;
    }

    // phase correlation
    if (_pc_state.last_processed_sensor_update_ms != _pc_state.last_update_ms) {
        _pc_state.last_processed_sensor_update_ms = _pc_state.last_update_ms;
    }

    // marker detection
    if (_md_state.last_processed_sensor_update_ms != _md_state.last_update_ms) {
        _md_state.last_processed_sensor_update_ms = _md_state.last_update_ms;
    }

}

// return true if stereo vision input is basically healthy (we are receiving data)
bool AP_StereoVision::stereo_vision_healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _stv_state.last_update_ms) < AP_STEREOVISION_TIMEOUT_MS);
}

// return true if net inspection is basically healthy (we are receiving data)
bool AP_StereoVision::net_inspection_healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _ni_state.last_update_ms) < AP_STEREOVISION_TIMEOUT_MS);
}

// return true if phase correlation is basically healthy (we are receiving data)
bool AP_StereoVision::phase_corr_healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _pc_state.last_update_ms) < AP_STEREOVISION_TIMEOUT_MS);
}

// return true if marker detection is basically healthy (we are receiving data)
bool AP_StereoVision::marker_detection_healthy() const
{
    if (!enabled()) {
        return false;
    }

    // healthy if we have received sensor messages within the past 300ms
    return ((AP_HAL::millis() - _md_state.last_update_ms) < AP_STEREOVISION_TIMEOUT_MS);
}

// consume STEREO_VISION_ODOM MAVLink message
void AP_StereoVision::handle_stereo_vision_msg(const mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_stereo_vision_msg(msg);
    }
}

// consume NET_INSPECTION MAVLink message
void AP_StereoVision::handle_net_inspection_msg(const mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_net_inspection_msg(msg);
    }
}

// consume PHASE_CORR MAVLink message
void AP_StereoVision::handle_phase_correlation_msg(const mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_phase_correlation_msg(msg);
    }
}

// consume NETTRACKING_MARKER MAVLink message
void AP_StereoVision::handle_marker_detection_msg(const mavlink_message_t *msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    // call backend
    if (_driver != nullptr) {
        _driver->handle_marker_detection_msg(msg);
    }
}

// singleton instance
AP_StereoVision *AP_StereoVision::_singleton;

namespace AP {

AP_StereoVision *stereovision()
{
    return AP_StereoVision::get_singleton();
}

}
