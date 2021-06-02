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

#include "AP_StereoVision_Backend.h"

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_StereoVision_Backend::AP_StereoVision_Backend(AP_StereoVision &frontend) :
    _frontend(frontend)
{
}

// set deltas (used by backend to update state)
void AP_StereoVision_Backend::set_stereovision_odometry(float distance, float delta_pitch, float delta_yaw, uint64_t time_delta_usec, float confidence)
{
    _frontend._stv_state.distance = distance;
    _frontend._stv_state.delta_pitch = RadiansToCentiDegrees(delta_pitch);
    _frontend._stv_state.delta_yaw = RadiansToCentiDegrees(delta_yaw);
    _frontend._stv_state.time_delta_usec = time_delta_usec;
    _frontend._stv_state.confidence = confidence;
    _frontend._stv_state.last_update_ms = AP_HAL::millis();
}

void AP_StereoVision_Backend::set_net_inspection_data(uint32_t mesh_count, float mesh_distribution, uint64_t time_delta_usec)
{
    _frontend._ni_state.mesh_count = mesh_count;
    _frontend._ni_state.mesh_distribution = mesh_distribution;
    _frontend._ni_state.time_delta_usec = time_delta_usec;
    _frontend._ni_state.last_update_ms = AP_HAL::millis();
}

void AP_StereoVision_Backend::set_opt_flow_data(float opt_flow_x, float opt_flow_y, float opt_flow_sum_x, float opt_flow_sum_y, uint64_t time_delta_usec)
{
    Vector2f opt_flow(opt_flow_x, opt_flow_y);
    Vector2f opt_flow_sum(opt_flow_sum_x, opt_flow_sum_y);

    _frontend._of_state.opt_flow = opt_flow;
    _frontend._of_state.opt_flow_sum = opt_flow_sum;
    _frontend._of_state.time_delta_usec = time_delta_usec;
    _frontend._of_state.last_update_ms = AP_HAL::millis();
}

void AP_StereoVision_Backend::set_marker_detection_data(uint8_t marker_visible, uint8_t terminate, float horizontal_pos, uint64_t time_delta_usec)
{
    _frontend._md_state.marker_visible = marker_visible == 1;
    _frontend._md_state.terminate = terminate == 1;
    _frontend._md_state.horizontal_pos = horizontal_pos;
    _frontend._md_state.time_delta_usec = time_delta_usec;
    _frontend._md_state.last_update_ms = AP_HAL::millis();

}
