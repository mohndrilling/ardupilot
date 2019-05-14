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
void AP_StereoVision_Backend::set_deltas(const Vector3f &angle_delta, const Vector3f& position_delta, uint64_t time_delta_usec, float confidence)
{
    _frontend._state.time_delta_usec = time_delta_usec;
    _frontend._state.confidence = confidence;
    _frontend._state.last_sensor_update_ms = AP_HAL::millis();
}
