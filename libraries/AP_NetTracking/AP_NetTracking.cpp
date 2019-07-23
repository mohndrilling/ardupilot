#include "AP_NetTracking.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NetTracking::var_info[] = {

    // @Param: NET_SHAPE
    // @DisplayName: Defines the shape of the tracked net (plane or tube)
    // @Description: Defines the shape of the tracked net (plane or tube)
    // @Values: 0:Plane 1:Tube
    // @User: Advanced
    AP_GROUPINFO("NET_SHAPE", 0, AP_NetTracking, _net_shape, AP_NETTRACKING_NETSHAPE_DEFAULT),

    // @Param: DISTANCE
    // @DisplayName: Desired distance to the net during net tracking
    // @Description: Desired distance to the net during net tracking
    // @Units: cm
    // @Range: 10.0 100.0
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DISTANCE", 1, AP_NetTracking, _tracking_distance, AP_NETTRACKING_DISTANCE_DEFAULT),

    // @Param: MESH_CNT
    // @DisplayName: Desired count of visible net meshes during net tracking
    // @Description: Desired count of visible net meshes during net tracking
    // @Range: 100 800
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MESH_CNT", 2, AP_NetTracking, _tracking_meshcount, AP_NETTRACKING_MESH_CNT_DEFAULT),

    // @Param: VEL
    // @DisplayName: Desired lateral velocity during net tracking
    // @Description: Desired lateral velocity during net tracking
    // @Units: cm/s
    // @Range: -100.0 100.0
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VEL", 3, AP_NetTracking, _tracking_velocity, AP_NETTRACKING_VELOCITY_DEFAULT),

    // @Param: CTRL_VAR
    // @DisplayName: Whether to control the distance to the net or the amount of visible net meshes
    // @Description: Whether to control the distance to the net or the amount of visible net meshes
    // @Values: 0:Distance 1:MeshCount
    // @User: Advanced
    AP_GROUPINFO("CTRL_VAR", 4, AP_NetTracking, _control_var, AP_NETTRACKING_CTRL_VAR_DEFAULT),

    AP_GROUPEND
};

void AP_NetTracking::perform_net_tracking(float &forward_out, float &lateral_out)
{
    // time difference (in seconds) between two measurements from stereo vision is used to lowpass filter the data
    float dt = _stereo_vision.get_time_delta_usec() / 1000000.0f;

    if (dt > 2.0f)
    {
        _attitude_control.reset_yaw_err_filter();
    }

    // only update target distance and attitude, if new measurement from stereo data available
    bool update_target = _stereo_vision.get_last_update_ms() - last_stereo_update_ms != 0;

    last_stereo_update_ms = _stereo_vision.get_last_update_ms();

    // whether to perform vision based attitude control
    bool att_ctrl;

    if (_control_var == ctrl_meshcount)
    {
        // retrieve amount of currently visible net meshes
        float cur_mesh_cnt = _stereo_vision.get_mesh_count();


        // desired mesh count (control the square root of current mesh count, since total meshcount grows quadratically over the distance to the net)
        // but we want a linear dependency between control input (forward throttle) and control variable (square rooted mesh count)
        float d_mesh_cnt = _tracking_meshcount;
        float mesh_cnt_error = safe_sqrt(cur_mesh_cnt) - safe_sqrt(float(d_mesh_cnt));

        // get forward command from mesh count controller
        _pos_control.update_mesh_cnt_controller(forward_out, mesh_cnt_error, dt, update_target);

        att_ctrl = abs(mesh_cnt_error) < 5.0f;

    }
    else
    {
        // retrieve current distance from stereovision module
        float cur_dist = _stereo_vision.get_distance();

        // desired distance (m)
        float d_dist = float(_tracking_distance) / 100.0f;
        float dist_error = cur_dist - d_dist;

        // get forward command from distance controller
        _pos_control.update_dist_controller(forward_out, dist_error, dt, update_target);

        att_ctrl = abs(dist_error) < 10.0f;
    }



    // if distance error is small enough, use the stereovision heading data to always orientate the vehicle normal to the faced object surface
    if (att_ctrl)
    {
        // no roll desired
        float target_roll = 0.0f;

        // get pitch and yaw offset (in centidegrees) with regard to the faced object (net) in front
        float target_pitch_error = _stereo_vision.get_delta_pitch();
        float target_yaw_error = _stereo_vision.get_delta_yaw();

        // assume concave net shape -> only allow increasing/decreasing of yaw error w.r.t. the direction of movement
//        float scan_dir = is_negative(float(net_track_vel)) ? -1.0f : 1.0f;
//        target_yaw_error = scan_dir * target_yaw_error < 0 ? target_yaw_error : 0;

        // only change pitch when changing altitude
//        target_pitch_error = fabsf(channel_throttle->norm_input()-0.5f) > 0.05f ? target_pitch_error : 0.0f;

        // the target values will be ignored, if no new stereo vision data received (update_target = false)
        // this will update the target attitude corresponding to the current errors and trigger the attitude controller
        _attitude_control.input_euler_roll_pitch_yaw_accumulate(target_roll, target_pitch_error, target_yaw_error, dt, update_target);

        // scale net tracking velocity proportional to yaw error
//        float ls_tmp = target_yaw_error / 2000.0f;
//        float lat_scale_f = ls_tmp * ls_tmp * ls_tmp; // ... to be beautified

//        // set commands for lateral motion
//        lateral_out = (1.0f + lat_scale_f) * net_track_vel / 100.0f;


        // update net tracking velocity
        net_track_vel = _tracking_velocity;

        // if the net shape equals an open plane, perform net edge detection based on the mesh distribution on the current image
        // if net edge detected, reverse the lateral output velocity
        if (_net_shape == NetShape::Plane)
        {
            float mesh_distr = _stereo_vision.get_mesh_distr();
            nettr_toggle_velocity = nettr_toggle_velocity || fabs(mesh_distr - 0.5f) < 0.1f;
            float distr_thr = 0.15f;
            bool net_edge_reached = mesh_distr < distr_thr || mesh_distr > (1.0f - distr_thr);
            if (nettr_toggle_velocity && net_edge_reached)
            {
                nettr_direction *= -1.0f;
                nettr_toggle_velocity = false;
            }
        }

        lateral_out = nettr_direction * net_track_vel / 1000.0f;
    }
    else
    {
        // if the distance is too large, the vehicle is supposed to obtain the current attitude and to not move laterally
        // call attitude controller
        _attitude_control.input_euler_roll_pitch_yaw_accumulate(0.0f, 0.0f, 0.0f, dt, false);

        // no lateral movement
        lateral_out = 0.0f;
    }

}

