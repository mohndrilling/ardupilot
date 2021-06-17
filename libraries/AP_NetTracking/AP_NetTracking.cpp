﻿#include "AP_NetTracking.h"
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

    // @Param: VEL_CTRL
    // @DisplayName: Whether to use phase correlation input to control the lateral velocity of the vehicle
    // @Description: Whether to use phase correlation input to control the lateral velocity of the vehicle
    // @Values: 0:Disabled 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("VEL_CTRL", 5, AP_NetTracking, _velocity_ctrl, AP_NETTRACKING_VEL_CTRL_DEFAULT),

    // @Param: PH_CRR_HZ
    // @DisplayName: Cut off frequency for low pass filter of phase correlation input
    // @Description: Cut off frequency for low pass filter of phase correlation input
    // @Units: Hz
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PH_CRR_HZ", 6, AP_NetTracking, _phase_shift_cutoff_freq, AP_NETTRACKING_PHASE_SHIFT_CUTOFF_FREQ_DEFAULT),

    // @Param: THR_SPEED
    // @DisplayName: The throttle speed during net tracking
    // @Description: The throttle amount applied, when the vehicle is moving down to perform scanning at a new altitude
    // @Range: 0.0 0.5
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("THR_SPEED", 7, AP_NetTracking, _throttle_speed, AP_NETTRACKING_THR_SPEED_DEFAULT),

    // @Param: PH_CRR_DST
    // @DisplayName: Desired optical distance the image is supposed to travel during throttle state
    // @Description: Desired optical distance the image is supposed to travel during throttle state
    // @Range: 10.0 600.0
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("PH_CRR_DST", 8, AP_NetTracking, _phase_shift_thr_dist, AP_NETTRACKING_PHASE_SHIFT_THR_DIST_DEFAULT),

    AP_GROUPEND
};

void AP_NetTracking::reset()
{
    _phase_shift_filt.reset();
}

void AP_NetTracking::init()
{
    _initial_yaw = _attitude_control.get_accumulated_yaw();

    // update time stamps
    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();
    _last_mesh_data_update_ms = _stereo_vision.get_last_ni_update_ms();
    _last_phase_corr_update_ms = _stereo_vision.get_last_pc_update_ms();
}

void AP_NetTracking::perform_net_tracking(float &forward_out, float &lateral_out, float &throttle_out)
{
    // time difference (in seconds) between two measurements from stereo vision is used to lowpass filter the data
    float stv_dt = _stereo_vision.get_stv_time_delta_usec() / 1000000.0f; // stereo vision net tracking messages
    float ni_dt = _stereo_vision.get_ni_time_delta_usec() / 1000000.0f; // net inspection  messages
    float pc_dt = _stereo_vision.get_pc_time_delta_usec() / 1000000.0f; // phase correlation messages

    if (stv_dt > 2.0f)
    {
        _attitude_control.reset_yaw_err_filter();
    }

    // only update target distance and attitude, if new measurement from stereo data available
    bool stv_updated = _stereo_vision.get_last_stv_update_ms() - _last_stereo_update_ms != 0;
    bool ni_updated = _stereo_vision.get_last_ni_update_ms() - _last_mesh_data_update_ms != 0;
    bool pc_updated = _stereo_vision.get_last_pc_update_ms() - _last_phase_corr_update_ms != 0;

    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();
    _last_mesh_data_update_ms = _stereo_vision.get_last_ni_update_ms();
    _last_phase_corr_update_ms = _stereo_vision.get_last_pc_update_ms();

    //
    bool forward_update = _control_var == ctrl_meshcount ? ni_updated : stv_updated;
    float forward_dt = _control_var == ctrl_meshcount ? ni_dt : stv_dt;

    switch (_state)
    {
        case State::Pause:
            forward_out = 0.0f;
            lateral_out = 0.0f;
            throttle_out = 0.0f;
            update_heading_control(stv_updated, stv_dt);
            break;

        case State::Scanning:
            update_forward_out(forward_out, forward_update, forward_dt);

            if (_perform_att_ctrl)
            {
                update_heading_control(stv_updated, stv_dt);
                update_lateral_out(lateral_out, pc_updated, pc_dt);
            }
            else
            {
                // if the distance is too large, the vehicle is supposed to obtain the current attitude and to not move laterally
                // call attitude controller
                _attitude_control.input_euler_roll_pitch_yaw_accumulate(0.0f, 0.0f, 0.0f, stv_dt, false);

                // no lateral movement
                lateral_out = 0.0f;
            }

            throttle_out = 0.0f;
            break;

        case State::Throttle:

            update_forward_out(forward_out, forward_update, forward_dt);
            update_heading_control(stv_updated, stv_dt);
            lateral_out = 0.0f;
            update_throttle_out(throttle_out, pc_updated, pc_dt);
            break;

        default:

            break;

    }

}

void AP_NetTracking::update_lateral_out(float &lateral_out, bool update_target, float dt)
{
    // scale net tracking velocity proportional to yaw error
//        float ls_tmp = target_yaw_error / 2000.0f;
//        float lat_scale_f = ls_tmp * ls_tmp * ls_tmp; // ... to be beautified

//        // set commands for lateral motion
//        lateral_out = (1.0f + lat_scale_f) * _nettr_velocity / 100.0f;

    // update phase shift input
    if (update_target)
        update_phase_shift(dt);

    // update net tracking velocity
    if (_velocity_ctrl)
    {
        // call controller to hold desired lateral velocity
        // interprete phase shift as optical flow by calculating derivation
        float d_optfl_x = _nettr_direction * _tracking_velocity; // factor needed to scale to reasonable optical flow setpoint
        float optfl_x_error = d_optfl_x - _stereo_vision.get_cur_transl_shift().x / dt;

        // get forward command from mesh count controller
        _pos_control.update_optfl_controller(_nettr_velocity, optfl_x_error, dt, update_target);

        gcs().send_named_float("lat_out", _nettr_velocity);
        gcs().send_named_float("optfl_err", optfl_x_error);


    }
    else
    {
        _nettr_velocity = _nettr_direction * _tracking_velocity;
    }


    // if the net shape equals an open plane, perform net edge detection based on the mesh distribution on the current image
    // if net edge detected, reverse the lateral output velocity
    if (_net_shape == NetShape::Plane)
    {
        float mesh_distr = _stereo_vision.get_mesh_distr();
        _nettr_toggle_velocity = _nettr_toggle_velocity || fabs(mesh_distr - 0.5f) < 0.1f;
        float distr_thr = 0.15f;
        bool net_edge_reached = mesh_distr < distr_thr || mesh_distr > (1.0f - distr_thr);
        if (_nettr_toggle_velocity && net_edge_reached)
        {
            _nettr_direction *= -1.0f;
            _nettr_toggle_velocity = false;
            _pos_control.relax_optfl_controller();

            // switch to Throttle state and store the current absolute phase shift y distance to track the distance that the image is traveling during throttle state
            _state = State::Throttle;
            _initial_phase_shift_sumy = _phase_shift_sum_y;
        }
    }
    else if (_net_shape == NetShape::Tube)
    {
        // check if ROV has performed 360 degrees of scanning
        gcs().send_named_float("dyaw", fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw));
        if (fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) > 360.0f)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Changing to Throttle");
            // toggle scanning direction
            _nettr_direction *= -1;
            _initial_yaw = _attitude_control.get_accumulated_yaw();

            _attitude_control.reset_yaw_err_filter();

            // switch to Throttle state and store the current absolute opt flow y distance to track the distance that the image is traveling during throttle state
            _state = State::Throttle;
            _initial_phase_shift_sumy = _phase_shift_sum_y;
        }
    }

    lateral_out = _nettr_velocity / 250.0f;
}

void AP_NetTracking::update_forward_out(float &forward_out, bool update_target, float dt)
{
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

        _perform_att_ctrl = abs(mesh_cnt_error) < 5.0f;

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

        _perform_att_ctrl = abs(dist_error) < 10.0f;
    }
}

void AP_NetTracking::update_heading_control(bool update_target, float dt)
{
    // no roll desired
    float target_roll = 0.0f;

    // get pitch and yaw offset (in centidegrees) with regard to the faced object (net) in front
    float target_pitch_error = _stereo_vision.get_delta_pitch();
    float target_yaw_error = _stereo_vision.get_delta_yaw();

    // assume concave net shape -> only allow increasing/decreasing of yaw error w.r.t. the direction of movement
//        float scan_dir = is_negative(float(_nettr_velocity)) ? -1.0f : 1.0f;
//        target_yaw_error = scan_dir * target_yaw_error < 0 ? target_yaw_error : 0;

    // only change pitch when changing altitude
//        target_pitch_error = fabsf(channel_throttle->norm_input()-0.5f) > 0.05f ? target_pitch_error : 0.0f;

    // the target values will be ignored, if no new stereo vision data received (update_target = false)
    // this will update the target attitude corresponding to the current errors and trigger the attitude controller
    _attitude_control.input_euler_roll_pitch_yaw_accumulate(target_roll, target_pitch_error, target_yaw_error, dt, update_target);
}

void AP_NetTracking::update_throttle_out(float &throttle_out, bool update_target, float dt)
{
    // constantly moving downwards
    throttle_out = - _throttle_speed;

    if (update_target)
        update_phase_shift(dt);

    if (fabs(_phase_shift_sum_y - _initial_phase_shift_sumy) > _phase_shift_thr_dist)
    {

        _state = State::Scanning;
    }

}

void AP_NetTracking::update_phase_shift(float dt)
{
    // update phase_shift input
    Vector2f phase_shift = _stereo_vision.get_acc_transl_shift();

    // lowpass filter
    _phase_shift_filt.set_cutoff_frequency(_phase_shift_cutoff_freq);
    // append negative value, because x-axes (and z-axes) of coordinate systems used in phase correlation node and ardusub are pointing in opposite directions
    _phase_shift_filt.apply(-phase_shift, dt);

    _phase_shift_sum_x = _phase_shift_filt.get().x;
    _phase_shift_sum_y = _phase_shift_filt.get().y;

    gcs().send_named_float("im_shift_x", _phase_shift_filt.get().x);
    gcs().send_named_float("im_shift_y", _phase_shift_filt.get().y);
}

