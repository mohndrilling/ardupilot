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
    // @DisplayName: Desired lateral velocity during net tracking in percent
    // @Description: Desired lateral velocity during net tracking in percent
    // @Units: percent
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

    // home position (defined by heading and altitude for now)
    _home_yaw = _ahrs.get_current_yaw();
    _home_altitude = _inav.get_altitude();

    // update time stamps
    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();
    _last_mesh_data_update_ms = _stereo_vision.get_last_ni_update_ms();
    _last_phase_corr_update_ms = _stereo_vision.get_last_pc_update_ms();
    _last_marker_detection_update_ms = _stereo_vision.get_last_md_update_ms();

    // set initial state
    _state = State::Scanning;
}

void AP_NetTracking::perform_net_tracking(float &forward_out, float &lateral_out, float &throttle_out)
{
    // time difference (in seconds) between two measurements from stereo vision is used to lowpass filter the data
    _sensor_intervals.stv_dt = _stereo_vision.get_stv_time_delta_usec() / 1000000.0f; // stereo vision net tracking messages
    _sensor_intervals.ni_dt = _stereo_vision.get_ni_time_delta_usec() / 1000000.0f; // net inspection  messages
    _sensor_intervals.pc_dt = _stereo_vision.get_pc_time_delta_usec() / 1000000.0f; // phase correlation messages
    _sensor_intervals.md_dt = _stereo_vision.get_md_time_delta_usec() / 1000000.0f; // marker detection messages

    if (_sensor_intervals.stv_dt > 2.0f)
    {
        _attitude_control.reset_yaw_err_filter();
    }

    // only update target distance and attitude, if new measurement from stereo data available
    _sensor_updates.stv_updated = _stereo_vision.get_last_stv_update_ms() - _last_stereo_update_ms != 0;
    _sensor_updates.ni_updated = _stereo_vision.get_last_ni_update_ms() - _last_mesh_data_update_ms != 0;
    _sensor_updates.pc_updated = _stereo_vision.get_last_pc_update_ms() - _last_phase_corr_update_ms != 0;
    _sensor_updates.md_updated = _stereo_vision.get_last_md_update_ms() - _last_marker_detection_update_ms != 0;

    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();
    _last_mesh_data_update_ms = _stereo_vision.get_last_ni_update_ms();
    _last_phase_corr_update_ms = _stereo_vision.get_last_pc_update_ms();
    _last_marker_detection_update_ms = _stereo_vision.get_last_md_update_ms();

    switch (_state)
    {
        case State::Pause:
            // call attitude controller, just keep the current attitude
            _attitude_control.input_euler_roll_pitch_yaw_accumulate(0.0f, 0.0f, 0.0f, _sensor_intervals.stv_dt, false);

            // perform distance control
            update_forward_out(forward_out);

            // perform opical flow stabilization
            update_lateral_out(lateral_out);

            // keep altitude
            throttle_out = 0.0f;

            break;

        case State::Scanning:
            scan(forward_out, lateral_out, throttle_out);
            break;

        case State::Throttle:

            update_forward_out(forward_out);
            update_lateral_out(lateral_out);
            update_heading_control();
            update_throttle_out(throttle_out);
            break;

        case State::ReturnToHomeHeading:
        {
            // continue performing distance and heading control
            scan(forward_out, lateral_out, throttle_out);

            // check termination condition
            float current_yaw = _ahrs.get_current_yaw();

            if (fabs(_home_yaw - current_yaw) < radians(3.0f))
                _state = State::ReturnToHomeAltitude;

            break;
        }

        case State::ReturnToHomeAltitude:
        {
            // continue performing distance and heading control
            scan(forward_out, lateral_out, throttle_out);

            // overwrite throttle
            // constant throttling towards home altitude
            float thr_dir = _home_altitude - _inav.get_altitude() < 0 ? -1.0f : 1.0f;
            throttle_out = thr_dir * _throttle_speed;

            // check termination condition
            if (fabs(_inav.get_altitude() - _home_altitude) < 5.0f)
                _state = State::Pause;

            break;
        }

        default:

            break;

    }

}


void AP_NetTracking::scan(float &forward_out, float &lateral_out, float &throttle_out)
{
    update_forward_out(forward_out);

    if (_perform_att_ctrl)
    {
        update_heading_control();
        update_lateral_out(lateral_out);
    }
    else
    {
        // if the distance is too large, the vehicle is supposed to obtain the current attitude and to not move laterally
        // call attitude controller
        _attitude_control.input_euler_roll_pitch_yaw_accumulate(0.0f, 0.0f, 0.0f, _sensor_intervals.stv_dt, false);

        // no lateral movement
        lateral_out = 0.0f;
    }

    throttle_out = 0.0f;
}

void AP_NetTracking::update_lateral_out(float &lateral_out)
{
    // update phase shift input
    if (_sensor_updates.pc_updated)
        update_phase_shift();

    // update net tracking velocity
    if (_velocity_ctrl)
    {
        // perform optical flow stabilization if measurements arrived
        if (_sensor_updates.pc_updated && fabs(_sensor_intervals.pc_dt) > 0.0f)
        {
            // negate the derivation since ardusub and phasecorr coordinate systems are flipped (z-axis in opposite directions)
            float cur_optfl_x = -_stereo_vision.get_cur_transl_shift().x / _sensor_intervals.pc_dt;

            // target optical flow velocity, only different from zero during net tracking
            float target_optfl_x = 0.0f;
            if (_state == State::Scanning || _state == State::ReturnToHomeHeading)
            {
                // scale the target tracking velocity to obtain a reasonable optical flow setpoint
                target_optfl_x = _nettr_direction * _tracking_velocity * _nettr_opt_flow_vel_factor;
            }
            _pos_control.update_optfl_controller(_nettr_velocity, cur_optfl_x, target_optfl_x, _sensor_intervals.pc_dt, _sensor_updates.pc_updated);
        }
    }
    else
    {
        if (_state == State::Scanning || _state == State::ReturnToHomeHeading)
        {
            _nettr_velocity = _nettr_direction * _tracking_velocity * _nettr_default_vel_factor;
        }
        else
        {
            _nettr_velocity = 0.0f;
        }
    }

    // scale to lateral_out command
    lateral_out = _nettr_velocity / 250.0f;

    // only perform following net edge detection/ loop detection, if we are in Scanning state
    if (_state != State::Scanning)
        return;

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
        update_loop_progress();

        // check if ROV has performed 360 degrees of scanning
        gcs().send_named_float("dyaw", fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw));
        if (fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) > 360.0f)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Changing to Throttle");
            // toggle scanning direction
            _nettr_direction *= -1;

            _attitude_control.reset_yaw_err_filter();

            // switch to Throttle state and store the current absolute opt flow y distance to track the distance that the image is traveling during throttle state
            _state = State::Throttle;
            _initial_phase_shift_sumy = _phase_shift_sum_y;

            // reset loop progress
            _loop_progress = -1;
        }
    }
}

void AP_NetTracking::update_forward_out(float &forward_out)
{    
    bool update_target = _control_var == ctrl_meshcount ? _sensor_updates.ni_updated : _sensor_updates.stv_updated;
    float dt = _control_var == ctrl_meshcount ? _sensor_intervals.ni_dt : _sensor_intervals.stv_dt;

    if (dt <= 0)
    {
        forward_out = 0.0f;
        return;
    }

    if (_control_var == ctrl_meshcount)
    {
        // retrieve amount of currently visible net meshes
        float cur_mesh_cnt = (float) _stereo_vision.get_mesh_count();

        float d_mesh_cnt = _tracking_meshcount;

        // get forward command from mesh count controller
        _pos_control.update_mesh_cnt_controller(forward_out, cur_mesh_cnt, d_mesh_cnt, dt, update_target);

        // mesh count error < 5.0, using square root to get linear dependency between mesh count error and distance from net
        _perform_att_ctrl = abs(safe_sqrt(d_mesh_cnt) - safe_sqrt(cur_mesh_cnt)) < 5.0f;

    }
    else
    {
        // retrieve current distance from stereovision module
        float cur_dist = _stereo_vision.get_distance();

        // desired distance (m)
        float d_dist = float(_tracking_distance) / 100.0f;

        // get forward command from distance controller
        _pos_control.update_dist_controller(forward_out, cur_dist, d_dist, dt, update_target);

        _perform_att_ctrl = abs(d_dist - cur_dist) < 10.0f;
    }
}

void AP_NetTracking::update_heading_control()
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

    // the target values will be ignored, if no new stereo vision data received (_sensor_updates.stv_updated = false)
    // this will update the target attitude corresponding to the current errors and trigger the attitude controller
    _attitude_control.input_euler_roll_pitch_yaw_accumulate(target_roll, target_pitch_error, target_yaw_error, _sensor_intervals.stv_dt, _sensor_updates.stv_updated);
}

void AP_NetTracking::update_throttle_out(float &throttle_out)
{
    // constantly moving downwards
    throttle_out = - _throttle_speed;

    if (_sensor_updates.stv_updated)
        update_phase_shift();

    if (fabs(_phase_shift_sum_y - _initial_phase_shift_sumy) > _phase_shift_thr_dist)
    {
        _initial_yaw = _attitude_control.get_accumulated_yaw();
        _state = State::Scanning;
    }

}

void AP_NetTracking::update_phase_shift()
{
    // update phase_shift input
    Vector2f phase_shift = _stereo_vision.get_acc_transl_shift();

    // lowpass filter
    _phase_shift_filt.set_cutoff_frequency(_phase_shift_cutoff_freq);
    // append negative value, because x-axes (and z-axes) of coordinate systems used in phase correlation node and ardusub are pointing in opposite directions
    _phase_shift_filt.apply(-phase_shift, _sensor_intervals.stv_dt);

    _phase_shift_sum_x = _phase_shift_filt.get().x;
    _phase_shift_sum_y = _phase_shift_filt.get().y;

    gcs().send_named_float("im_shift_x", _phase_shift_filt.get().x);
    gcs().send_named_float("im_shift_y", _phase_shift_filt.get().y);
}

void AP_NetTracking::update_loop_progress()
{
    //progress in percent (elapsed angle / 360 degrees ' 100)
    float tmp_loop_progress = fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) / 3.6f;

    // only update if it has increased since last run
    if (tmp_loop_progress > _loop_progress)
        _loop_progress = tmp_loop_progress;

    // constrain
    constrain_float(_loop_progress, 0.0f, 100.0f);
}


void AP_NetTracking::set_return_home()
{
    // don't do anything if ROV already performs return home routine
    if (_state == State::ReturnToHomeHeading || _state == State::ReturnToHomeAltitude) return;

    // in case of a closed net shape (fish farm), ROV continues scanning until initial heading reached, then throttles to initial altitude
    // in case of plane net shape, ROV directly throttles to initial altitude
    _state = _net_shape == NetShape::Tube ? State::ReturnToHomeHeading : State::ReturnToHomeAltitude;
}

