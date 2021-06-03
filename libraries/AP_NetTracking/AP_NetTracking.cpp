#include "AP_NetTracking.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NetTracking::var_info[] = {

    // @Param: NET_SHAPE
    // @DisplayName: Defines the shape of the tracked net (plane or tube)
    // @Description: Defines the shape of the tracked net (plane or tube)
    // @Values: 0:Planar 1:FishFarm
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

    // @Param: DST_TOL
    // @DisplayName: Tolerance of the AUV's distance to the net
    // @Description: Tolerance of the AUV's distance to the net
    // @Units: cm
    // @Range: 0.0 50.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DST_TOL", 2, AP_NetTracking, _tracking_distance_tolerance, AP_NETTRACKING_INITIAL_NET_DISTANCE_TOLERANCE_DEFAULT),

    // @Param: MESH_CNT
    // @DisplayName: Desired count of visible net meshes during net tracking
    // @Description: Desired count of visible net meshes during net tracking
    // @Range: 100 800
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MESH_CNT", 3, AP_NetTracking, _tracking_meshcount, AP_NETTRACKING_MESH_CNT_DEFAULT),

    // @Param: VEL
    // @DisplayName: Desired lateral velocity during net tracking in percent
    // @Description: Desired lateral velocity during net tracking in percent
    // @Units: percent
    // @Range: -100.0 100.0
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("VEL", 4, AP_NetTracking, _tracking_velocity, AP_NETTRACKING_VELOCITY_DEFAULT),

    // @Param: CTRL_VAR
    // @DisplayName: Whether to control the distance to the net or the amount of visible net meshes
    // @Description: Whether to control the distance to the net or the amount of visible net meshes
    // @Values: 0:Distance 1:MeshCount
    // @User: Advanced
    AP_GROUPINFO("CTRL_VAR", 5, AP_NetTracking, _control_var, AP_NETTRACKING_CTRL_VAR_DEFAULT),

    // @Param: VEL_CTRL
    // @DisplayName: Whether to use optical flow input to control the lateral velocity of the vehicle
    // @Description: Whether to use optical flow input to control the lateral velocity of the vehicle
    // @Values: 0:Disabled 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("VEL_CTRL", 6, AP_NetTracking, _velocity_ctrl, AP_NETTRACKING_VEL_CTRL_DEFAULT),

    // @Param: OPTFL_HZ
    // @DisplayName: Cut off frequency for low pass filter of optical flow input
    // @Description: Cut off frequency for low pass filter of optical flow input
    // @Units: Hz
    // @Range: 0.0 10.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("OPTFL_HZ", 7, AP_NetTracking, _opt_flow_cutoff_freq, AP_NETTRACKING_OPT_FLOW_CUTOFF_FREQ_DEFAULT),

    // @Param: OPTFL_VDST
    // @DisplayName: By how many pixels the ROV is supposed to move vertically to the next scanning level
    // @Description: By how many pixels the ROV is supposed to move vertically to the next scanning level
    // @Range: 10.0 600.0
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("OPTFL_VDST", 8, AP_NetTracking, _opt_flow_vertical_dist, AP_NETTRACKING_OPT_FLOW_VERTICAL_DIST_DEFAULT),

    // @Param: OPT_MARKER
    // @DisplayName: Whether to use a tape with optical markers to detect 360 degree loops
    // @Description: Whether to use a tape with optical markers to detect 360 degree loops
    // @Values: 0:Disabled 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("OPT_MARKER", 9, AP_NetTracking, _use_optical_marker_termination, AP_NETTRACKING_USE_OPT_MARKER_DEFAULT),

    // @Param: STRT_DEPTH
    // @DisplayName: Altitude at which the net tracking starts
    // @Description: Altitude at which the net tracking starts
    // @Units: cm
    // @Range: 10.0 1000.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STRT_DEPTH", 10, AP_NetTracking, _start_tracking_depth, AP_NETTRACKING_START_TRACKING_DEPTH_DEFAULT),

    // @Param: END_DEPTH
    // @DisplayName: Altitude at which the net tracking ends
    // @Description: Altitude at which the net tracking ends
    // @Units: cm
    // @Range: 10.0 3000.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("END_DEPTH", 11, AP_NetTracking, _finish_tracking_depth, AP_NETTRACKING_FINISH_TRACKING_DEPTH_DEFAULT),

    // @Param: APPR_FORW
    // @DisplayName: Forward thrust when detecting net
    // @Description: Forward thrust when detecting net
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("APPR_FORW", 12, AP_NetTracking, _detect_net_forw_trust, AP_NETTRACKING_DETECTING_NET_FORWARD_THRUST_DEFAULT),

    // @Param: CLIMB_RATE
    // @DisplayName: Climbing rate when changing altitudes in cm/s
    // @Description: Climbing rate when changing altitudes in cm/s
    // @Units: cm/s
    // @Range: 0.0 100.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CLIMB_RATE", 13, AP_NetTracking, _climb_rate, AP_NETTRACKING_CLIMBING_RATE_CMS_DEFAULT),

    // @Param: T_ADJUST
    // @DisplayName: Time (s) for operator to manually adjust AUV heading once in water
    // @Description: Time (s) for operator to manually adjust AUV heading once in water
    // @Units: s
    // @Range: 0.0 100.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("T_ADJUST", 14, AP_NetTracking, _manual_adjustment_duration, AP_NETTRACKING_ADJUSTED_BY_OPERATOR_POST_DELAY),

    AP_GROUPEND
};

void AP_NetTracking::reset()
{
    // update time stamps
    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();
    _last_mesh_data_update_ms = _stereo_vision.get_last_ni_update_ms();
    _last_opt_flow_update_ms = _stereo_vision.get_last_of_update_ms();
    _last_marker_detection_update_ms = _stereo_vision.get_last_md_update_ms();

    // process variables
    _net_detect_yaw_rate_dir = 0.0f;
    _nettr_toggle_velocity = false;
    _nettr_direction = 1.0f;
    _loop_progress = -1.0f;
    _terminate = false;

    // reset filters
    _opt_flow_filt.reset();
    _net_detect_yaw_filt.reset();
    _net_detect_yaw_rate_filt.reset();

    // set initial state (must be called after setup)
    _current_state = _states[StateID::AutoLevel];
    _prev_state = _states[StateID::Inactive];
    _first_run = true;
}

void AP_NetTracking::init()
{
    // create the states of the state machines
    setup_state_machines();

    // set default values
    reset();
}

void AP_NetTracking::setup_state_machines()
{
    add_state(new State(StateID::Inactive, "Inactive",&AP_NetTracking::inactive, 0, StateID::Inactive));    

    add_state(new State(StateID::AutoLevel, "AutoLevel",&AP_NetTracking::auto_level,
                        AP_NETTRACKING_AUTO_LEVEL_POST_DELAY, StateID::AdjustedByOperator));

    add_state(new State(StateID::AdjustedByOperator, "AdjustedByOperator",&AP_NetTracking::adjusted_by_operator,
                        _manual_adjustment_duration*1000.0f, StateID::ApproachingInitialAltitude));

    add_state(new State(StateID::ApproachingInitialAltitude, "ApproachingInitialAltitude",&AP_NetTracking::approach_initial_altitude,
                        AP_NETTRACKING_APPROACHING_INIT_ALTITUDE_POST_DELAY, StateID::DetectingNetInitially));

    add_state(new State(StateID::DetectingNetInitially, "DetectingNetInitially",&AP_NetTracking::detect_net,
                        AP_NETTRACKING_DETECTING_NET_POST_DELAY, StateID::ApproachingNetInitially));

    add_state(new State(StateID::ApproachingNetInitially, "ApproachingNetInitially",&AP_NetTracking::approach_net,
                        AP_NETTRACKING_APPROACHING_NET_POST_DELAY, StateID::HoldingNetDistance));

    add_state(new State(StateID::HoldingNetDistance, "HoldingNetDistance",&AP_NetTracking::hold_net_distance,
                        AP_NETTRACKING_HOLDING_NET_DISTANCE_POST_DELAY, StateID::Scanning));

    add_state(new State(StateID::Scanning, "Scanning",&AP_NetTracking::scan,
                        AP_NETTRACKING_SCANNING_POST_DELAY, StateID::ThrottleDownwards, StateID::Surfacing));

    add_state(new State(StateID::ThrottleDownwards, "ThrottleDownwards",&AP_NetTracking::throttle_downwards,
                        AP_NETTRACKING_THROTTLE_DOWNWARDS_POST_DELAY, StateID::Scanning));

    add_state(new State(StateID::Surfacing, "Surfacing",&AP_NetTracking::surface,
                        AP_NETTRACKING_SURFACING_POST_DELAY, StateID::WaitingAtTerminal));

    add_state(new State(StateID::WaitingAtTerminal, "WaitingAtTerminal",&AP_NetTracking::wait_at_terminal,
                        AP_NETTRACKING_SURFACING_POST_DELAY, StateID::WaitingAtTerminal));
}

void AP_NetTracking::run(float &forward_out, float &lateral_out, float &throttle_out)
{
    // if the state machine is not set up properly, set translational thrust output to zero and keep attitude
    if (_current_state == nullptr)
    {
        forward_out = 0.0f;
        lateral_out = 0.0f;
        throttle_out = 0.0f;

        _attitude_control.keep_current_attitude();

        return;
    }

    // update stereo vision and optical flow
    update_stereo_vision();
    update_opt_flow();

    // set _first_run flag, checked by state logics to perform entry action
    if (_prev_state->_id != _current_state->_id)
    {
        _first_run = true;
        _prev_state = _current_state;
        _first_state_execution_ms = AP_HAL::millis();
    }

    // run state logic
    StateLogicFunction run_current_state_logic = _current_state->_state_logic_func;
    (this->*run_current_state_logic)();

    // check transition
    if (_state_logic_finished)
        switch_state_after_post_delay();

    // write the target thrusts, they will be applied by the top level flight mode logic
    forward_out = _forward_out;
    lateral_out = _lateral_out;
    throttle_out = _throttle_out;

    // store time stamp of this loop
    _last_state_execution_ms = AP_HAL::millis();

    // reset first_run flag
    _first_run = false;
}


void AP_NetTracking::auto_level()
{
    // entry action
    if (_first_run)
    {
        Vector3f target_euler_angles_cd = Vector3f(0.0f, 0.0f, RadiansToCentiDegrees(_ahrs.get_current_yaw()));
        _attitude_control.start_trajectory(target_euler_angles_cd, 0, false);
    }

    // perform rotational trajectory, update_trajectory returns true if trajectory has finished
    bool trajectory_finished = _attitude_control.update_trajectory();

    // no translational movement
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    /////////////// State Transition ////////////////
    if (trajectory_finished)
        set_state_logic_finished();
}

void AP_NetTracking::adjusted_by_operator()
{
    // entry action
    if (_first_run)
    {
        // set horizontal target attitude
        _attitude_control.set_levelled_target_attitude();
    }

    // run attitude controller to hold horizontal attitude
    _attitude_control.keep_current_attitude();

    // release yaw controller so operator can align the AUV's heading properly
    _attitude_control.relax_yaw_control();

    // no translational movement (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    /////////////// State Transition ////////////////
    // directly set state logic finished to run the post delay.
    // The post delay defines the time, during which the operator can adjust the vehicle's heading and position
    // state is switched after post delay is elapsed
    set_state_logic_finished();

}

void AP_NetTracking::approach_initial_altitude()
{
    // entry action
    if (_first_run)
    {
        // set horizontal target attitude
        _attitude_control.set_levelled_target_attitude();
    }

    // run attitude controller to hold horizontal attitude
    _attitude_control.keep_current_attitude();

    // translational movement  (forward, lateral, throttle)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // whether the starting depth for net tracking is reached
    bool target_alt_reached;

    if (_start_tracking_depth > 0 && !_state_logic_finished)
    {
        float dt = (AP_HAL::millis() - _last_state_execution_ms) / 1000.0f;
        target_alt_reached = _pos_control.climb_to_target_altitude(-_start_tracking_depth, -_climb_rate, dt, false);
    }
    else
    {
        // if the start tracking depth is set to zero, it is ignored and net tracking started right away
        target_alt_reached = true;
    }

    /////////////// State Transition ////////////////
    if (target_alt_reached)
        set_state_logic_finished();
}

void AP_NetTracking::detect_net()
{
    // no translational movement (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    /////////////// State Transition ////////////////
    float dt = float(AP_HAL::millis() - _first_state_execution_ms);
    float yaw_rate = 1500.0f;
    float delta_yaw = 0.0f;

    if (_use_optical_marker_termination && _stereo_vision.marker_visible())
    {
        set_state_logic_finished();
    }
    else if (_stereo_vision.stereo_vision_healthy())
    {
        // relative yaw in degrees with regard to detected netfloat
        delta_yaw = _net_detect_yaw_filt.apply(_stereo_vision.get_delta_yaw(), dt);
        yaw_rate = constrain_float(0.25f * fabs(delta_yaw) - 250.0f, 500.0f, 1500.0f);
        // consider net detected if the yaw error towards the net gets smaller than certain threshold (centidegrees!)
        if (fabs(delta_yaw) < 100.0f)
        {
            set_state_logic_finished();
        }
    }


    if (is_zero(_net_detect_yaw_rate_dir) && AP_HAL::millis() - _first_state_execution_ms > 1000)
        _net_detect_yaw_rate_dir = is_positive(delta_yaw) ? -1.0f : 1.0f;


    yaw_rate = _net_detect_yaw_rate_filt.apply(_net_detect_yaw_rate_dir * yaw_rate, dt);

    if (_state_logic_finished)
    {
        _attitude_control.reset_target_attitude();
        yaw_rate = 0.0f;
    }

    _attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, yaw_rate);
}

void AP_NetTracking::approach_net()
{
    // run attitude controller, keep current attitude
    _attitude_control.keep_current_attitude();

    // no translational movement (forward, lateral, throttle)
    set_translational_thrust(_detect_net_forw_trust, 0.0f, 0.0f);

    /////////////// State Transition ////////////////

    if (_stereo_vision.stereo_vision_healthy())
    {
        // current distance to the net
        float cur_dist = _stereo_vision.get_distance();

        // desired distance (m)
        float d_dist = float(_tracking_distance) / 100.0f;

        // switch state if distance to the net is close to or smaller than desired distance
        if (cur_dist - d_dist < _tracking_distance_tolerance / 100.0f)
            set_state_logic_finished();
    }
}

void AP_NetTracking::hold_net_distance()
{
    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _tracking_distance towards net
    hold_heading_and_distance(_tracking_distance);

    /////////////// State Transition ////////////////
    if (_stereo_vision.stereo_vision_healthy())
    {
        // current distance to the net
        float cur_dist = _stereo_vision.get_distance();

        // desired distance (m)
        float d_dist = float(_tracking_distance) / 100.0f;

        // check whether task is finished
        if (fabs(cur_dist - d_dist) < _tracking_distance_tolerance / 100.0f)
        {
            set_state_logic_finished();
            _home_yaw = _ahrs.get_current_yaw();
            _home_altitude = _inav.get_altitude();
        }
    }
}

void AP_NetTracking::scan()
{
    if (_first_run)
    {
        _initial_yaw = _attitude_control.get_accumulated_yaw();
        _current_state->_next_state = _terminate ? _current_state->_next_stateB : _current_state->_next_stateA;
    }

    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _tracking_distance towards net
    hold_heading_and_distance(_tracking_distance);

    // update lateral out
    if (_state_logic_finished)
    {
        if (_stereo_vision.stereo_vision_healthy() && fabs(_stereo_vision.get_delta_yaw()) < 100.0f)
            update_lateral_out(0.0f);
        return;
    }
    else
    {
        float ellapsed_time = float(AP_HAL::millis() - _first_state_execution_ms);
        float accel_duration = 3000.0f;
        float target_vel = constrain_float(_tracking_velocity / accel_duration * ellapsed_time, 0.0f, _tracking_velocity);
        target_vel = _state_logic_finished ? 0.0f : target_vel;
        update_lateral_out(target_vel);
    }

    // if the net shape equals an open plane, perform net edge detection based on the mesh distribution on the current image
    // if net edge detected, reverse the lateral output velocity
    if (_net_shape == NetShape::Planar)
    {
        float mesh_distr = _stereo_vision.get_mesh_distr();
        _nettr_toggle_velocity = _nettr_toggle_velocity || fabs(mesh_distr - 0.5f) < 0.1f;
        float distr_thr = 0.15f;
        bool net_edge_reached = mesh_distr < distr_thr || mesh_distr > (1.0f - distr_thr);
        if (!(_nettr_toggle_velocity && net_edge_reached)) return;
        _nettr_toggle_velocity = false;
    }
    else if (_net_shape == NetShape::FishFarm)
    {
        update_loop_progress();

        // check if ROV has performed 360 degrees of scanning
        // debugging output
        if (AP_HAL::millis() - _last_debug_dyaw_ms > 200)
        {
            gcs().send_named_float("dyaw", degrees(fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw)));
            _last_debug_dyaw_ms = AP_HAL::millis();
        }

        if (!detect_loop_closure()) return

        // reset yaw error low pass filter to avoid overshooting
        _attitude_control.reset_yaw_err_filter();

        // reset loop progress
        _loop_progress = -1;
    }

    _nettr_direction *= -1.0f;
    _initial_opt_flow_sumy = _opt_flow_sum_y;
    set_state_logic_finished();

    gcs().send_text(MAV_SEVERITY_INFO, "Loop Finished");

}

void AP_NetTracking::throttle_downwards()
{
    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _tracking_distance towards net
    hold_heading_and_distance(_tracking_distance);    

    float doptfl_fac = 1.0f - constrain_float(fabs(_opt_flow_sum_y - _initial_opt_flow_sumy) / _opt_flow_vertical_dist, 0.0f, 1.0f);
    float doptfl_throttle_start = 0.3f;
    float climb_rate = _climb_rate * constrain_float(doptfl_fac / doptfl_throttle_start, 0.0f, 1.0f);
    climb_rate = _state_logic_finished ? 0.0f : constrain_float(climb_rate, 5.0f, _climb_rate);
    float dt = (AP_HAL::millis() - _last_state_execution_ms) / 1000.0f;
    _pos_control.set_alt_target_from_climb_rate(-climb_rate, dt, false);

    if (!_state_logic_finished && fabs(_opt_flow_sum_y - _initial_opt_flow_sumy) > _opt_flow_vertical_dist)
    {
        _terminate = _use_optical_marker_termination ? _stereo_vision.marker_terminate() : _inav.get_altitude() <= -_finish_tracking_depth;
        set_state_logic_finished();
    }
}

void AP_NetTracking::surface()
{
    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _tracking_distance towards net
    hold_heading_and_distance(_tracking_distance);

    // ascend
    float dt = (AP_HAL::millis() - _last_state_execution_ms) / 1000.0f;

   bool target_alt_reached = _pos_control.climb_to_target_altitude(_home_altitude, _climb_rate, dt, false);

    /////////////// State Transition ////////////////
    if (target_alt_reached)
        set_state_logic_finished();
}

void AP_NetTracking::wait_at_terminal()
{
    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _tracking_distance towards net
    hold_heading_and_distance(_tracking_distance);
}

void AP_NetTracking::update_lateral_out(float target_vel)
{
    // perform optical flow stabilization if enabled and measurements arrived
    if (_velocity_ctrl)
    {
        if (!_stereo_vision.opt_flow_healthy())
        {
            _lateral_out = 0.0f;
            return;
        }
        // negate the derivation since ardusub and optical flow coordinate systems are flipped (z-axis in opposite directions)
        float cur_optfl_x = -_stereo_vision.get_cur_transl_shift().x / _sensor_intervals.of_dt;

        float target_optfl_x = _nettr_direction * target_vel * _nettr_opt_flow_vel_factor;
        _pos_control.update_optflx_controller(_lateral_out, cur_optfl_x, target_optfl_x, _sensor_intervals.of_dt, _sensor_updates.of_updated);

    }
    else
    {
        _lateral_out = _nettr_direction * target_vel * _nettr_default_vel_factor;
    }

    _lateral_out /= 250.0f;
}

void AP_NetTracking::hold_heading_and_distance(float target_dist)
{
    /////////////////////////////////////////////////////////
    // check and update stereo vision module

    if (!_stereo_vision.stereo_vision_healthy())
    {
        _attitude_control.keep_current_attitude();
        return;
    }

    /////////////////////////////////////////////////////////////
    // attitude control, hold perpendicular heading with regard to net

    if (_sensor_intervals.stv_dt > 2.0f)
    {
        _attitude_control.reset_yaw_err_filter();
    }

    // no roll desired
    float target_roll = 0.0f;

    // get pitch and yaw offset (in centidegrees) with regard to the faced object (net) in front
    float target_pitch_error = _stereo_vision.get_delta_pitch();
    float target_yaw_error = _stereo_vision.get_delta_yaw();

    _attitude_control.input_euler_roll_pitch_yaw_accumulate(target_roll, target_pitch_error, target_yaw_error, _sensor_intervals.stv_dt, _sensor_updates.stv_updated);


    ////////////////////////////////////////////////////////////
    // distance control

    // current distance to the net
    float cur_dist = _stereo_vision.get_distance();

    // desired distance (m)
    float d_dist = float(target_dist) / 100.0f;

    // get forward command from distance controller
    _pos_control.update_dist_controller(_forward_out, cur_dist, d_dist, _sensor_intervals.stv_dt, _sensor_updates.stv_updated);

    // constrain forward velocity in order to prevent rapid movement when the distance controller activates
    // todo: check for any issues regarding integral windup (integrator of distance pid controller keeps charging while forward output is saturated)
    // this may lead to overshooting (windup effect) but shouldn't be a problem here as the integral term of the pid controller is usually chosen to be relatively small
    _forward_out = constrain_float(_forward_out, -_detect_net_forw_trust, _detect_net_forw_trust);
}

bool AP_NetTracking::detect_loop_closure()
{
    if (_use_optical_marker_termination)
    {
        // a visible marker indicates, that the ROV is at the starting (home) position of each loop
        // This function shall only return true if the ROV has performed a full scanning loop.
        // As markers are still visible right after starting a loop, we check for the yaw angle of the ROV
        // of having exceeded a certain minimum angle since start of the last loop
        if(fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) < radians(40.0f))
            return false;

        else
            return _stereo_vision.marker_visible();
    }
    else
    {
        // detect loop closure by elapsed yaw angle (prone to measurement drifts)
        return fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) > radians(20.0f);
    }
}

void AP_NetTracking::update_opt_flow()
{
    // update opt_flow input
    Vector2f opt_flow = _stereo_vision.get_acc_transl_shift();

    // lowpass filter
    _opt_flow_filt.set_cutoff_frequency(_opt_flow_cutoff_freq);
    // append negative value, because x-axes (and z-axes) of coordinate systems used in optical flow node and ardusub are pointing in opposite directions
    _opt_flow_filt.apply(-opt_flow, _sensor_intervals.stv_dt);

    _opt_flow_sum_x = _opt_flow_filt.get().x;
    _opt_flow_sum_y = _opt_flow_filt.get().y;

    // debugging output
    if (AP_HAL::millis() - _last_debug_opt_flow_ms > 200)
    {
        gcs().send_named_float("im_shift_x", _opt_flow_filt.get().x);
        gcs().send_named_float("im_shift_y", _opt_flow_filt.get().y);
        _last_debug_opt_flow_ms = AP_HAL::millis();
    }
}

void AP_NetTracking::update_loop_progress()
{
    //progress in percent (elapsed angle / 2pi * 100)
    float tmp_loop_progress = fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) * 50.0f / M_PI;

    // only update if it has increased since last run
    if (tmp_loop_progress > _loop_progress)
        _loop_progress = tmp_loop_progress;

    // constrain
    constrain_float(_loop_progress, 0.0f, 100.0f);
}

void AP_NetTracking::set_return_home()
{
    _terminate = true;
    gcs().send_text(MAV_SEVERITY_INFO, "Returning to Home");
}

void AP_NetTracking::switch_state_after_post_delay()
{
    if (AP_HAL::millis() - _state_logic_finished_ms > _current_state->_post_delay)
    {
        _current_state = _states[_current_state->_next_state];
        _state_logic_finished = false;

        if (_current_state != nullptr)
            gcs().send_text(MAV_SEVERITY_INFO, "[NetTracking] Switch to state '%s'", _current_state->_name);
    }
}

void AP_NetTracking::set_state_logic_finished()
{
    if (_state_logic_finished) return;

    _state_logic_finished = true;
    _state_logic_finished_ms = AP_HAL::millis();
}

void AP_NetTracking::update_stereo_vision()
{
    // time difference (in seconds) between two measurements from stereo vision is used to lowpass filter the data
    _sensor_intervals.stv_dt = _stereo_vision.get_stv_time_delta_usec() / 1000000.0f; // stereo vision net tracking messages
    _sensor_intervals.ni_dt = _stereo_vision.get_ni_time_delta_usec() / 1000000.0f; // net inspection  messages
    _sensor_intervals.of_dt = _stereo_vision.get_of_time_delta_usec() / 1000000.0f; // optical flow messages
    _sensor_intervals.md_dt = _stereo_vision.get_md_time_delta_usec() / 1000000.0f; // marker detection messages

    // only update target distance and attitude, if new measurement from stereo data available
    _sensor_updates.stv_updated = _stereo_vision.get_last_stv_update_ms() - _last_stereo_update_ms != 0;
    _sensor_updates.ni_updated = _stereo_vision.get_last_ni_update_ms() - _last_mesh_data_update_ms != 0;
    _sensor_updates.of_updated = _stereo_vision.get_last_of_update_ms() - _last_opt_flow_update_ms != 0;
    _sensor_updates.md_updated = _stereo_vision.get_last_md_update_ms() - _last_marker_detection_update_ms != 0;

    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();
    _last_mesh_data_update_ms = _stereo_vision.get_last_ni_update_ms();
    _last_opt_flow_update_ms = _stereo_vision.get_last_of_update_ms();
    _last_marker_detection_update_ms = _stereo_vision.get_last_md_update_ms();
}

