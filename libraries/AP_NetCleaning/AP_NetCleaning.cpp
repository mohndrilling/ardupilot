#include "AP_NetCleaning.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NetCleaning::var_info[] = {

    // @Param: NET_DST
    // @DisplayName: Distance of the AUV when aligning to the net
    // @Description: Distance of the AUV when aligning to the net
    // @Units: cm
    // @Range: 10.0 100.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NET_DST", 0, AP_NetCleaning, _init_net_dist, AP_NETCLEANING_INITIAL_NET_DISTANCE_DEFAULT),

    // @Param: DST_TOL
    // @DisplayName: Tolerance of the AUV's distance to the net
    // @Description: Tolerance of the AUV's distance to the net
    // @Units: cm
    // @Range: 0.0 50.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DST_TOL", 1, AP_NetCleaning, _init_net_dist_tolerance, AP_NETCLEANING_INITIAL_NET_DISTANCE_TOLERANCE_DEFAULT),

    // @Param: APPR_THR
    // @DisplayName: Throttle thrust when approaching net
    // @Description: Throttle thrust when approaching net
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("APPR_THR", 2, AP_NetCleaning, _approach_thr_thrust, AP_NETCLEANING_APPROACHING_THROTTLE_THRUST_DEFAULT),

    // @Param: CLEAN_THR
    // @DisplayName: Throttle thrust when cleaning net
    // @Description: Throttle thrust when cleaning net
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("CLEAN_THR", 3, AP_NetCleaning, _cleaning_thr_thrust, AP_NETCLEANING_CLEANING_THROTTLE_THRUST_DEFAULT),

    // @Param: APPR_FORW
    // @DisplayName: Forward thrust when detecting net
    // @Description: Forward thrust when detecting net
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("APPR_FORW", 4, AP_NetCleaning, _detect_net_forw_trust, AP_NETCLEANING_DETECTING_NET_FORWARD_THRUST_DEFAULT),

    // @Param: CLEAN_FORW
    // @DisplayName: Forward thrust when cleaning net
    // @Description: Forward thrust when cleaning net
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("CLEAN_FORW", 5, AP_NetCleaning, _cleaning_forw_thrust, AP_NETCLEANING_CLEANING_FORWARD_THRUST_DEFAULT),

    // @Param: LANE_WIDTH
    // @DisplayName: Lane width between two cleaning levels
    // @Description: Lane width between two cleaning levels
    // @Units: cm
    // @Range: 10.0 100.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LANE_WIDTH", 6, AP_NetCleaning, _lane_width, AP_NETCLEANING_LANE_WIDTH_DEFAULT),

    // @Param: STRT_DEPTH
    // @DisplayName: Altitude at which the net cleaning starts
    // @Description: Altitude at which the net cleaning starts
    // @Units: cm
    // @Range: 10.0 1000.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("STRT_DEPTH", 7, AP_NetCleaning, _start_cleaning_altitude, AP_NETCLEANING_START_CLEANING_DEPTH_DEFAULT),

    // @Param: END_DEPTH
    // @DisplayName: Altitude at which the net cleaning ends
    // @Description: Altitude at which the net cleaning ends
    // @Units: cm
    // @Range: 10.0 3000.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("END_DEPTH", 8, AP_NetCleaning, _finish_cleaning_altitude, AP_NETCLEANING_FINISH_CLEANING_DEPTH_DEFAULT),

    // @Param: CLIMB_RATE
    // @DisplayName: Climbing rate when changing altitudes in cm/s
    // @Description: Climbing rate when changing altitudes in cm/s
    // @Units: cm/s
    // @Range: 0.0 100.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CLIMB_RATE", 9, AP_NetCleaning, _climb_rate, AP_NETCLEANING_CLIMBING_RATE_CMS_DEFAULT),

    // @Param: ROT_DUR
    // @DisplayName: Duration of rotational trajectory when aligning to net
    // @Description: Duration of rotational trajectory when aligning to net
    // @Units: s
    // @Range: 0.0 100.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ROT_DUR", 10, AP_NetCleaning, _rot_traj_duration, AP_NETCLEANING_ROT_TRAJECTORY_DURATION_DEFAULT),

    // @Param: THR_DUR
    // @DisplayName: Duration of altitude trajectory when switching altitude levels
    // @Description: Duration of altitude trajectory when switching altitude levels
    // @Units: s
    // @Range: 0.0 100.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_DUR", 11, AP_NetCleaning, _alt_traj_duration, AP_NETCLEANING_ALT_TRAJECTORY_DURATION_DEFAULT),

    // @Param: CLEAN_CW
    // @DisplayName: Whether to clean in clockwise or counterclockwise direction
    // @Description: Whether to clean in clockwise or counterclockwise direction
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("CLEAN_CW", 12, AP_NetCleaning, _clean_clockwise, AP_NETCLEANING_CLEANING_CLOCKWISE_DEFAULT),

    AP_GROUPEND
};

void AP_NetCleaning::reset()
{
    // resetting routine
}

void AP_NetCleaning::init()
{
    // update time stamps
    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();

    // set initial state
    _current_state = State::AdjustedByOperator;
    _prev_state = State::Inactive;
    _first_run = true;

    // miscellaneous initial parameters
    _terminate = false;
    _brush_motors_active = false;
    _state_logic_finished = false;
    _last_state_execution_ms = 0;
    _state_logic_finished_ms = 0;
    _loop_progress = -1;

}

void AP_NetCleaning::run(float &forward_out, float &lateral_out, float &throttle_out)
{

    // set _first_run flag, checked by state logics to perform entry action
    if (_prev_state != _current_state)
    {
        _first_run = true;
        _prev_state = _current_state;
    }

    // run state logic
    switch (_current_state)
    {
        case State::AdjustedByOperator:
            adjusted_by_operator();
            break;

        case State::ApproachingInitialAltitude:
            approach_initial_altitude();
            break;

        case State::DetectingNetInitially:
            detect_net_initially();
            break;

        case State::HoldingNetDistance:
            hold_net_distance();
            break;

        case State::AligningVertical:
            align_vertical();
            break;

        case State::StartingBrushMotors:
            start_brush_motors();
            break;

        case State::ApproachingNet:
            approach_net();
            break;

        case State::AttachingBrushes:
            attach_brushes();
            break;

        case State::CleaningNet:
            clean_net();
            break;

        case State::ThrottleDownwards:
            throttle_downwards();
            break;

        case State::DetachingFromNet:
            detach_from_net();
            break;

        case State::StoppingBrushMotors:
            stop_brush_motors();
            break;

        case State::AligningHorizontal:
            align_horizontal();
            break;

        case State::DetectingNetTerminally:
            detect_net_terminally();
            break;

        case State::Surfacing:
            surface();
            break;

        case State::WaitingAtTerminal:
            wait_at_terminal();
            break;

        default:

            break;

    }

    // write the target thrusts to be calculated by the top level flight mode logic
    forward_out = _forward_out;
    lateral_out = _lateral_out;
    throttle_out = _throttle_out;

    // store time stamp of this loop
    _last_state_execution_ms = AP_HAL::millis();

    // reset first_run flag
    _first_run = false;
}

void AP_NetCleaning::adjusted_by_operator()
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
    // switch to next state after post delay. The post delay defines the time, during which the operator can adjust the vehicle's heading and position
    if (!_state_logic_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::ApproachingInitialAltitude, "ApproachingInitialAltitude", AP_NETCLEANING_ADJUSTED_BY_OPERATOR_POST_DELAY);
}

void AP_NetCleaning::approach_initial_altitude()
{
    // run attitude controller to hold horizontal attitude
    _attitude_control.keep_current_attitude();

    // translational movement  (forward, lateral, throttle)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // whether the starting altitude for net cleaning is reached
    bool target_alt_reached;

    if (_start_cleaning_altitude > 0 && !_state_logic_finished)
    {
        float dt = (AP_HAL::millis() - _last_state_execution_ms) / 1000.0f;

        target_alt_reached = _pos_control.climb_to_target_altitude(-_start_cleaning_altitude, -_climb_rate, dt, false);
    }
    else
    {
        // if the start cleaning altitude is set to zero, it is ignored and net cleaning started right away
        target_alt_reached = true;
    }

    /////////////// State Transition ////////////////
    if (!_state_logic_finished && target_alt_reached)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::DetectingNetInitially, "DetectingNetInitially", AP_NETCLEANING_APPROACHING_INIT_ALTITUDE_POST_DELAY);
}

void AP_NetCleaning::detect_net_initially()
{
    detect_net();

    if (_state_logic_finished)
        switch_state(State::HoldingNetDistance, "HoldingNetDistance");
}

void AP_NetCleaning::hold_net_distance()
{
    // entry action
    if (_first_run)
    {
        // store the home altitude at the very start of net cleaning
        _home_altitude = _inav.get_altitude();
    }

    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _init_net_dist towards net
    hold_heading_and_distance(_init_net_dist);

    /////////////// State Transition ////////////////
    if (_stereo_vision.stereo_vision_healthy())
    {
        // current distance to the net
        float cur_dist = _stereo_vision.get_distance();

        // desired distance (m)
        float d_dist = float(_init_net_dist) / 100.0f;

        // check whether task is finished
        if (!_state_logic_finished && fabs(cur_dist - d_dist) < _init_net_dist_tolerance / 100.0f)
        {
            set_state_logic_finished();
        }
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::AligningVertical, "AligningVertical", AP_NETCLEANING_HOLDING_NET_DISTANCE_POST_DELAY);
}

void AP_NetCleaning::align_vertical()
{
    // entry action
    if (_first_run)
    {
        // relative rotation: 90 degrees about x axis, 90 degrees about z axis, ypr-sequence
        // values in centidegrees
        Vector3f target_euler_angles_cd = Vector3f(9000.0f, 0.0f, 9000.0f);
        if (_clean_clockwise != AP_NETCLEANING_CLEAN_CLOCKWISE)
            target_euler_angles_cd *= -1.0f;

        uint32_t duration_ms = static_cast<uint32_t>(_rot_traj_duration * 1000.0f);
        _attitude_control.start_trajectory(target_euler_angles_cd, duration_ms, true);
    }

    // perform rotational trajectory, update_trajectory returns true if trajectory has finished
    bool trajectory_finished = _attitude_control.update_trajectory();

    // no translational movement
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    /////////////// State Transition ////////////////
    if (!_state_logic_finished && trajectory_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::StartingBrushMotors, "StartingBrushMotors", AP_NETCLEANING_ALIGNING_VERTICAL_POST_DELAY);
}

void AP_NetCleaning::start_brush_motors()
{
    // run attitude controller to hold horizontal attitude
    _attitude_control.keep_current_attitude();

    // no translational movement  (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // Todo: Start the motors. If the motors are supposed to be started and stopped by the companion computer
    // the state information about the net cleaning states can be used to trigger (see ArduSub/GCS_Mavlink.cpp - send_netcleaning_state)
    _brush_motors_active = true;

    /////////////// State Transition ////////////////
    // switch to next state after post delay
    if (!_state_logic_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::ApproachingNet, "ApproachingNet", AP_NETCLEANING_STARTING_BRUSH_MOTORS_POST_DELAY);
}

void AP_NetCleaning::approach_net()
{
    // run attitude controller, keep current attitude
    _attitude_control.keep_current_attitude();

    // translational movement  (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, -_approach_thr_thrust);

    /////////////// State Transition ////////////////
    // directly set state logic to finished and go into post delay
    // later use the motor currents to detect, if the AUV has attached to the net
    if (!_state_logic_finished)
    {
        set_state_logic_finished();
    }

    // the AUV will continue throttling towards the net and states are switched when the post delay is elapsed
    if (_state_logic_finished)
        switch_state_after_post_delay(State::AttachingBrushes, "AttachingBrushes", AP_NETCLEANING_APPROACHING_NET_POST_DELAY);
}

void AP_NetCleaning::attach_brushes()
{
    // run net cleaning attitude control
    run_net_cleaning_attitude_control();

    // translational movement  (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, -_cleaning_thr_thrust);

    /////////////// State Transition ////////////////
    // directly set state logic to finished and go into post delay
    if (!_state_logic_finished)
    {
        set_state_logic_finished();
    }

    // this state will continue until post delay elapsed
    if (_state_logic_finished)
        switch_state_after_post_delay(State::CleaningNet, "CleaningNet", AP_NETCLEANING_ATTACHING_BRUSHES_POST_DELAY);
}


void AP_NetCleaning::clean_net()
{
    // entry action
    if (_first_run)
    {
        // store initial yaw angle
        _initial_yaw = _attitude_control.get_accumulated_yaw();
    }

    // run net cleaning attitude control
    run_net_cleaning_attitude_control();

    // translational movement  (forward, lateral, throttle)
    set_translational_thrust(_cleaning_forw_thrust, 0.0f, -_cleaning_thr_thrust);

    /////////////// State Transition ////////////////
    // finish state after performing 360 degree loop
    if (!_state_logic_finished && detect_loop_closure())
    {
        set_state_logic_finished();
    }

    // apply post delay and set _forward_out negative in order to make the AUV decelerate its forwards movement
    // Todo: The forward movement should be controlled by using camera-based ego motion estimation
    if (_state_logic_finished){
        _forward_out = -_cleaning_forw_thrust;;
        if (_terminate)
            switch_state_after_post_delay(State::DetachingFromNet, "DetachingFromNet", AP_NETCLEANING_CLEANING_NET_POST_DELAY);
        else
            switch_state_after_post_delay(State::ThrottleDownwards, "ThrottleDownwards", AP_NETCLEANING_CLEANING_NET_POST_DELAY);
    }
}

void AP_NetCleaning::throttle_downwards()
{
    // entry action
    if (_first_run)
    {
        // start polynomial altitude trajectory moving the vehicle to a deeper lane

        uint32_t duration_ms = static_cast<uint32_t>(_alt_traj_duration * 1000.0f);
        float cur_altitude = _inav.get_altitude();

        if (cur_altitude < _lane_width - _finish_cleaning_altitude)
        {
            // set absolute altitude target to finish_cleaning_depth
            _pos_control.start_altitude_trajectory(static_cast<float>(-_finish_cleaning_altitude), duration_ms, false);
        }
        else
        {
            // move to the next deeper lane
            _pos_control.start_altitude_trajectory(static_cast<float>(-_lane_width), duration_ms, true);
        }

        // if the lane after the next one is smaller than 20 % of the default lane width, terminate after next cleaning loop already
        if (cur_altitude < 1.2 * _lane_width - _finish_cleaning_altitude)
            _terminate = true;
    }

    // follow depth trajectory, update_trajectory returns true if trajectory has finished
    bool trajectory_finished = _pos_control.update_altitude_trajectory();

    // run net cleaning attitude control
    run_net_cleaning_attitude_control();

    // translational movement (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, -_cleaning_thr_thrust);

    /////////////// State Transition ////////////////
    if (!_state_logic_finished && trajectory_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::CleaningNet, "CleaningNet", AP_NETCLEANING_THROTTLE_DOWNWARDS_POST_DELAY);
}

void AP_NetCleaning::detach_from_net()
{
    // run attitude controller, keep current attitude
    _attitude_control.keep_current_attitude();

    // translational movement(forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, _approach_thr_thrust);

    /////////////// State Transition ////////////////
    // directly set state logic to finished and go into post delay
    if (!_state_logic_finished)
    {
        set_state_logic_finished();
    }

    // the AUV will continue throttling away from the net and states are switched when the post delay is elapsed
    // the post delay/ throttle should be chosen such that the AUV can move sufficiently far away from the net
    // in order to have enough space to rotate back into horizontal orientation
    if (_state_logic_finished)
        switch_state_after_post_delay(State::StoppingBrushMotors, "StoppingBrushMotors", AP_NETCLEANING_DETACHING_FROM_NET_POST_DELAY);
}

void AP_NetCleaning::stop_brush_motors()
{
    // run attitude controller to hold horizontal attitude
    _attitude_control.keep_current_attitude();

    // no translational movement (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // Todo: Stop the motors. If the motors are supposed to be started and stopped by the companion computer
    // the state information about the net cleaning states can be used to trigger (see ArduSub/GCS_Mavlink.cpp - send_netcleaning_state)
    _brush_motors_active = false;

    /////////////// State Transition ////////////////
    // switch to next state after post delay.
    if (!_state_logic_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::AligningHorizontal, "AligningHorizontal", AP_NETCLEANING_STOPPING_BRUSH_MOTORS_POST_DELAY);
}

void AP_NetCleaning::align_horizontal()
{
    // entry action
    if (_first_run)
    {
        // relative rotation: -90 degrees about x axis, -90 degrees about y axis, ypr-sequence, exact opponent trajectory of 'AligningToNet' state
        // values in centidegrees
        Vector3f target_euler_angles_cd = Vector3f(-9000.0f, -9000.0f, 0.0f);
        if (_clean_clockwise != AP_NETCLEANING_CLEAN_CLOCKWISE)
            target_euler_angles_cd *= -1.0f;
        uint32_t duration_ms = static_cast<uint32_t>(_rot_traj_duration * 1000.0f);
        _attitude_control.start_trajectory(target_euler_angles_cd, duration_ms, true);
    }

    // perform rotational trajectory, update_trajectory returns true if trajectory has finished
    bool trajectory_finished = _attitude_control.update_trajectory();

    // no translational movement (forward, lateral, throttle)
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    /////////////// State Transition ////////////////
    if (!_state_logic_finished && trajectory_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::DetectingNetTerminally, "DetectingNetTerminally", AP_NETCLEANING_ALIGNING_HORIZONTAL_POST_DELAY);
}

void AP_NetCleaning::detect_net_terminally()
{
    detect_net();

    if (_state_logic_finished)
        switch_state(State::Surfacing, "Surfacing");
}

void AP_NetCleaning::surface()
{
    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _init_net_dist towards net
    hold_heading_and_distance(_init_net_dist);    

    // ascend
    float dt = (AP_HAL::millis() - _last_state_execution_ms) / 1000.0f;

   bool target_alt_reached = _pos_control.climb_to_target_altitude(_home_altitude, _climb_rate, dt, false);

    /////////////// State Transition ////////////////
    if (target_alt_reached)
    {
        switch_state(State::WaitingAtTerminal, "WaitingAtTerminal");
    }
}

void AP_NetCleaning::wait_at_terminal()
{    
    // translational movement (forward, lateral, throttle) (forward_out is overwritten by hold_heading_and_distance)
    // todo: use optical flow stabilization for lateral velocity
    set_translational_thrust(0.0f, 0.0f, 0.0f);

    // hold perpendicular heading with regard to the net and hold _init_net_dist towards net
    hold_heading_and_distance(_init_net_dist);
}

void AP_NetCleaning::detect_net()
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
        float d_dist = float(_init_net_dist) / 100.0f;

        // switch state if distance to the net is close to or smaller than desired distance
        if (cur_dist - d_dist < _init_net_dist_tolerance / 100.0f)
            set_state_logic_finished();
    }
}

void AP_NetCleaning::hold_heading_and_distance(float target_dist)
{
    /////////////////////////////////////////////////////////
    // check and update stereo vision module

    if (!_stereo_vision.stereo_vision_healthy())
    {
        _attitude_control.keep_current_attitude();
        return;
    }

    // time difference (in seconds) between two measurements from stereo vision is used to lowpass filter the data
    _sensor_intervals.stv_dt = _stereo_vision.get_stv_time_delta_usec() / 1000000.0f; // stereo vision net tracking messages

    if (_sensor_intervals.stv_dt > 2.0f)
    {
        _attitude_control.reset_yaw_err_filter();
    }

    // only update target distance and attitude, if new measurement from stereo data available
    _sensor_updates.stv_updated = _stereo_vision.get_last_stv_update_ms() - _last_stereo_update_ms != 0;

    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();

    /////////////////////////////////////////////////////////////
    // attitude control, hold perpendicular heading with regard to net

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

void AP_NetCleaning::run_net_cleaning_attitude_control()
{
    // keep the vehicle's x-axis levelled so the vehicle will move in horizontal lanes
    _attitude_control.keep_nose_horizontal();

    // by relaxing the roll and pitch control of the vehicle, it's attitude can be freely manipulated about the x- and y-axis.
    // This way it will be evenly pushed towards the net, and the roll and pitch orientation will be passively guided by the net
    _attitude_control.relax_roll_control();
    _attitude_control.relax_pitch_control();
}

void AP_NetCleaning::switch_state(State target_state, const char *state_name)
{
    _current_state = target_state;
    _state_logic_finished = false;
    gcs().send_text(MAV_SEVERITY_INFO, "[NetCleaning] Switch state: '%s'", state_name);
}

void AP_NetCleaning::switch_state_after_post_delay(State target_state, const char *state_name, uint32_t post_delay)
{
    if (AP_HAL::millis() - _state_logic_finished_ms > post_delay)
        switch_state(target_state, state_name);
}

void AP_NetCleaning::set_state_logic_finished()
{
    _state_logic_finished = true;
    _state_logic_finished_ms = AP_HAL::millis();
}

bool AP_NetCleaning::detect_loop_closure()
{
    // detect loop closure by elapsed yaw angle (prone to measurement drifts)
    return fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) > 360.0f;
}

void AP_NetCleaning::update_loop_progress()
{
    //progress in percent (elapsed angle / 360 degrees ' 100)
    float tmp_loop_progress = fabs(_attitude_control.get_accumulated_yaw() - _initial_yaw) / 3.6f;

    // only update if it has increased since last run
    if (tmp_loop_progress > _loop_progress)
        _loop_progress = tmp_loop_progress;

    // constrain
    constrain_float(_loop_progress, 0.0f, 100.0f);
}
