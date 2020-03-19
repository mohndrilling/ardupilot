#include "AP_NetCleaning.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NetCleaning::var_info[] = {

    // @Param: INIT_DIST
    // @DisplayName: Distance of the AUV when aligning to the net
    // @Description: Distance of the AUV when aligning to the net
    // @Units: cm
    // @Range: 10.0 100.0
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("INIT_DIST", 0, AP_NetCleaning, _initial_net_distance, AP_NETCLEANING_INITIAL_NET_DISTANCE_DEFAULT),

    AP_GROUPEND
};

void AP_NetCleaning::reset()
{
    // resetting routine
}

void AP_NetCleaning::init()
{
    _initial_yaw = _attitude_control.get_accumulated_yaw();

    // home position (defined by heading and altitude for now)
    _home_yaw = _ahrs.get_current_yaw();
    _home_altitude = _inav.get_altitude();

    // update time stamps
    _last_stereo_update_ms = _stereo_vision.get_last_stv_update_ms();

    // set initial state
    _current_state = State::ApproachingNet;
}

void AP_NetCleaning::run(float &forward_out, float &lateral_out, float &throttle_out)
{


    switch (_current_state)
    {
        case State::ApproachingNet:
            approach_net(forward_out, lateral_out, throttle_out);
            break;

        case State::AligningToNet:
            align_to_net(forward_out, lateral_out, throttle_out);
            break;

        case State::AttachingToNet:
            attach_to_net(forward_out, lateral_out, throttle_out);
            break;

        case State::AttachingBrushes:
            attach_brushes(forward_out, lateral_out, throttle_out);
            break;

        case State::CleaningNet:
            clean_net(forward_out, lateral_out, throttle_out);
            break;

        case State::ThrottleDownwards:
            throttle_downwards(forward_out, lateral_out, throttle_out);
            break;

        default:

            break;

    }

}

void AP_NetCleaning::approach_net(float &forward_out, float &lateral_out, float &throttle_out)
{
    // hold perpendicular heading with regard to the net and hold _initial_net_distance towards net
    hold_heading_and_distance(forward_out, _initial_net_distance);

    // no lateral movement, (todo: use optical flow stabilization)
    lateral_out = 0.0f;

    // no throttle
    throttle_out = 0.0f;

    /////////////// State Transition ////////////////
    if (_state_logic_finished)
        switch_state_after_post_delay(State::AligningToNet, "AligningToNet", AP_NETCLEANING_APPROACHING_NET_POST_DELAY);
}

void AP_NetCleaning::align_to_net(float &forward_out, float &lateral_out, float &throttle_out)
{
    // entry action
    if (_prev_state != _current_state)
    {
        // relative rotation: 90 degrees about x axis, 90 degrees about y axis, rpy-sequence
        // values in centidegrees
        Vector3f target_euler_angles_cd = Vector3f(9000.0f, 9000.0f, 0.0f);
        uint32_t duration_ms = 6000;
        _attitude_control.start_trajectory(target_euler_angles_cd, duration_ms, true);
        _prev_state = _current_state;
    }

    // perform rotational trajectory, update_trajectory returns true if trajectory has finished
    bool trajectory_finished = _attitude_control.update_trajectory();

    // no translational movement
    forward_out = 0.0f;
    lateral_out = 0.0f;
    throttle_out = 0.0f;

    /////////////// State Transition ////////////////
    if (!_state_logic_finished && trajectory_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::AttachingToNet, "AttachingToNet", AP_NETCLEANING_ALIGNING_TO_NET_POST_DELAY);
}

void AP_NetCleaning::attach_to_net(float &forward_out, float &lateral_out, float &throttle_out)
{
    // run attitude controller, keep current attitude
    _attitude_control.keep_current_attitude();

    // translational movement
    forward_out = 0.0f;
    lateral_out = 0.0f;
    throttle_out = -AP_NETCLEANING_THROTTLE_THRUST_DEFAULT;

    /////////////// State Transition ////////////////
    // directly set state logic to finished and go into post delay
    // later use the motor currents to detect, if the AUV has attached to the net
    if (!_state_logic_finished)
    {
        set_state_logic_finished();
    }

    // the AUV will continue throttling towards the net and states are switched when the post delay is elapsed
    if (_state_logic_finished)
        switch_state_after_post_delay(State::AttachingBrushes, "AttachingBrushes", AP_NETCLEANING_ATTACHING_TO_NET_POST_DELAY);
}

void AP_NetCleaning::attach_brushes(float &forward_out, float &lateral_out, float &throttle_out)
{
    // run net cleaning attitude control
    run_net_cleaning_attitude_control();

    // translational movement
    forward_out = 0.0f;
    lateral_out = 0.0f;
    throttle_out = -AP_NETCLEANING_THROTTLE_THRUST_DEFAULT;

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


void AP_NetCleaning::clean_net(float &forward_out, float &lateral_out, float &throttle_out)
{
    // entry action
    if (_prev_state != _current_state)
    {
        // store initial yaw angle
        _initial_yaw = _attitude_control.get_accumulated_yaw();
        _prev_state = _current_state;
    }

    // run net cleaning attitude control
    run_net_cleaning_attitude_control();

    // translational movement
    forward_out = AP_NETCLEANING_FORWARD_THRUST_DEFAULT;
    lateral_out = 0.0f;
    throttle_out = -AP_NETCLEANING_THROTTLE_THRUST_DEFAULT;

    /////////////// State Transition ////////////////
    // finish state after performing 360 degree loop
    if (!_state_logic_finished && detect_loop_closure())
    {
        set_state_logic_finished();
    }

    // apply post delay and set forward_out to zero in order to make the AUV decelerate its forwards movement
    if (_state_logic_finished){
        forward_out = 0.0f;
        switch_state_after_post_delay(State::ThrottleDownwards, "ThrottleDownwards", AP_NETCLEANING_CLEANING_NET_POST_DELAY);
    }
}

void AP_NetCleaning::throttle_downwards(float &forward_out, float &lateral_out, float &throttle_out)
{
    // entry action
    if (_prev_state != _current_state)
    {
        // start polynomial altitude trajectory moving the vehicle to a deeper lane
        uint32_t duration_ms = 10000;
        _pos_control.start_altitude_trajectory(-AP_NETCLEANING_LANE_WIDTH_DEFAULT, duration_ms, true);
        _prev_state = _current_state;
    }

    // follow depth trajectory, update_trajectory returns true if trajectory has finished
    bool trajectory_finished = _pos_control.update_altitude_trajectory();

    // run net cleaning attitude control
    run_net_cleaning_attitude_control();

    // translational movement
    forward_out = 0.0f;
    lateral_out = 0.0f;
    throttle_out = -AP_NETCLEANING_THROTTLE_THRUST_DEFAULT;

    /////////////// State Transition ////////////////
    if (!_state_logic_finished && trajectory_finished)
    {
        set_state_logic_finished();
    }

    if (_state_logic_finished)
        switch_state_after_post_delay(State::CleaningNet, "CleaningNet", AP_NETCLEANING_THROTTLE_DOWNWARDS_POST_DELAY);
}

void AP_NetCleaning::hold_heading_and_distance(float &forward_out, float target_dist)
{
    /////////////////////////////////////////////////////////
    // check and update stereo vision module

    if (!_stereo_vision.stereo_vision_healthy())
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "NetCleaning: No STEREO_VISION messages received.");
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
    _pos_control.update_dist_controller(forward_out, cur_dist, d_dist, _sensor_intervals.stv_dt, _sensor_updates.stv_updated);

    /////////////////////////////////////////////////////////////
    // check whether task is finished
    if (!_state_logic_finished && fabs(cur_dist - d_dist) < _dist_tolerance)
    {
        set_state_logic_finished();
    }
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
    _prev_state = _current_state;
    _current_state = target_state;
    _state_logic_finished = false;
    gcs().send_text(MAV_SEVERITY_INFO, "NetCleaning: Switching to state '%s'", state_name);
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
