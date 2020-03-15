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

    // perform rotational trajectory and switch state if finished
    if(_attitude_control.update_trajectory())
    {
        switch_state(State::AttachingToNet, "AttachingToNet");
    }

    // no translational movement
    forward_out = 0.0f;
    lateral_out = 0.0f;
    throttle_out = 0.0f;
}

void AP_NetCleaning::attach_to_net(float &forward_out, float &lateral_out, float &throttle_out)
{
    // run attitude controller, keep current attitude
    _attitude_control.keep_current_attitude();

    // no translational movement
    forward_out = 0.0f;
    lateral_out = 0.0f;
    throttle_out = 0.0f;
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
    // state transition
    if (!_target_dist_reached && fabs(cur_dist - d_dist) < _dist_tolerance)
    {
        _target_dist_reached_ms = AP_HAL::millis();
        _target_dist_reached = true;
    }

    if (_target_dist_reached && AP_HAL::millis() - _target_dist_reached_ms > _hold_dist_ms)
    {
        switch_state(State::AligningToNet, "AligningToNet");
    }
}

void AP_NetCleaning::switch_state(State target_state, const char *state_name)
{
    _prev_state = _current_state;
    _current_state = target_state;
    gcs().send_text(MAV_SEVERITY_INFO, "NetCleaning: Switching to state '%s'", state_name);
}

void AP_NetCleaning::switch_state_with_delay(uint32_t milliseconds, State target_state)
{

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
