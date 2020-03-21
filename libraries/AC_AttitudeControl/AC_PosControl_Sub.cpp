#include "AC_PosControl_Sub.h"
#include "AP_StereoVision/AP_StereoVision.h"

// table of user settable parameters
const AP_Param::GroupInfo AC_PosControl_Sub::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_PosControl, 0),

    // @Param: VELDST_P
    // @DisplayName: Distance derivative controller P gain
    // @Description: Distance derivative controller P gain.
    // @Range: 0.0 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: VELDST_I
    // @DisplayName: Distance derivative controller I gain
    // @Description: Distance derivative controller I gain.
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: VELDST_IMAX
    // @DisplayName: Distance derivative controller I gain maximum
    // @Description: Distance derivative controller I gain maximum.
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: VELDST_D
    // @DisplayName: Distance derivative controller D gain
    // @Description: Distance derivative controller D gain.
    // @Range: 0.0 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELDST_FF
    // @DisplayName: Distance derivative controller feed forward
    // @Description: Distance derivative controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: VELDST_FILT
    // @DisplayName: Distance derivative controller input cutoff frequency in Hz
    // @Description: Distance derivative controller input cutoff frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_vel_dist, "_VELDST_", 1, AC_PosControl_Sub, AC_PID),

    // @Param: _POSDST_P
    // @DisplayName: Distance controller P gain
    // @Description: Distance controller P gain.  Converts distance error to target velocity
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_dist, "_POSDST_", 2, AC_PosControl_Sub, AC_P),

    // @Param: _DDST_FILT
    // @DisplayName: Low pass filter of the velocity w.r.t. the fishing net during net tracking
    // @Description: Low pass filter of the velocity w.r.t. the fishing net during net tracking
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_DDST_FILT", 3, AC_PosControl_Sub, _dist_vel_filter_cutoff, POSCONTROL_DIST_VEL_FILTER_HZ),

    // @Param: _DST_LEASH
    // @DisplayName: Maximum value of the distance error during net tracking
    // @Description: Maximum value of the distance error during net tracking
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_DST_LEASH", 4, AC_PosControl_Sub, _leash_dist, POSCONTROL_DIST_LEASH_LENGTH),

    // @Param: MSH_VEL_P
    // @DisplayName: Mesh count velocity controller P gain
    // @Description: Mesh count velocity controller P gain.
    // @Range: 0.0 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: MSH_VEL_I
    // @DisplayName: Mesh count velocity controller I gain
    // @Description: Mesh count velocity controller I gain.
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: MSH_VEL_IMAX
    // @DisplayName: Mesh count velocity controller I gain maximum
    // @Description: Mesh count velocity controller I gain maximum.
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: MSH_VEL_D
    // @DisplayName: Mesh count velocity controller D gain
    // @Description: Mesh count velocity controller D gain.
    // @Range: 0.0 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: MSH_VEL_FF
    // @DisplayName: Mesh count velocity controller feed forward
    // @Description: Mesh count velocity controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: MSH_VEL_FILT
    // @DisplayName: Mesh count velocity controller input frequency in Hz
    // @Description: Mesh count velocity controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_mesh_vel, "_MSH_VEL_", 5, AC_PosControl_Sub, AC_PID),

    // @Param: _MSH_CNT_P
    // @DisplayName: Distance controller P gain
    // @Description: Distance controller P gain.  Converts mesh count error to target velocity
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_mesh_cnt, "_MSH_CNT_", 6, AC_PosControl_Sub, AC_P),

    // @Param: _DMSH_FILT
    // @DisplayName: Low pass filter of the derivation of the current mesh count
    // @Description: Low pass filter of the derivation of the current mesh count
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_DMSH_FILT", 7, AC_PosControl_Sub, _mesh_cnt_vel_filter_cutoff, POSCONTROL_MESH_CNT_DERIVATION_FILTER_HZ),

    // @Param: _MSH_LEASH
    // @DisplayName: Maximum value of the mesh count error during net tracking
    // @Description: Maximum value of the mesh count error during net tracking
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_MSH_LEASH", 8, AC_PosControl_Sub, _leash_mesh_cnt, POSCONTROL_MESH_CNT_LEASH_LENGTH),

    // @Param: OPTFLX_P
    // @DisplayName: Optical flow controller P gain
    // @Description: Optical flow controller P gain.
    // @Range: 0.0 0.30
    // @Increment: 0.005
    // @User: Standard

    // @Param: OPTFLX_I
    // @DisplayName: Optical flow controller I gain
    // @Description: Optical flow controller I gain.
    // @Range: 0.0 0.5
    // @Increment: 0.01
    // @User: Standard

    // @Param: OPTFLX_IMAX
    // @DisplayName: Optical flow controller I gain maximum
    // @Description: Optical flow controller I gain maximum.
    // @Range: 0 1
    // @Increment: 0.01
    // @Units: %
    // @User: Standard

    // @Param: OPTFLX_D
    // @DisplayName: Optical flow  controller D gain
    // @Description: Optical flow controller D gain.
    // @Range: 0.0 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: OPTFLX_FF
    // @DisplayName: Optical flow controller feed forward
    // @Description: Optical flow controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: OPTFLX_FILT
    // @DisplayName: Optical flow controller input frequency in Hz
    // @Description: Optical flow controller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_optflx, "_OPTFLX_", 9, AC_PosControl_Sub, AC_PID),

    AP_GROUPEND
};

AC_PosControl_Sub::AC_PosControl_Sub(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                                     AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    AC_PosControl(ahrs, inav, motors, attitude_control),
    _alt_max(0.0f),
    _alt_min(0.0f),
    _alt_target_reached(false),
    _dist_last(0.0f),
    _pid_vel_dist(POSCONTROL_DIST_VEL_P, POSCONTROL_DIST_VEL_I, POSCONTROL_DIST_VEL_D, POSCONTROL_DIST_VEL_IMAX, POSCONTROL_DIST_VEL_FILT_HZ, POSCONTROL_DIST_VEL_DT),
    _p_pos_dist(POSCONTROL_DIST_P),
    _mesh_cnt_last(0.0f),
    _pid_mesh_vel(POSCONTROL_MESH_CNT_VEL_P, POSCONTROL_MESH_CNT_VEL_I, POSCONTROL_MESH_CNT_VEL_D, POSCONTROL_MESH_CNT_VEL_IMAX, POSCONTROL_MESH_CNT_VEL_FILT_HZ, POSCONTROL_MESH_CNT_VEL_DT),
    _p_mesh_cnt(POSCONTROL_MESH_CNT_P),
    _pid_optflx(POSCONTROL_OPTFLX_P, POSCONTROL_OPTFLX_I, POSCONTROL_OPTFLX_D, POSCONTROL_OPTFLX_IMAX, POSCONTROL_OPTFLX_FILT_HZ, POSCONTROL_OPTFLX_DT)
{}


bool AC_PosControl_Sub::climb_to_target_altitude(float target_alt, float climb_rate_cms, float dt, bool force_descend)
{
    // assure correct sign of climbing rate
    float cur_alt = _inav.get_altitude();
    bool ascending = target_alt > cur_alt;
    if (    ascending && climb_rate_cms < 0.0f
        || !ascending && climb_rate_cms > 0.0f)
    {
        climb_rate_cms *= -1.0f;
    }

    // update altitude target
    set_alt_target_from_climb_rate(climb_rate_cms, dt, force_descend);

    if (    ascending && _pos_target.z > target_alt
        || !ascending && _pos_target.z < target_alt)
    {
        // note, this is also set to true, if the altitude reaches one of the limits alt_max and alt_min
        _alt_target_reached = true;
    }

    return _alt_target_reached;
}

/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl_Sub::set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend)
{
    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((climb_rate_cms<0 && (!_motors.limit.throttle_lower || force_descend)) || (climb_rate_cms>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += climb_rate_cms * dt;
    }

    // flag to store whether one of the upper or lower altitude limits has been reached
    _alt_target_reached = false;

    // do not let target alt get above limit
    if (_alt_max < 100 && _pos_target.z > _alt_max) {
        _pos_target.z = _alt_max;
        _alt_target_reached = true;
        _limit.pos_up = true;
    }

    // do not let target alt get below limit
    if (_alt_min < 0 && _alt_min < _alt_max && _pos_target.z < _alt_min) {
        _pos_target.z = _alt_min;
        _alt_target_reached = true;
        _limit.pos_down = true;
    }

    // do not use z-axis desired velocity feed forward
    // vel_desired set to desired climb rate for reporting and land-detector
    _flags.use_desvel_ff_z = false;
    _vel_desired.z = climb_rate_cms;
}

/// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
///     set force_descend to true during landing to allow target to move low enough to slow the motors
void AC_PosControl_Sub::set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend)
{
    // calculated increased maximum acceleration if over speed
    float accel_z_cms = _accel_z_cms;
    if (_vel_desired.z < _speed_down_cms && !is_zero(_speed_down_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_down_cms;
    }
    if (_vel_desired.z > _speed_up_cms && !is_zero(_speed_up_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_up_cms;
    }
    accel_z_cms = constrain_float(accel_z_cms, 0.0f, 750.0f);

    // jerk_z is calculated to reach full acceleration in 1000ms.
    float jerk_z = accel_z_cms * POSCONTROL_JERK_RATIO;

    float accel_z_max = MIN(accel_z_cms, safe_sqrt(2.0f*fabsf(_vel_desired.z - climb_rate_cms)*jerk_z));

    _accel_last_z_cms += jerk_z * dt;
    _accel_last_z_cms = MIN(accel_z_max, _accel_last_z_cms);

    float vel_change_limit = _accel_last_z_cms * dt;
    _vel_desired.z = constrain_float(climb_rate_cms, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
    _flags.use_desvel_ff_z = true;

    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((_vel_desired.z<0 && (!_motors.limit.throttle_lower || force_descend)) || (_vel_desired.z>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += _vel_desired.z * dt;
    }

    // flag to store whether one of the upper or lower altitude limits has been reached
    _alt_target_reached = false;

    // do not let target alt get above limit
    if (_alt_max < 100 && _pos_target.z > _alt_max) {
        _pos_target.z = _alt_max;
        _limit.pos_up = true;
        _alt_target_reached = true;
        // decelerate feed forward to zero
        _vel_desired.z = constrain_float(0.0f, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
    }

    // do not let target alt get below limit
    if (_alt_min < 0 && _alt_min < _alt_max && _pos_target.z < _alt_min) {
        _pos_target.z = _alt_min;
        _alt_target_reached = true;
        _limit.pos_down = true;
        // decelerate feed forward to zero
        _vel_desired.z = constrain_float(0.0f, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
    }
}

/// relax_alt_hold_controllers - set all desired and targets to measured
void AC_PosControl_Sub::relax_alt_hold_controllers()
{
    // set altitude buffer;
    float alt_buffer = _inav.get_velocity_z() * _alt_brake_tc;
    _pos_target.z = _inav.get_altitude() + alt_buffer;
    _vel_desired.z = 0.0f;
    _flags.use_desvel_ff_z = false;
    _vel_target.z = _inav.get_velocity_z();
    _vel_last.z = _inav.get_velocity_z();
    _accel_desired.z = 0.0f;
    _accel_last_z_cms = 0.0f;
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _flags.reset_accel_to_throttle = true;
}


void AC_PosControl_Sub::start_altitude_trajectory(float target_altitude, uint32_t duration, bool relative)
{
    // store trajectory's starting altitude (current altitude)
    _trajectory_starting_altitude = _inav.get_altitude();

    // if relative is true, add target_altitude to current_altitude
    if (relative)
    {
        _trajectory_target_altitude = _inav.get_altitude() + target_altitude;
    }
    else
    {
        _trajectory_target_altitude = target_altitude;
    }

    // store duration
    _trajectory_altitude_duration_ms = duration;

    // store starting time stamp
    _trajectory_altitude_start_ms = AP_HAL::millis();
}

bool AC_PosControl_Sub::update_altitude_trajectory()
{
    // current time stamp
    float t = static_cast<float>(AP_HAL::millis() - _trajectory_altitude_start_ms);

    if (t <= _trajectory_altitude_duration_ms)
    {
        // get target altitude from 5th order polynomial trajectory (defined in AP_Math/polynomial5.h)
        float cur_target_altitude;
        polynomial_trajectory(cur_target_altitude, _trajectory_starting_altitude, _trajectory_target_altitude, _trajectory_altitude_duration_ms, t);

        _pos_target.z = cur_target_altitude;

        return false;
    }

    return true;
}

void AC_PosControl_Sub::update_dist_controller(float& target_forward, float cur_dist, float target_dist, float dt, bool update)
{
    // simple pid controller for distance control
    // todo: check if useful to use implemented xy-position controller
    float p, i, d;

    if (update)
    {
        // distance error
        float dist_error = target_dist - cur_dist;

        // leash
        dist_error = constrain_float(dist_error, -_leash_dist, _leash_dist);

        // target velocity
        // TODO: use square root controller
        // TODO: constrain target velocity?
        float vel_dist_target = _p_pos_dist.get_p(dist_error);

        // calculate relative velocity w.r.t the net
        _dist_vel_filter.set_cutoff_frequency(_dist_vel_filter_cutoff);
        float cur_vel_dist = _dist_vel_filter.apply((cur_dist - _dist_last) / dt, dt);
        _dist_last = cur_dist;

        // velocity error
        float vel_dist_error = vel_dist_target - cur_vel_dist;

        // update pid controller
        // negate pid input, as positive distance error requires negative motor input (backwards driving) and vice versa
        _pid_vel_dist.set_dt(dt);
        _pid_vel_dist.set_input_filter_all(- vel_dist_error);

        // debugging output
        gcs().send_named_float("d_dst", target_dist);
        gcs().send_named_float("dst", cur_dist);
        gcs().send_named_float("dstvel", cur_vel_dist);
        gcs().send_named_float("d_dstvel", vel_dist_target);
    }

    p = _pid_vel_dist.get_p();

    // get i term
    i = _pid_vel_dist.get_i();

    // get d term
    d = _pid_vel_dist.get_d();

    target_forward = p + i + d;

    // set the cutoff frequency of the motors forward input filter
    _motors.set_forward_filter_cutoff(POSCONTROL_FORWARD_CUTOFF_FREQ);
}

void AC_PosControl_Sub::update_mesh_cnt_controller(float& target_forward, float cur_mesh_cnt, float target_mesh_cnt, float dt, bool update)
{
    float p, i, d;

    if (update)
    {        
        // we control the square root of current mesh count, since total meshcount grows quadratically over the distance to the net
        // but we want a linear dependency between control input (forward throttle) and control variable (square root mesh count)
        // mesh count error
        float mesh_cnt_error = safe_sqrt(target_mesh_cnt) - safe_sqrt(cur_mesh_cnt);

        // leash
        mesh_cnt_error = constrain_float(mesh_cnt_error, -_leash_mesh_cnt, _leash_mesh_cnt);

        // target velocity
        // TODO: use square root controller
        // TODO: constrain target velocity?
        float mesh_cnt_vel_target = _p_mesh_cnt.get_p(mesh_cnt_error);

        // calculate derivation of mesh count course
        _mesh_cnt_vel_filter.set_cutoff_frequency(_mesh_cnt_vel_filter_cutoff);
        float cur_mesh_cnt_vel = _mesh_cnt_vel_filter.apply((safe_sqrt(cur_mesh_cnt) - safe_sqrt(_mesh_cnt_last)) / dt, dt);
        _mesh_cnt_last = cur_mesh_cnt;

        // velocity error
        float vel_dist_error = mesh_cnt_vel_target - cur_mesh_cnt_vel;

        // update pid controller
        // negate pid input, as positive distance error requires negative motor input (backwards driving) and vice versa
        _pid_mesh_vel.set_dt(dt);
        _pid_mesh_vel.set_input_filter_all(- vel_dist_error);

        // debugging output
        gcs().send_named_float("d_mshcnt", target_mesh_cnt);
        gcs().send_named_float("mshcnt", cur_mesh_cnt);
        gcs().send_named_float("mshvel", cur_mesh_cnt_vel);
        gcs().send_named_float("d_mshvel", mesh_cnt_vel_target);
    }

    p = _pid_mesh_vel.get_p();

    // get i term
    i = _pid_mesh_vel.get_i();

    // get d term
    d = _pid_mesh_vel.get_d();

    target_forward = p + i + d;

    // set the cutoff frequency of the motors forward input filter
    _motors.set_forward_filter_cutoff(POSCONTROL_FORWARD_CUTOFF_FREQ);
}


void AC_PosControl_Sub::update_optfl_controller(float& target_lateral, float cur_optflx, float target_optflx, float dt, bool update)
{
    //optical flow error
    float optflx_error = target_optflx - cur_optflx;

    // simple pid controller for optfl control
    float p, i, d;

    if (update)
    {
        _pid_optflx.set_dt(dt);
        _pid_optflx.set_input_filter_all(optflx_error);
    }

    // probably better to use square root constrain
    p = _pid_optflx.get_p();

    // get i term
    i = _pid_optflx.get_i();

    // get d term
    d = _pid_optflx.get_d();

    target_lateral = p + i + d;

    // set the cutoff frequency of the motors lateral input filter
    _motors.set_lateral_filter_cutoff(POSCONTROL_LATERAL_CUTOFF_FREQ);

    //debug output
    gcs().send_named_float("optflx", cur_optflx);
    gcs().send_named_float("d_optflx", target_optflx);
}
