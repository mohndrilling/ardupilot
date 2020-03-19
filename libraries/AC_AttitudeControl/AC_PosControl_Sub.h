#pragma once

#include "AC_PosControl.h"

// distance controller default definitions
# define POSCONTROL_DIST_VEL_P                    4.0f    // distance controller P gain default
# define POSCONTROL_DIST_VEL_I                    0.03f    // distance controller I gain default
# define POSCONTROL_DIST_VEL_IMAX                 0.05f   // distance controller IMAX gain default
# define POSCONTROL_DIST_VEL_D                    0.0f    // distance controller D gain default
# define POSCONTROL_DIST_VEL_FILT_HZ              10.0f   // distance controller input filter default
# define POSCONTROL_DIST_VEL_DT                   0.01f   // distance controller dt default
# define POSCONTROL_DIST_P                        0.3f    // distance controller distance to velocity gain

#define POSCONTROL_DIST_VEL_FILTER_HZ             2.0f // low pass filter cutoff frequency for first distance derivation (velocity)

#define POSCONTROL_DIST_LEASH_LENGTH              0.5f  // maximum distance error in m

// mesh count controller default definitions
# define POSCONTROL_MESH_CNT_VEL_P                0.04f   // mesh controller P gain default
# define POSCONTROL_MESH_CNT_VEL_I                0.1f    // mesh controller I gain default
# define POSCONTROL_MESH_CNT_VEL_D                0.0f    // mesh controller D gain default
# define POSCONTROL_MESH_CNT_VEL_IMAX             0.1f  // mesh controller IMAX gain default
# define POSCONTROL_MESH_CNT_VEL_FILT_HZ          10.0f   // mesh controller input filter default
# define POSCONTROL_MESH_CNT_VEL_DT               0.01f   // mesh controller dt default
# define POSCONTROL_MESH_CNT_P                    0.6f   // mesh controller mesh count to derivative gain

#define POSCONTROL_MESH_CNT_DERIVATION_FILTER_HZ  2.0f // low pass filter cutoff frequency for first distance derivation (velocity)

#define POSCONTROL_MESH_CNT_LEASH_LENGTH          10.0f  // maximum distance error in m

// optfl lateral controller default definitions
# define POSCONTROL_OPTFLX_P                      1.0f   // opt flow controller P gain default
# define POSCONTROL_OPTFLX_I                      3.0f    // opt flow controller I gain default
# define POSCONTROL_OPTFLX_D                      0.0f    // opt flow controller D gain default
# define POSCONTROL_OPTFLX_IMAX                   15.0f  // opt flow controller IMAX gain default
# define POSCONTROL_OPTFLX_FILT_HZ                20.0f   // opt flow controller input filter default
# define POSCONTROL_OPTFLX_DT                     0.01f   // opt flow controller dt default

class AC_PosControl_Sub : public AC_PosControl {
public:
    AC_PosControl_Sub(const AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                      AP_Motors& motors, AC_AttitudeControl& attitude_control);

    /// set_alt_max - sets maximum altitude above the ekf origin in cm
    ///   only enforced when set_alt_target_from_climb_rate is used
    ///   set to zero to disable limit
    void set_alt_max(float alt) { _alt_max = alt; }

    /// set_alt_min - sets the minimum altitude (maximum depth) in cm
    ///   only enforced when set_alt_target_from_climb_rate is used
    ///   set to zero to disable limit
    void set_alt_min(float alt) { _alt_min = alt; }

    /// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    void set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend) override;

    /// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
    ///     should be called continuously (with dt set to be the expected time between calls)
    ///     actual position target will be moved no faster than the speed_down and speed_up
    ///     target will also be stopped if the motors hit their limits or leash length is exceeded
    ///     set force_descend to true during landing to allow target to move low enough to slow the motors
    void set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend) override;

    /// relax_alt_hold_controllers - set all desired and targets to measured
    void relax_alt_hold_controllers(float throttle_setting) {
        AC_PosControl::relax_alt_hold_controllers(throttle_setting);
    }

    /// relax_alt_hold_controllers - set all desired and targets to measured
    ///     integrator is not reset
    void relax_alt_hold_controllers();

    // start_altitude_trajectory: stores target altitude and duration for a altitude trajectory
    // if relative is true, the target_altitude value gets added tu the current altitude
    void start_altitude_trajectory(float target_altitude, uint32_t duration, bool relative);

    // updates trajectory's intermediate altitude
    // returns true if finished
    bool update_altitude_trajectory();

    /// control distance to net
    void update_dist_controller(float& target_forward, float cur_dist, float target_dist, float dt, bool update);

    /// control currently visible net meshes
    void update_mesh_cnt_controller(float& target_forward, float cur_mesh_cnt, float target_mesh_cnt, float dt, bool update);

    /// control lateral velocity based on optical flow input error
    void update_optfl_controller(float& target_lateral, float cur_optflx, float target_optflx, float dt, bool update);

    /// reset the integrator of the optical flow controller
    void relax_optfl_controller() { _pid_optflx.reset_I(); }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    float       _alt_max; // max altitude - should be updated from the main code with altitude limit from fence
    float       _alt_min; // min altitude - should be updated from the main code with altitude limit from fence

    // internal variables for altitude trajectory following
    float _trajectory_starting_altitude; // altitude at which the altitude trajectory starts
    float _trajectory_target_altitude; // altitude at which the altitude trajectory finishes

    uint32_t _trajectory_altitude_duration_ms; // duration of altitude trajectory

    uint32_t _trajectory_altitude_start_ms; // starting time stamp of altitude trajectory

    // position controller internal variables
    float       _dist_last;             // last distance measurement in m, needed for velocity calculation during nettracking
    AP_Float    _leash_dist;            // constrains distance error

    LowPassFilterFloat _dist_vel_filter;   // low-pass-filter on derivative of distance (relative velocity w.r.t. net)

    AP_Float _dist_vel_filter_cutoff;       // dist_vel filter cutoff frequency

    AC_PID      _pid_vel_dist; // distance controller
    AC_P        _p_pos_dist;     // dist_error to velocity gain

    // mesh count controller internal variables
    float       _mesh_cnt_last;             // last mesh count measurement, needed for derivation during nettracking
    AP_Float    _leash_mesh_cnt;            // constrains mesh count error

    LowPassFilterFloat _mesh_cnt_vel_filter;   // low-pass-filter on derivative of mesh count (proportional to relative velocity w.r.t. net)

    AP_Float _mesh_cnt_vel_filter_cutoff;       // mesh count velocity filter cutoff frequency

    AC_PID      _pid_mesh_vel; // mesh count controller
    AC_P        _p_mesh_cnt;   // gain between mesh count error and derivative mesh count

    // optfl controller internal variables
    AC_PID      _pid_optflx; // opt_flow pid controller

};
