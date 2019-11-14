#pragma once

#include "AC_PosControl.h"

// distance controller default definitions
# define POSCONTROL_DIST_P                    0.6f    // distance controller P gain default
# define POSCONTROL_DIST_I                    6.0f    // distance controller I gain default
# define POSCONTROL_DIST_D                    0.0f    // distance controller D gain default
# define POSCONTROL_DIST_IMAX                 0.01f   // distance controller IMAX gain default
# define POSCONTROL_DIST_FILT_HZ              10.0f   // distance controller input filter default
# define POSCONTROL_DIST_DT                   0.01f   // distance controller dt default
# define POSCONTROL_DIST_PMAX                 0.3f   // distance controller PMAX gain default

// mesh count controller default definitions
# define POSCONTROL_MESH_CNT_P                    0.02f   // mesh controller P gain default
# define POSCONTROL_MESH_CNT_I                    0.5f    // mesh controller I gain default
# define POSCONTROL_MESH_CNT_D                    0.0f    // mesh controller D gain default
# define POSCONTROL_MESH_CNT_IMAX                 0.001f  // mesh controller IMAX gain default
# define POSCONTROL_MESH_CNT_FILT_HZ              10.0f   // mesh controller input filter default
# define POSCONTROL_MESH_CNT_DT                   0.01f   // mesh controller dt default
# define POSCONTROL_MESH_CNT_PMAX                 0.08f   // mesh controller PMAX gain default

// optfl lateral controller default definitions
# define POSCONTROL_OPTFL_P                    0.0f   // opt flow controller P gain default
# define POSCONTROL_OPTFL_I                    10.0f    // opt flow controller I gain default
# define POSCONTROL_OPTFL_D                    0.0f    // opt flow controller D gain default
# define POSCONTROL_OPTFL_IMAX                 50.0f  // opt flow controller IMAX gain default
# define POSCONTROL_OPTFL_FILT_HZ              0.0f   // opt flow controller input filter default
# define POSCONTROL_OPTFL_DT                   0.01f   // opt flow controller dt default
# define POSCONTROL_OPTFL_PMAX                 20.0f   // opt flow controller PMAX gain default

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

    /// control distance to net
    void update_dist_controller(float& target_forward, float distance_error, float dt, bool update);

    /// control currently visible net meshes
    void update_mesh_cnt_controller(float& target_forward, float mesh_cnt_error, float dt, bool update);

    /// control lateral velocity based on optical flow input error
    void update_optfl_controller(float& target_lateral, float optfl_error, float dt, bool update);

    /// reset the integrator of the optical flow controller
    void relax_optfl_controller() { _pid_optfl.reset_I(); }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:
    float       _alt_max; // max altitude - should be updated from the main code with altitude limit from fence
    float       _alt_min; // min altitude - should be updated from the main code with altitude limit from fence

    float       _net_track_dist; // desired distance of vehicle to tracked net in meters

    AC_PID      _pid_dist; // distance controller

    AC_PID      _pid_mesh_cnt; // mesh_cnt controller

    AC_PID      _pid_optfl; // opt_flow controller


};
