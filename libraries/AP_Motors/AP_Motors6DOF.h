/// @file	AP_Motors6DOF.h
/// @brief	Motor control class for ROVs with direct control over 6DOF (or fewer) in movement

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"

/// @class      AP_MotorsMatrix
class AP_Motors6DOF : public AP_MotorsMatrix {
public:

    AP_Motors6DOF(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz) {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // Supported frame types
    typedef enum {
        SUB_FRAME_BLUEROV1,
        SUB_FRAME_VECTORED,
        SUB_FRAME_VECTORED_6DOF,
        SUB_FRAME_VECTORED_6DOF_90DEG,
        SUB_FRAME_SIMPLEROV_3,
        SUB_FRAME_SIMPLEROV_4,
        SUB_FRAME_SIMPLEROV_5,
        SUB_FRAME_CUSTOM
    } sub_frame_t;

    // Override parent
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // Override parent
    void output_min() override;

    void set_pilot_throttle(float pilot_throttle_in) { _pilot_throttle_in = -pilot_throttle_in; }

    // set vehicle attitude (to perform proper control allocation of the throttle commands)
    void set_vehicle_attitude(Matrix3f body_to_ned_rot_mat) { _vehicle_attitude = body_to_ned_rot_mat; }

    // set the coordinate frame, in which translational movement commands are supposed to be interpreted
    void set_control_frame(Matrix3f control_frame) { _control_frame = control_frame; }

    // Map thrust input -1~1 to pwm output 1100~1900
    int16_t calc_thrust_to_pwm(float thrust_in) const;

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

protected:
    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    float               get_current_limit_max_throttle() override;

    //Override MotorsMatrix method
    void add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float lat_fac, uint8_t testing_order);

    void output_armed_stabilizing() override;
    void output_armed_stabilizing_vectored();
    void output_armed_stabilizing_vectored_6dof();

    // Parameters
    AP_Int8             _motor_reverse[AP_MOTORS_MAX_NUM_MOTORS];
    AP_Float            _forwardVerticalCouplingFactor;

    // motor's contribution to linear velocity w.r.t. the body frame (static)
    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _lateral_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)

    // motor's contribution to the inertial z-axis (dynamic)
    float               _inertial_throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)

    // motor's contribution to linear velocity w.r.t. the current control frame (dynamic)
    float               _cf_throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _cf_forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _cf_lateral_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)

    // current limiting
    float _output_limited = 1.0f;
    float _batt_current_last = 0.0f;

    // coordinate frames
    Matrix3f            _vehicle_attitude;          // rotation matrix from vehicle's body frame to inertial frame
    Matrix3f            _control_frame;             // rotation matrix describing the coordinate frame where pilot commands are interpreted in

    // pilot throttle
    float _pilot_throttle_in = 0.0f;
};
