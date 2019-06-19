#include "Sub.h"
#include <GCS_MAVLink/GCS.h>

//md_althold_init - initialise attitude and position controllers
bool Sub::md_althold_init()
{
    if(!control_check_barometer()) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    return true;
}

// md_althold_run - runs the main stabilize and depth hold controller
// should be called at 100hz or more
void Sub::md_althold_run()
{
    // target attitude
    float target_roll, target_pitch, target_yaw;

    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_alt_hold_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw = get_pilot_desired_yaw_angle(channel_yaw->get_control_in());

    // call attitude controller
    // update attitude controller targets
    attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);

    // run depth control (implemented in control_althold.cpp)
    control_depth();

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
