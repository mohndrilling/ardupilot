#include "Sub.h"

//md stabilize_init - initialise stabilize controller
bool Sub::md_stabilize_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);
    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// md stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::md_stabilize_run()
{
    // target attitude
    float target_roll, target_pitch, target_yaw;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        last_pilot_heading = ahrs.yaw_sensor;
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

    // output pilot's throttle
    attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}
