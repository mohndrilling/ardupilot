#include "Sub.h"

//md_net_cleaning_init - initialise attitude and position controllers
bool Sub::md_net_cleaning_init()
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

    // set control frame to 'body'
    g.control_frame = CF_Body;

    // set attitude target to current attitude
    attitude_control.reset_target_attitude();

    netcleaning.init();

    return true;
}

// md_net_cleaning_run - runs the main stabilize and depth hold controller
// should be called at 100hz or more
void Sub::md_net_cleaning_run()
{
    // update vertical speeds, acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /////////////////// Net Cleaning Logic //////////////////////

    // place holders for translational commands
    float forward_out, lateral_out, throttle_out;

    // run state machine of netcleaning library
    netcleaning.run(forward_out, lateral_out, throttle_out);

    //////////////////// Pilot Commands /////////////////////////

    //threshold for pilot input commands
    float inp_threshold = 0.05f;

    // overwrite forward and throttle command if it is sent by pilot
    if (fabsf(channel_forward->norm_input()) > inp_threshold)
        forward_out = channel_forward->norm_input();

    if (fabsf(channel_lateral->norm_input()) > inp_threshold)
        lateral_out = channel_lateral->norm_input();

    if (fabsf(channel_throttle->norm_input() - 0.5f) > inp_threshold)
        throttle_out = channel_throttle->norm_input() - 0.5f;

    // update cut off frequencies for translational input filters
    motors.set_forward_filter_cutoff(g.forward_filt);
    motors.set_lateral_filter_cutoff(g.lateral_filt);
    motors.set_pilot_throttle_filter_cutoff(g.throttle_filt);

    // output pilot's translational motion commmands
    motors.set_pilot_throttle(throttle_out);
    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);

    ///////////////////// Depth Control /////////////////////////////

    if (ap.at_bottom) {
        pos_control.relax_alt_hold_controllers(); // clear velocity and position targets
        pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
    }

    pos_control.update_z_controller();

}
