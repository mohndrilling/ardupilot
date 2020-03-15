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

    last_pilot_heading = ahrs.yaw_sensor;

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
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get current time
    uint32_t tnow = AP_HAL::millis();

    // place holders for translational commands
    float forward_out, lateral_out, throttle_out;

    //threshold for pilot input commands
    float inp_threshold = 0.05f;

    // whether there are any pilot throttle commands
    bool pilot_throttle_in =  fabsf(channel_throttle->norm_input()-0.5f) > inp_threshold;

    // whether there are any pilot input commands
    bool pilot_in = pilot_throttle_in
                    || fabsf(channel_forward->norm_input()) > inp_threshold
                    || fabsf(channel_lateral->norm_input()) > inp_threshold;

    ///////////// Attitude and Distance Control ////////////////////////

    // run state machine of netcleaning library
    netcleaning.run(forward_out, lateral_out, throttle_out);

    // overwrite forward and throttle command if it is sent by pilot
    if (fabsf(channel_forward->norm_input()) > inp_threshold)
        forward_out = channel_forward->norm_input();

    if (fabsf(channel_throttle->norm_input() - 0.5f) > inp_threshold)
        throttle_out = channel_throttle->norm_input() - 0.5f;

    // overwrite lateral command, if there are any pilot inputs (-> hence the lateral scanning motion is stopped during pilot inputs)
    if ( pilot_in )
        lateral_out = channel_lateral->norm_input();


    ///////////// Depth Control ////////////////////////////////////////
    // Hold actual position until zero derivative is detected
    static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    static bool lastVelocityZWasNegative = false;

    if ( pilot_throttle_in || fabs(throttle_out) > 0.0f) { // Pilot throttle input above 5% or throttle_out set by netcleaning library

        // disable depth control
        // the throttle for hovering will be applied along inertial z-axis
        // all of the remaining pilot inputs will be added up on top of that applied to the axes corresponding to the current control frame
        // see update_control_frame in motors.cpp and output_armed_stabilizing_vectored_6dof() in Motors6DOF.cpp
        attitude_control.set_throttle_out(motors.get_throttle_hover(), false, g.throttle_filt);

        // update cut off frequencies for translational input filters
        motors.set_forward_filter_cutoff(g.forward_filt);
        motors.set_lateral_filter_cutoff(g.lateral_filt);
        motors.set_pilot_throttle_filter_cutoff(g.throttle_filt);

        // reset z targets to current values
        pos_control.relax_alt_hold_controllers();
        engageStopZ = true;
        lastVelocityZWasNegative = is_negative(inertial_nav.get_velocity_z());
    } else { // hold z

        if (ap.at_bottom) {
            pos_control.relax_alt_hold_controllers(); // clear velocity and position targets
            pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
        } else if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            float target_climb_rate = get_surface_tracking_climb_rate(0, pos_control.get_alt_target(), G_Dt);
            pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        }

        // Detects a zero derivative
        // When detected, move the altitude set point to the actual position
        // This will avoid any problem related to joystick delays
        // or smaller input signals
        if(engageStopZ && (lastVelocityZWasNegative ^ is_negative(inertial_nav.get_velocity_z()))) {
            engageStopZ = false;
            pos_control.relax_alt_hold_controllers();
        }

        pos_control.update_z_controller();
    }

    // output pilot's translational motion commmands
    motors.set_pilot_throttle(throttle_out);
    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}
