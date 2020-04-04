#include "Sub.h"

//md_net_tracking_init - initialise attitude and position controllers
bool Sub::md_net_tracking_init()
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

    nettracking.init();

    return true;
}

// md_net_tracking_run - runs the main stabilize and depth hold controller
// should be called at 100hz or more
void Sub::md_net_tracking_run()
{
    // update vertical speeds, acceleration and net tracking distance
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
    // if stereovision keeps receiving heading and distance information from stereo camera data via mavlink, run distance and attitude controllers
    if (stereovision.stereo_vision_healthy())
    {
        nettracking.perform_net_tracking(forward_out, lateral_out, throttle_out);

        // overwrite forward and throttle command if it is sent by pilot
        if (fabsf(channel_forward->norm_input()) > inp_threshold)
            forward_out = channel_forward->norm_input();

        if (fabsf(channel_throttle->norm_input() - 0.5f) > inp_threshold)
            throttle_out = channel_throttle->norm_input() - 0.5f;

        // overwrite lateral command, if there are any pilot inputs (-> hence the lateral scanning motion is stopped during pilot inputs)
        if ( pilot_in )
            lateral_out = channel_lateral->norm_input();

        // update the heading, the vehicle keeps the current heading after loosing input data from stereovision library
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    }
    else {
        // target attitude from pilot commands
        float target_roll, target_pitch, target_yaw_rate;

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // call attitude controller
        if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            last_pilot_heading = ahrs.yaw_sensor;
            last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

        } else { // hold current heading
            // this check is required to prevent bounce back after very fast yaw maneuvers
            // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
            if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
                target_yaw_rate = 0; // Stop rotation on yaw axis

                // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
                attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
                last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

            } else { // call attitude controller holding absolute absolute bearing
                attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
            }
        }

        // set pilot input commands
        forward_out = channel_forward->norm_input();
        lateral_out = channel_lateral->norm_input();
    }

    ///////////// Depth Control ////////////////////////////////////////
    // Hold actual position until zero derivative is detected
    static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    static bool lastVelocityZWasNegative = false;

    if ( pilot_throttle_in || fabs(throttle_out) > 0.0f) { // Pilot throttle input above 5% or throttle_out set by nettracking library

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
