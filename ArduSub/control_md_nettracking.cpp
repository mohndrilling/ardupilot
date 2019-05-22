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


    ///////////// Attitude and Distance Control ////////////////////////
    // if stereovision keeps receiving heading and distance information from stereo camera data via mavlink, run distance and attitude controllers
    if (stereovision.healthy())
    {
        perform_net_tracking();
    }
    else {
        // target attitude from pilot commands
        float target_roll, target_pitch, target_yaw;

        // convert pilot input to lean angles
        // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
        get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

        // get pilot's desired yaw rate
        target_yaw = get_pilot_desired_yaw_angle(channel_yaw->get_control_in());

        // call attitude controller
        // update attitude controller targets
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);

        // set commands for forward and lateral motion
        motors.set_forward(channel_forward->norm_input());
        motors.set_lateral(channel_lateral->norm_input());
    }

    ///////////// Depth Control ////////////////////////////////////////
    // Hold actual position until zero derivative is detected
    static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    static bool lastVelocityZWasNegative = false;
    if (fabsf(channel_throttle->norm_input()-0.5f) > 0.05f) { // Throttle input above 5%
        // output pilot's throttle
        attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);
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
}

// net tracking logic
void Sub::perform_net_tracking()
{
    // retrieve current distance from stereovision module
    float cur_dist = stereovision.get_distance();

    // desired distance (m)
    float d_dist = float(g.nettracking_distance) / 100.0f;
    float dist_error = cur_dist - d_dist;

    // time difference (in seconds) between two measurements from stereo vision is used to lowpass filter the data
    float dt = stereovision.get_time_delta_usec() / 1000000.0f;

    // only update target distance and attitude, if new measurement from stereo data available
    bool update_target = stereovision.get_last_update_ms() - last_stereo_update_ms != 0;

    // get forward command from distance controller
    if (update_target)
    {
        last_stereo_update_ms = stereovision.get_last_update_ms();
        float target_forward;
        pos_control.update_dist_controller(target_forward, dist_error, dt);
        motors.set_forward(target_forward);
    }

    // if distance error is small enough, use the stereovision heading data to always orientate the vehicle normal to the faced object surface
    if (abs(dist_error) < 0.3)
    {
        // no roll desired
        float target_roll = 0.0f;

        // get pitch and yaw offset (in centidegrees) with regard to the faced object (net) in front
        float target_pitch_error = stereovision.get_delta_pitch();
        float target_yaw_error = stereovision.get_delta_yaw();

        // the target values will be ignored, if no new stereo vision data received (update_target = false)
        // this will update the target attitude corresponding to the current errors and trigger the attitude controller
        attitude_control.input_euler_roll_pitch_yaw_accumulate(target_roll, target_pitch_error, target_yaw_error, dt, update_target);

        // set commands for lateral motion
        motors.set_lateral(g.nettracking_velocity);
    }
    else
    {
        // if the distance is too large, the vehicle is supposed to obtain the current attitude and to not move laterally
        // call attitude controller
        attitude_control.input_euler_roll_pitch_yaw_accumulate(0.0f, 0.0f, 0.0f, dt, false);

        // no lateral movement
        motors.set_lateral(0.0f);
    }

}
