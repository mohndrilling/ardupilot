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

    // set initial net tracking velocity
    net_track_vel = g.nettracking_velocity;

//    attitude_control.reset_target_attitude();

    return true;
}

// md_net_tracking_run - runs the main stabilize and depth hold controller
// should be called at 100hz or more
void Sub::md_net_tracking_run()
{
    // update vertical speeds, acceleration and net tracking distance
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // update net tracking velocity
    net_track_vel = g.nettracking_velocity;

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
    float forward_out, lateral_out;

    //threshold for pilot input commands
    float inp_threshold = 0.05f;

    // whether there are any pilot commands
    bool pilot_in =  fabsf(channel_throttle->norm_input()-0.5f) > inp_threshold
                 || fabsf(channel_forward->norm_input()) > inp_threshold
                 || fabsf(channel_lateral->norm_input()) > inp_threshold;

    ///////////// Attitude and Distance Control ////////////////////////
    // if stereovision keeps receiving heading and distance information from stereo camera data via mavlink, run distance and attitude controllers
    if (stereovision.healthy())
    {
        perform_net_tracking(forward_out, lateral_out);

        // overwrite forward command if it is sent by pilot
        if (fabsf(channel_forward->norm_input()) > inp_threshold)
            forward_out = channel_forward->norm_input();

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

    if ( pilot_in ) { // Pilot input above 5%

        // disable depth control
        // the throttle for hovering will be applied along inertial z-axis
        // all of the remaining pilot inputs will be added up on top of that applied to the axes corresponding to the current control frame
        // see update_control_frame in motors.cpp and output_armed_stabilizing_vectored_6dof() in Motors6DOF.cpp
        attitude_control.set_throttle_out(motors.get_throttle_hover(), false, g.throttle_filt);

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
    motors.set_pilot_throttle(channel_throttle->norm_input() - 0.5f);
    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);
}

// net tracking logic
void Sub::perform_net_tracking(float &forward_out, float &lateral_out)
{
    // time difference (in seconds) between two measurements from stereo vision is used to lowpass filter the data
    float dt = stereovision.get_time_delta_usec() / 1000000.0f;

    if (dt > 2.0f)
    {
        attitude_control.reset_yaw_err_filter();
    }

    // only update target distance and attitude, if new measurement from stereo data available
    bool update_target = stereovision.get_last_update_ms() - last_stereo_update_ms != 0;

    last_stereo_update_ms = stereovision.get_last_update_ms();

    // whether to perform vision based attitude control
    bool att_ctrl;

    if (g.nettracking_mesh_ctrl)
    {
        // retrieve amount of currently visible net meshes
        float cur_mesh_cnt = stereovision.get_mesh_count();


        // desired mesh count (control the square root of current mesh count, since total meshcount grows quadratically over the distance to the net)
        // but we want a linear dependency between control input (forward throttle) and control variable (square rooted mesh count)
        float d_mesh_cnt = g.nettracking_mesh_cnt;
        float mesh_cnt_error = safe_sqrt(cur_mesh_cnt) - d_mesh_cnt;

        // get forward command from mesh count controller
        pos_control.update_mesh_cnt_controller(forward_out, mesh_cnt_error, dt, update_target);

        att_ctrl = abs(mesh_cnt_error) < 10.0f;

    }
    else
    {
        // retrieve current distance from stereovision module
        float cur_dist = stereovision.get_distance();

        // desired distance (m)
        float d_dist = float(g.nettracking_distance) / 100.0f;
        float dist_error = cur_dist - d_dist;

        // get forward command from distance controller
        pos_control.update_dist_controller(forward_out, dist_error, dt, update_target);

        att_ctrl = abs(dist_error) < 1.0f;
    }



    // if distance error is small enough, use the stereovision heading data to always orientate the vehicle normal to the faced object surface
    if (att_ctrl)
    {
        // no roll desired
        float target_roll = 0.0f;

        // get pitch and yaw offset (in centidegrees) with regard to the faced object (net) in front
        float target_pitch_error = stereovision.get_delta_pitch();
        float target_yaw_error = stereovision.get_delta_yaw();

        // assume concave net shape -> only allow increasing/decreasing of yaw error w.r.t. the direction of movement
//        float scan_dir = is_negative(float(net_track_vel)) ? -1.0f : 1.0f;
//        target_yaw_error = scan_dir * target_yaw_error < 0 ? target_yaw_error : 0;

        // only change pitch when changing altitude
//        target_pitch_error = fabsf(channel_throttle->norm_input()-0.5f) > 0.05f ? target_pitch_error : 0.0f;

        // the target values will be ignored, if no new stereo vision data received (update_target = false)
        // this will update the target attitude corresponding to the current errors and trigger the attitude controller
        attitude_control.input_euler_roll_pitch_yaw_accumulate(target_roll, target_pitch_error, target_yaw_error, dt, update_target);

        // scale net tracking velocity proportional to yaw error
//        float ls_tmp = target_yaw_error / 2000.0f;
//        float lat_scale_f = ls_tmp * ls_tmp * ls_tmp; // ... to be beautified

//        // set commands for lateral motion
//        lateral_out = (1.0f + lat_scale_f) * net_track_vel / 100.0f;

        float mesh_distr = stereovision.get_mesh_distr();
        nettr_toggle_velocity = nettr_toggle_velocity || fabs(mesh_distr - 0.5f) < 0.1f;
        float distr_thr = 0.15f;
        bool net_edge_reached = mesh_distr < distr_thr || mesh_distr > (1.0f - distr_thr);
        if (nettr_toggle_velocity && net_edge_reached)
        {
            nettr_direction *= -1.0f;
            nettr_toggle_velocity = false;
        }

        lateral_out = nettr_direction * net_track_vel / 1000.0f;
    }
    else
    {
        // if the distance is too large, the vehicle is supposed to obtain the current attitude and to not move laterally
        // call attitude controller
        attitude_control.input_euler_roll_pitch_yaw_accumulate(0.0f, 0.0f, 0.0f, dt, false);

        // no lateral movement
        lateral_out = 0.0f;
    }

}
