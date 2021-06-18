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
        attitude_control.set_throttle_out(0,false,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_alt_hold_controllers();
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    /////////////////// Net Tracking Logic //////////////////////

    // place holders for translational commands
    float forward_out, lateral_out, throttle_out;

    // run state machine of netcleaning library
    nettracking.run(forward_out, lateral_out, throttle_out);

    //////////////////// Pilot Commands /////////////////////////

    //threshold for pilot input commands
    float inp_threshold = 0.05f;
    uint32_t man_ctrl_timeout = static_cast<uint32_t>(1000 * g.manual_ctrl_timeout);

    uint32_t now = AP_HAL::millis();

    // overwrite forward and throttle command if it is sent by pilot
    if (fabsf(channel_forward->norm_input()) > inp_threshold)
        pilot_input.last_forward_ms = now;

    if (fabsf(channel_lateral->norm_input()) > inp_threshold)
        pilot_input.last_lateral_ms = now;

    if (now - pilot_input.last_forward_ms < man_ctrl_timeout)
    {
        forward_out = channel_forward->norm_input();
        pos_control.relax_dist_controller();
    }

    if (now - pilot_input.last_lateral_ms < man_ctrl_timeout)
    {
        lateral_out = channel_lateral->norm_input();        
        pos_control.relax_optflx_controller();
    }

    bool relax_depth_control = fabsf(channel_throttle->norm_input()-0.5f) > inp_threshold;

    if (relax_depth_control)
    {
        throttle_out = channel_throttle->norm_input() - 0.5f;

        // disable depth control
        // the throttle for hovering will be applied along inertial z-axis
        // all of the remaining pilot inputs will be added up on top of that applied to the axes corresponding to the current control frame
        // see update_control_frame in motors.cpp and output_armed_stabilizing_vectored_6dof() in Motors6DOF.cpp
        attitude_control.set_throttle_out(motors.get_throttle_hover(), false, g.throttle_filt);
        // reset z targets to current values
        pos_control.relax_alt_hold_controllers();
    }
    else
    {
        pos_control.update_z_controller();
    }

    // update cut off frequencies for translational input filters
    motors.set_forward_filter_cutoff(g.forward_filt);
    motors.set_lateral_filter_cutoff(g.lateral_filt);
    motors.set_pilot_throttle_filter_cutoff(g.throttle_filt);

    // output pilot's translational motion commmands
    motors.set_pilot_throttle(throttle_out);
    motors.set_forward(forward_out);
    motors.set_lateral(lateral_out);

}
