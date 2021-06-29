#include "Sub.h"


/*
 * control_common.cpp - control logic that is shared by multiple flight modes
*/

void Sub::control_depth() {
    // Hold actual position until zero derivative is detected
    static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    static bool lastVelocityZWasNegative = false;

    //threshold for pilot input commands
    float inp_threshold = 0.05f;

    // relax alt controller if throttle inputs are above threshold, or if forward or lateral inputs are above threshold AND the control frame
    // is set to BODY. (Scenario: control frame is set to body and vehicle pitched nose up. As we drive forward, we actually affect the altitude, so
    // the depth controller would block this movement if not relaxed)
    bool forw_lat_in = fabsf(channel_forward->norm_input()) > inp_threshold
                       || fabsf(channel_lateral->norm_input()) > inp_threshold;

    bool relax_depth_control = fabsf(channel_throttle->norm_input()-0.5f) > inp_threshold
                               || (g.control_frame == CF_Body && forw_lat_in);

    if (relax_depth_control) { // Pilot input above 5%

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
}
