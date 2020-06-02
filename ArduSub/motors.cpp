#include "Sub.h"

// enable_motor_output() - enable and output lowest possible value to motors
void Sub::enable_motor_output()
{
    motors.output_min();
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool Sub::init_arm_motors(AP_Arming::Method method)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }

    in_arm_motors = true;

    if (!arming.pre_arm_checks(true)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // let logger know that we're armed (it may open logs e.g.)
    AP::logger().set_vehicle_armed(true);

    // disable cpu failsafe because initialising everything takes a while
    mainloop_failsafe_disable();

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        notify.update();
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Arming motors");
#endif

    initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)

        // Always use absolute altitude for ROV
        // ahrs.resetHeightDatum();
        // Log_Write_Event(DATA_EKF_ALT_RESET);
    } else if (ahrs.home_is_set() && !ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        if (!set_home_to_current_location(false)) {
            // ignore this failure
        }
    }
	
    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

    // enable output to motors
    enable_motor_output();

    // finally actually arm the motors
    motors.armed(true);

    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    logger.Write_Mode(control_mode, control_mode_reason);

    // reenable failsafe
    mainloop_failsafe_enable();

    // perf monitor ignores delay due to arming
    scheduler.perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}

// init_disarm_motors - disarm motors
void Sub::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Disarming motors");
#endif

    // save compass offsets learned by the EKF if enabled
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LEARN_EKF) {
        for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

    Log_Write_Event(DATA_DISARMED);

    // send disarm command to motors
    motors.armed(false);

    // reset the mission
    mission.reset();

    AP::logger().set_vehicle_armed(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    // clear input holds
    clear_input_hold();
}

// check_auto_arm_motors - automatically arm motors if auto arming enabled and arming depth reached
void Sub::check_auto_arm_motors()
{
    // return immediately if we are already disarmed
    if (motors.armed() || g.auto_arming == DISABLED) {
        return;
    }

    // get current altitude in cm
    float cur_alt = inertial_nav.get_altitude();

    if (cur_alt > g.surface_depth)
    {
        auto_arming_allowed = true;
    }

    // arm motors if the current altitude is lower than the minimum auto arming depth
    if (cur_alt < -g.auto_arming_depth && auto_arming_allowed)
    {
        init_arm_motors(AP_Arming::Method::AUTO);
        auto_arming_allowed = false; // the vehicle has to surface before it's allowed to auto arm again
    }

    // TODO: Implement a disarming routine as well. E.g. make the vehicle ascending, when unarmed.
    // It may be automatically disarmed above the auto_arming_depth then.
}

// update_control_frame - sends the rotation matrix of the current control frame to the motor classes
// the motor classes transform the thrust factors of each thruster accordingly
void Sub::update_control_frame()
{
    // get current vehicle attitude
    Quaternion vehicle_attitude;
    ahrs.get_quat_body_to_ned(vehicle_attitude);

    // the thrust commands from the depth controller are always supposed to be directed along the inertial z-axis
    // inform the motor classes about the current vehicle attitude, so they can assign the distinct thruster outputs such that throttle along inertial z-axis is achieved.
    Matrix3f att_to_rot_matrix; // rotation from the target body frame to the inertial frame.
    vehicle_attitude.rotation_matrix(att_to_rot_matrix);
    motors.set_vehicle_attitude(att_to_rot_matrix);

    // inform motor classes about current control frame
    Quaternion control_frame_q;
    Matrix3f control_frame_mat;

    switch(g.control_frame)
    {
        case CF_Inertial:
            control_frame_q = vehicle_attitude;
            break;

        case CF_Local:
            // the local frame is horizontally levelled (as the inertial frame) but rotated around the z-axis corresponding to current vehicles yaw angle
            // so the projection of the x-axis of the ROV to the inertial xy-plane points to the same direction as the local frame's x-axis

            // get the current vehicle attitude describes as euler angles
            float current_roll, current_pitch, current_yaw;
            vehicle_attitude.to_euler(current_roll, current_pitch, current_yaw);

            // pitching above 90 degrees leads to jump of yaw angle (lateral pilot commands will change direction)
            // the local control frame feels unintuitive for large roll angles as well.
            // Therefore we switch to 'body' control frame, if lean angles exceed following maxima
            if (fabs(degrees(current_roll)) > 45.0f || fabs(degrees(current_pitch)) > 80.0f)
            {
                uint32_t tnow = AP_HAL::millis();
                if (tnow > last_control_frame_fail + 10000) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Control frame 'Local': Maximum lean angle exceeded! Switching to control frame 'Body'.");
                    last_control_frame_fail = tnow;
                    control_frame_q.from_euler(0.0f, 0.0f, 0.0f);
                }
            }
            else
            {
                // rotate the vehicle attitude by -current_yaw back around the z-axis to obtain local control frame
                Quaternion yaw_compensation_q;
                yaw_compensation_q.from_euler(0.0f, 0.0f, -current_yaw);
                control_frame_q = yaw_compensation_q * vehicle_attitude;
            }

            break;

        case CF_Body:
            // zero transformation: pilot commands will be statically executed w.r.t. the vehicle's body frame
            control_frame_q.from_euler(0.0f, 0.0f, 0.0f);
            break;

        default:
            // use 'body' control frame as default
            control_frame_q.from_euler(0.0f, 0.0f, 0.0f);
            break;
    }

    // finally inform motor classes about resulting control frame
    control_frame_q.rotation_matrix(control_frame_mat);
    motors.set_control_frame(control_frame_mat);

}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Sub::motors_output()
{
    // check if we are performing the motor test
    if (ap.motor_test) {
        verify_motor_test();
    } else {
        motors.set_interlock(true);
        motors.output();
    }

    SRV_Channels::push();
}

// Initialize new style motor test
// Perform checks to see if it is ok to begin the motor test
// Returns true if motor test has begun
bool Sub::init_motor_test()
{
    uint32_t tnow = AP_HAL::millis();

    // Ten second cooldown period required with no do_set_motor requests required
    // after failure.
    if (tnow < last_do_motor_test_fail_ms + 10000 && last_do_motor_test_fail_ms > 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "10 second cool down required");
        return false;
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Disarm hardware safety switch before testing motors.");
        return false;
    }

    // Make sure we are on the ground
    if (!motors.armed()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Arm motors before testing motors.");
        return false;
    }

    enable_motor_output(); // set all motor outputs to zero
    ap.motor_test = true;

    return true;
}

// Verify new style motor test
// The motor test will fail if the interval between received
// MAV_CMD_DO_SET_MOTOR requests exceeds a timeout period
// Returns true if it is ok to proceed with new style motor test
bool Sub::verify_motor_test()
{
    bool pass = true;

    // Require at least 2 Hz incoming do_set_motor requests
    if (AP_HAL::millis() > last_do_motor_test_ms + 500) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Motor test timed out!");
        pass = false;
    }

    if (!pass) {
        ap.motor_test = false;
        motors.armed(false); // disarm motors
        last_do_motor_test_fail_ms = AP_HAL::millis();
        return false;
    }

    return true;
}

bool Sub::handle_do_motor_test(mavlink_command_long_t command) {
    last_do_motor_test_ms = AP_HAL::millis();

    // If we are not already testing motors, initialize test
    if(!ap.motor_test) {
        if (!init_motor_test()) {
            gcs().send_text(MAV_SEVERITY_WARNING, "motor test initialization failed!");
            return false; // init fail
        }
    }

    float motor_number = command.param1;
    float throttle_type = command.param2;
    float throttle = command.param3;
    // float timeout_s = command.param4; // not used
    // float motor_count = command.param5; // not used
    float test_type = command.param6;

    if (!is_equal(test_type, (float)MOTOR_TEST_ORDER_BOARD)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad test type %0.2f", (double)test_type);
        return false; // test type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PILOT)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "bad throttle type %0.2f", (double)throttle_type);

        return false; // throttle type not supported here
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PWM)) {
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    if (is_equal(throttle_type, (float)MOTOR_TEST_THROTTLE_PERCENT)) {
        throttle = constrain_float(throttle, 0.0f, 100.0f);
        throttle = channel_throttle->get_radio_min() + throttle / 100.0f * (channel_throttle->get_radio_max() - channel_throttle->get_radio_min());
        return motors.output_test_num(motor_number, throttle); // true if motor output is set
    }

    return false;
}


// translate wpnav roll/pitch outputs to lateral/forward
void Sub::translate_wpnav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = wp_nav.get_roll();
    int32_t forward = -wp_nav.get_pitch(); // output is reversed

    // constrain target forward/lateral values
    // The outputs of wp_nav.get_roll and get_pitch should already be constrained to these values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

// translate wpnav roll/pitch outputs to lateral/forward
void Sub::translate_circle_nav_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = circle_nav.get_roll();
    int32_t forward = -circle_nav.get_pitch(); // output is reversed

    // constrain target forward/lateral values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}

// translate pos_control roll/pitch outputs to lateral/forward
void Sub::translate_pos_control_rp(float &lateral_out, float &forward_out)
{
    // get roll and pitch targets in centidegrees
    int32_t lateral = pos_control.get_roll();
    int32_t forward = -pos_control.get_pitch(); // output is reversed

    // constrain target forward/lateral values
    lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
    forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

    // Normalize
    lateral_out = (float)lateral/(float)aparm.angle_max;
    forward_out = (float)forward/(float)aparm.angle_max;
}
