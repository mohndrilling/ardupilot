#pragma once

/// @file    AP_NetCleaning.h
/// @brief   ArduSub Net Cleaning library

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include "AC_AttitudeControl/AC_AttitudeControl_Sub.h"
#include "AC_AttitudeControl/AC_PosControl_Sub.h"
#include "AP_StereoVision/AP_StereoVision.h"

#define AP_NETCLEANING_INITIAL_NET_DISTANCE_DEFAULT 50.0f
#define AP_NETCLEANING_INITIAL_NET_DISTANCE_TOLERANCE_DEFAULT 10.0f
#define AP_NETCLEANING_APPROACHING_THROTTLE_THRUST_DEFAULT 0.2f
#define AP_NETCLEANING_CLEANING_THROTTLE_THRUST_DEFAULT 0.25f
#define AP_NETCLEANING_CLEANING_FORWARD_THRUST_DEFAULT 0.25f
#define AP_NETCLEANING_DETECTING_NET_FORWARD_THRUST_DEFAULT 0.15f
#define AP_NETCLEANING_LANE_WIDTH_DEFAULT 60.0f
#define AP_NETCLEANING_START_CLEANING_DEPTH_DEFAULT 100
#define AP_NETCLEANING_FINISH_CLEANING_DEPTH_DEFAULT 300
#define AP_NETCLEANING_CLIMBING_RATE_CMS_DEFAULT 10

#define AP_NETCLEANING_APPROACHING_INIT_ALTITUDE_POST_DELAY 3000
#define AP_NETCLEANING_HOLDING_NET_DISTANCE_POST_DELAY 6000
#define AP_NETCLEANING_ALIGNING_VERTICAL_POST_DELAY 4000
#define AP_NETCLEANING_APPROACHING_NET_POST_DELAY 7000
#define AP_NETCLEANING_ATTACHING_BRUSHES_POST_DELAY 3000
#define AP_NETCLEANING_CLEANING_NET_POST_DELAY 2000
#define AP_NETCLEANING_THROTTLE_DOWNWARDS_POST_DELAY 2000
#define AP_NETCLEANING_DETACHING_FROM_NET_POST_DELAY 5000
#define AP_NETCLEANING_ALIGNING_HORIZONTAL_POST_DELAY 4000



class AP_NetCleaning {
public:

    AP_NetCleaning( AP_AHRS_View &ahrs,
                    AP_InertialNav &inav,
                    AC_AttitudeControl_Sub& attitude_control,
                    AC_PosControl_Sub& pos_control,
                    AP_StereoVision& stereo_vision) :
                    _ahrs(ahrs),
                    _inav(inav),
                    _attitude_control(attitude_control),
                    _pos_control(pos_control),
                    _stereo_vision(stereo_vision),
                    _current_state(State::Inactive),
                    _prev_state(State::Inactive),
                    _dist_tolerance(AP_NETCLEANING_INITIAL_NET_DISTANCE_TOLERANCE_DEFAULT),
                    _loop_progress(-1)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Empty destructor to suppress compiler warning
    virtual ~AP_NetCleaning() {}
    
    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // init net cleaning
    void init();

    // run: main function running the state machine
    void run(float &forward_out, float &lateral_out, float &throttle_out);

    // detects whether the ROV has performed a full 360 degrees loop
    bool detect_loop_closure();

    // get 360 degrees loop progress in percent (further sent via mavlink)
    float get_loop_progress() { return _loop_progress; }

    // get net cleaning state to be sent via mavlink
    uint8_t get_state() { return _current_state; }

    // resets internal variables to default values
    void reset();

protected:
    enum State
    {
      Inactive,
      ApproachingInitialAltitude,
      DetectingNetInitially,
      HoldingNetDistance,
      AligningVertical,
      StartingBrushMotors,
      ApproachingNet,
      AttachingBrushes,
      CleaningNet,
      ThrottleDownwards,
      DetachingFromNet,
      AligningHorizontal,
      DetectingNetTerminally,
      Surfacing,
      WaitingAtTerminal
    };

    //////////////////////// State Logic Functions ////////////////////////////////////////////

    // approach_initial_altitude: Move to initial altitude where net cleaning is about to start
    void approach_initial_altitude();

    // detect_net_initially: move forwards until stereovision module detects the net
    void detect_net_initially();

    // hold_net_distance(): run distance controller and keep initial distance to net
    void hold_net_distance();

    // align_vertical: perform rotational trajectory such that brushes face the net
    void align_vertical();

    // approach_net: throttles along vehicles z-axis until auv touch the net.
    void approach_net();

    // attach_brushes: relax yaw and pitch controller and keep throttling, so brushes properly align to the net.
    void attach_brushes();

    // clean_net: move forwards whith activated brushes pushed to the net
    void clean_net();

    // throttle_downwards: move to the next cleaning lane
    void throttle_downwards();

    // detach_from_net: stabilize attitude and move AUV away from net
    void detach_from_net();

    // align_horizontal: perform rotational trajectory back to horizontl orientation
    void align_horizontal();

    // detect_net_terminally: move forwards until stereovision module detects the net again
    void detect_net_terminally();

    // surface: move back to surface while keeping fixed distance and orientation towards net
    void surface();

    // wait_at_terminal: keeping fixed distance to net without translational movement
    void wait_at_terminal();

    ////////////////// Helper Functions //////////////////////////////////////////////////
    // detect_net_terminally: move forwards until stereovision module detects the net again
    void detect_net();

    // hold_heading_and_distance: keeps desired distance and perpendicular heading w.r.t. the net
    void hold_heading_and_distance(float target_dist);

    // run_net_cleaning_attitude_control: keep nose horizontal and relax roll and pitch controller
    void run_net_cleaning_attitude_control();

    // switch_state: switch the state of the state machine
    void switch_state(State target_state, const char *state_name);

    // switch_state_after_post_delay: wait for specified time and switch to target_state afterwards
    void switch_state_after_post_delay(State target_state, const char *state_name, uint32_t milliseconds);

    // set_state_logic_finished: set timestamp and flag (called when a state has finished its task)
    void set_state_logic_finished();

    //update loop progress (just for monitoring)
    void update_loop_progress();

    // References to external libraries
    const AP_AHRS_View&         _ahrs;
    AC_AttitudeControl_Sub&     _attitude_control;
    AP_InertialNav&             _inav;
    AC_PosControl_Sub&          _pos_control;
    AP_StereoVision&            _stereo_vision;



    // stores time difference (seconds) between incoming messages of stereovision module
    // updated each loop
    struct SensorIntervals
    {
        float stv_dt; // dt of stereo vision messages
    };

    // stores whether each of the sensor modules holds new information
    struct SensorUpdated
    {
        bool stv_updated; // whether stereovision module has new data
    };

    // the desired translational movement of the vehicle
    float _forward_out;
    float _lateral_out;
    float _throttle_out;

    // stores the accumulated yaw value at start of each new 360 degrees loop.
    float _initial_yaw;

    // the altitude at start of net cleaning
    float _home_altitude;

    // indicates whether maximum depth is reached
    bool _terminate;

    // current net cleaning State
    State _current_state;

    // previous net cleaning State
    State _prev_state;

    // 360 degrees loop progress in percent
    float _loop_progress;

    // sensor information
    SensorIntervals _sensor_intervals;
    SensorUpdated _sensor_updates;
    uint32_t _last_stereo_update_ms;

    /////////////// state specific variables ////////////////

    ///////// shared between states

    // true if the task of the current state is fulfilled
    bool _state_logic_finished;

    // stores time stamp of last state execution
    uint32_t _last_state_execution_ms;

    // the time stamp of when the task of the current state was fulfilled
    uint32_t _state_logic_finished_ms;

    ///////// ApproachingNet

    // target distance towards net (cm)
    AP_Int16 _initial_net_distance;

    // tolerance for target distance (m)
    float _dist_tolerance;

public:
};
