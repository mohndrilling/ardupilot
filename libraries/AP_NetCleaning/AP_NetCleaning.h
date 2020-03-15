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

#define AP_NETCLEANING_INITIAL_NET_DISTANCE_DEFAULT 70

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
                    _state(State::ApproachingNet),
                    _loop_progress(-1)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Empty destructor to suppress compiler warning
    virtual ~AP_NetCleaning() {}
    
    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // init net tracking
    void init();

    // run: main function running the state machine
    void run(float &forward_out, float &lateral_out, float &throttle_out);

    // detects whether the ROV has performed a full 360 degrees loop
    bool detect_loop_closure();

    // get 360 degrees loop progress in percent (further sent via mavlink)
    float get_loop_progress() { return _loop_progress; }

    // get net tracking state to be sent via mavlink
    uint8_t get_state() { return _state; }

    // resets internal variables to default values
    void reset();

protected:
    enum State
    {
      Inactive,
      Waiting,
      ApproachingNet,
      AligningToNet,
      StartingBrushMotors,
      AttachingToNet,
      Cleaning,
      ThrottleDownwards,
      Surfacing
    };

    // approach_net: contains logic of ApproachingNet state
    void approach_net(float &forward_out, float &lateral_out, float &throttle_out);

    // align_to_net: perform rotational trajectory such that brushes face the net
    void align_to_net(float &forward_out, float &lateral_out, float &throttle_out);

    // hold_heading_and_distance: keeps desired distance and perpendicular heading w.r.t. the net
    void hold_heading_and_distance(float &forward_out, float target_dist);

    // switch_state_with_delay: wait for specified time and switch to target_state afterwards
    void switch_state_with_delay(uint32_t milliseconds, AP_NetCleaning::State target_state);

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

    // stores the accumulated yaw value at start of each new 360 degrees loop.
    float _initial_yaw;    

    // the heading (yaw angle in radians) at start of net tracking
    float _home_yaw;

    // the altitude at start of net tracking
    float _home_altitude;

    // net tracking State
    State _state;

    // 360 degrees loop progress in percent
    float _loop_progress;

    // sensor information
    SensorIntervals _sensor_intervals;
    SensorUpdated _sensor_updates;
    uint32_t _last_stereo_update_ms = 0;

    /////////////// state specific variables ////////////////

    ///////// ApproachingNet

    // target distance towards net (cm)
    AP_Int16 _initial_net_distance;

    // time stamp, when vehicle reaches target distance towards net
    uint32_t _target_dist_reached_ms = 0;

    // whether target distance is reached
    bool _target_dist_reached = false;

    // periods in milliseconds, that vehicle has to obtain target distance before changing to next state
    uint32_t _hold_dist_ms = 4000; // 2 seconds

    // tolerance for target distance (m)
    float _dist_tolerance = 0.1;

public:
};
