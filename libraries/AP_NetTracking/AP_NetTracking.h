#pragma once

/// @file    AP_NetTracking.h
/// @brief   ArduSub Net tracking library

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include "AC_AttitudeControl/AC_AttitudeControl_Sub.h"
#include "AC_AttitudeControl/AC_PosControl_Sub.h"
#include "AP_StereoVision/AP_StereoVision.h"

#define AP_NETTRACKING_NETSHAPE_DEFAULT NetShape::Tube
#define AP_NETTRACKING_DISTANCE_DEFAULT 50
#define AP_NETTRACKING_MESH_CNT_DEFAULT 200
#define AP_NETTRACKING_VELOCITY_DEFAULT 0.0f
#define AP_NETTRACKING_OPTFLOW_VELOCITY_FACTOR 2.0f
#define AP_NETTRACKING_DEFAULT_VELOCITY_FACTOR 0.8f
#define AP_NETTRACKING_CTRL_VAR_DEFAULT ControlVar::ctrl_distance
#define AP_NETTRACKING_VEL_CTRL_DEFAULT 1
#define AP_NETTRACKING_THR_SPEED_DEFAULT 0.05f
#define AP_NETTRACKING_PHASE_SHIFT_THR_DIST_DEFAULT 200
#define AP_NETTRACKING_PHASE_SHIFT_CUTOFF_FREQ_DEFAULT 0.2f

class AP_NetTracking {
public:

    AP_NetTracking( AP_AHRS_View &ahrs,
                    AP_InertialNav &inav,
                    AC_AttitudeControl_Sub& attitude_control,
                    AC_PosControl_Sub& pos_control,
                    AP_StereoVision& stereo_vision) :
                    _ahrs(ahrs),
                    _inav(inav),
                    _attitude_control(attitude_control),
                    _pos_control(pos_control),
                    _stereo_vision(stereo_vision),
                    _state(State::Scanning),
                    _perform_att_ctrl(false),
                    _phase_shift_filt(AP_NETTRACKING_PHASE_SHIFT_CUTOFF_FREQ_DEFAULT),
                    _loop_progress(-1),
                    _nettr_velocity(0.0f),
                    _nettr_default_vel_factor(AP_NETTRACKING_DEFAULT_VELOCITY_FACTOR),
                    _nettr_opt_flow_vel_factor(AP_NETTRACKING_OPTFLOW_VELOCITY_FACTOR)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Empty destructor to suppress compiler warning
    virtual ~AP_NetTracking() {}
    
    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // init net tracking
    void init();

    // perform net tracking: calls attitude and pos controller to maintain orthonormal heading and distance
    void perform_net_tracking(float &forward_out, float &lateral_out, float &throttle_out);

    // sets state to 'ReturnHome'
    // in case of a closed net shape (fish farm), ROV continues scanning until initial heading reached, then throttles to initial altitude
    // in case of plane net shape, ROV directly throttles to initial altitude
    void set_return_home() { _state = _net_shape == NetShape::Tube ? State::ReturnToHomeHeading : State::ReturnToHomeAltitude; }

    // get net tracking state to be sent via mavlink
    uint8_t get_state() { return _state; }

    // get 360 degrees loop progress in percent (further sent via mavlink)
    float get_loop_progress() { return _loop_progress; }

    // resets internal variables to default values
    void reset();

protected:

    // call distance controller and attitude controller to keep desired distance and heading to net plane
    void scan(float &forward_out, float &lateral_out, float &throttle_out);

    // calculate lateral velocity (either static or based on phase shift measurement)
    void update_lateral_out(float &lateral_out);

    // calculate forward velocity (either distance or mesh count controller)
    void update_forward_out(float &forward_out);

    // calls the attitude controller to achieve normal heading of the ROV w.r.t. the net plane
    void update_heading_control();

    // throttles the vehicle down by constant throttle
    void update_throttle_out(float &throttle_out);

    // updates the phase shift low pass filter by new measurement values and accumulates the absolute distance, the image has shifted
    void update_phase_shift();

    //update loop progress (just for monitoring)
    void update_loop_progress();

    // References to external libraries
    const AP_AHRS_View&         _ahrs;
    AC_AttitudeControl_Sub&     _attitude_control;
    AP_InertialNav&             _inav;
    AC_PosControl_Sub&          _pos_control;
    AP_StereoVision&            _stereo_vision;

    //
    enum NetShape
    {
        Plane,
        Tube
    };

    enum ControlVar
    {
        ctrl_distance,
        ctrl_meshcount
    };

    enum State
    {
      Inactive,
      Scanning,
      Throttle,
      Pause,
      ReturnToHomeHeading,
      ReturnToHomeAltitude,
    };

    // stores time difference (seconds) between each incoming message of stereovision, image shift and net inspection modules
    // updated each loop
    struct SensorIntervals
    {
        float stv_dt; // dt of stereo vision messages
        float ni_dt;  // dt of net inspection messages
        float pc_dt;  // dt of phase corr (image shift) messages
        float md_dt;  // dt of marker detection messages
    };

    // stores whether each of the stereovision, image shift and net inspection modules holds new information
    struct SensorUpdated
    {
        bool stv_updated; // whether stereovision module has new data
        bool ni_updated;  // whether net inspection module has new data
        bool pc_updated;  // whether phase corr module has new data
        bool md_updated;  // whether marker detection module has new data
    };

    // Parameters
    AP_Int8  _net_shape;
    AP_Int16 _tracking_distance;
    AP_Int32 _tracking_meshcount;
    AP_Int8  _control_var;
    AP_Float _tracking_velocity;
    AP_Int8  _velocity_ctrl;
    AP_Float _phase_shift_cutoff_freq;
    AP_Float _throttle_speed;
    AP_Float _phase_shift_thr_dist;

    uint32_t _last_stereo_update_ms = 0;
    uint32_t _last_mesh_data_update_ms = 0;
    uint32_t _last_phase_corr_update_ms = 0;
    uint32_t _last_marker_detection_update_ms = 0;

    float _nettr_velocity;
    bool _nettr_toggle_velocity = false;
    float _nettr_direction = 1.0f;
    float _nettr_default_vel_factor;
    float _nettr_opt_flow_vel_factor;

    float _phase_shift_sum_x;
    float _phase_shift_sum_y;

    // stores the accumulated yaw value at start of each new 360 degrees loop. ROV should switch directions of lateral movements after 360 degrees scan
    float _initial_yaw;    

    // the heading (yaw angle in radians) at start of net tracking
    float _home_yaw;

    // the altitude at start of net tracking
    float _home_altitude;

    // stores the absolute distance, the image has travelled along y-axis. This is stored when switching to throttle-state to track the distance that the image is "moving upwards" during throttling
    float _initial_phase_shift_sumy;

    // if distance error is small enough, use the stereovision heading data to always orientate the vehicle normal to the faced object surface
    bool _perform_att_ctrl;

    // filter
    LowPassFilterVector2f _phase_shift_filt;  // low pass filtering of phase shift input

    // net tracking State
    State _state;

    // 360 degrees loop progress in percent
    float _loop_progress;

    // sensor information
    SensorIntervals _sensor_intervals;
    SensorUpdated _sensor_updates;

public:
};
