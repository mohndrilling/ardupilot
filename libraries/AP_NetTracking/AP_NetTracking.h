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

#define AP_NETTRACKING_NETSHAPE_DEFAULT NetShape::FishFarm
#define AP_NETTRACKING_DISTANCE_DEFAULT 50
#define AP_NETTRACKING_INITIAL_NET_DISTANCE_TOLERANCE_DEFAULT 10.0f
#define AP_NETTRACKING_MESH_CNT_DEFAULT 200
#define AP_NETTRACKING_DETECTING_NET_FORWARD_THRUST_DEFAULT 0.15f
#define AP_NETTRACKING_VELOCITY_DEFAULT 0.0f
#define AP_NETTRACKING_OPTFLOW_VELOCITY_FACTOR 2.0f
#define AP_NETTRACKING_DEFAULT_VELOCITY_FACTOR 0.8f
#define AP_NETTRACKING_USE_OPT_MARKER_DEFAULT 0
#define AP_NETTRACKING_CTRL_VAR_DEFAULT ControlVar::ctrl_distance
#define AP_NETTRACKING_VEL_CTRL_DEFAULT 1
#define AP_NETTRACKING_OPT_FLOW_VERTICAL_DIST_DEFAULT 200
#define AP_NETTRACKING_OPT_FLOW_CUTOFF_FREQ_DEFAULT 0.2f
#define AP_NETTRACKING_DETECT_NET_YAW_FILT_CUTOFF_FREQ_DEFAULT 0.5f
#define AP_NETTRACKING_DETECT_NET_YAWRATE_FILT_CUTOFF_FREQ_DEFAULT 0.5f
#define AP_NETTRACKING_START_TRACKING_DEPTH_DEFAULT 100.0f
#define AP_NETTRACKING_FINISH_TRACKING_DEPTH_DEFAULT 300.0f
#define AP_NETTRACKING_CLIMBING_RATE_CMS_DEFAULT 10.0f

#define AP_NETTRACKING_AUTO_LEVEL_POST_DELAY 2000
#define AP_NETTRACKING_ADJUSTED_BY_OPERATOR_POST_DELAY 0
#define AP_NETTRACKING_APPROACHING_INIT_ALTITUDE_POST_DELAY 2000
#define AP_NETTRACKING_DETECTING_NET_POST_DELAY 0
#define AP_NETTRACKING_APPROACHING_NET_POST_DELAY 0
#define AP_NETTRACKING_HOLDING_NET_DISTANCE_POST_DELAY 5000
#define AP_NETTRACKING_SCANNING_POST_DELAY 4000
#define AP_NETTRACKING_THROTTLE_DOWNWARDS_POST_DELAY 2000
#define AP_NETTRACKING_RETURN_HOME_HDG_POST_DELAY 3000
#define AP_NETTRACKING_SURFACING_POST_DELAY 0
#define AP_NETCLEANING_WAITING_AT_TERMINAL_POST_DELAY 0

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
                    _opt_flow_filt(AP_NETTRACKING_OPT_FLOW_CUTOFF_FREQ_DEFAULT),
                    _net_detect_yaw_filt(AP_NETTRACKING_DETECT_NET_YAW_FILT_CUTOFF_FREQ_DEFAULT),
                    _net_detect_yaw_rate_filt(AP_NETTRACKING_DETECT_NET_YAWRATE_FILT_CUTOFF_FREQ_DEFAULT),
                    _nettr_default_vel_factor(AP_NETTRACKING_DEFAULT_VELOCITY_FACTOR),
                    _nettr_opt_flow_vel_factor(AP_NETTRACKING_OPTFLOW_VELOCITY_FACTOR),
                    _loop_progress(-1)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Empty destructor to suppress compiler warning
    virtual ~AP_NetTracking() {}
    
    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // init net tracking
    void init();

    // run: main function running the state machine
    void run(float &forward_out, float &lateral_out, float &throttle_out);

    // finish net tracking after next loop
    void set_return_home();

    // get net tracking state to be sent via mavlink
    uint8_t get_state() { return _current_state->_id; }

    // get 360 degrees loop progress in percent (further sent via mavlink)
    float get_loop_progress() { return _loop_progress; }

    // resets internal variables to default values
    void reset();

protected:
    // a template for a member function of AP_NetTracking, containing the logic of a certain state
    typedef void (AP_NetTracking::*StateLogicFunction) (void);

    // enumeration of all available states of the net tracking state machines
    enum StateID
    {
      Inactive,
      AutoLevel,
      AdjustedByOperator,
      ApproachingInitialAltitude,
      DetectingNetInitially,
      ApproachingNetInitially,
      HoldingNetDistance,
      Scanning,
      ThrottleDownwards,
      ReturnToHomeHeading,
      Surfacing,
      WaitingAtTerminal,

      MAX_NUM_STATES // This has to be the last entry of this enumeration
    };

    // state specification
    // id: referring to the StateID enumeration
    // name: for user information
    // state_logic_func: a function of the nettracking class containing the state logic; called when the state is active
    // post_delay: time in milliseconds which the state remains active before switching to the next state
    // next_stateA: subsequent state, alternative A
    // next_stateB: subsequent state, alternative B
    // next_state: currently each state has a maximum of two possible subsequent states. next_state holds the actual next state (updated in state logic)
    struct State
    {
        const StateID _id;
        const char *_name;
        StateLogicFunction _state_logic_func;
        const uint32_t _post_delay;
        const StateID _next_stateA;
        const StateID _next_stateB;
        StateID _next_state;

        // constructor
        State(StateID id, const char* name, StateLogicFunction f, uint32_t post_delay,
              StateID next_stateA, StateID next_stateB = StateID::Inactive)
            : _id(id), _name(name), _state_logic_func(f), _post_delay(post_delay),
              _next_stateA(next_stateA), _next_stateB(next_stateB), _next_state(next_stateA) {}
    };

    //////////////////////// State Logic Functions ////////////////////////////////////////////
    // inactive: Set output to zero
    void inactive() { set_translational_thrust(0.0f, 0.0f, 0.0f); }

    // auto_level: Smooth transition to levelled orientation
    void auto_level();

    // adjusted_by_operator: Wait for adjustment by operator
    void adjusted_by_operator();

    // approach_initial_altitude: Move to initial altitude where net cleaning is about to start
    void approach_initial_altitude();

    // detect_net: rotate and search for net
    void detect_net();

    // approach_net: move forwards until target distance to the net is reached
    void approach_net();

    // hold_net_distance(): run distance controller and keep initial distance to net
    void hold_net_distance();

    // call distance controller and attitude controller to keep desired distance and heading to net plane
    void scan();

    // throttle_downwards: move to the next tracking lane
    void throttle_downwards();

    // surface: move back to surface while keeping fixed distance and orientation towards net
    void surface();

    // wait_at_terminal: keeping fixed distance to net without translational movement
    void wait_at_terminal();

    ////////////////// Helper Functions //////////////////////////////////////////////////
    // hold_heading_and_distance: keeps desired distance and perpendicular heading w.r.t. the net
    void hold_heading_and_distance(float target_dist);

    // update lateral velocity (either static or based on optical flow controller)
    void update_lateral_out(float target_vel);

    // set_translational_thrust: sets the values for forward, lateral and throttle output
    void set_translational_thrust(float forward, float lateral, float throttle)
    {
        _forward_out = forward;
        _lateral_out = lateral;
        _throttle_out = throttle;
    }

    // detects whether the ROV has performed a full 360 degrees loop
    bool detect_loop_closure();

    //update loop progress (just for monitoring)
    void update_loop_progress();

    // updates the optical flow low pass filter by new measurement values and accumulates the absolute distance, the image has shifted
    void update_opt_flow();

    // set time stamps and update flags of stereo vision readings
    void update_stereo_vision();

    ///////////////// State Machine Setup and Switch State Functions ////////////////////////////////////////////////////
    // setup_state_machines: creates a State struct for each state contained in StateID enumeration
    void setup_state_machines();

    // add_state: add state specification to the array of available states
    void add_state(State * state) {_states[state->_id] = state; }

    // switch_state: switch the state of the state machine
    void switch_state();

    // switch_state_after_post_delay: wait for specified time and switch to target_state afterwards
    void switch_state_after_post_delay();

    // set_state_logic_finished: set timestamp and flag (called when a state has finished its task)
    void set_state_logic_finished();

    // References to external libraries
    const AP_AHRS_View&         _ahrs;
    AC_AttitudeControl_Sub&     _attitude_control;
    AP_InertialNav&             _inav;
    AC_PosControl_Sub&          _pos_control;
    AP_StereoVision&            _stereo_vision;

    //
    enum NetShape
    {
        Planar,
        FishFarm
    };

    enum ControlVar
    {
        ctrl_distance,
        ctrl_meshcount
    };

    // stores time difference (seconds) between each incoming message of stereovision, image shift and net inspection modules
    // updated each loop
    struct SensorIntervals
    {
        float stv_dt; // dt of stereo vision messages
        float ni_dt;  // dt of net inspection messages
        float of_dt;  // dt of optical flow (image shift) messages
        float md_dt;  // dt of marker detection messages
    };

    // stores whether each of the stereovision, image shift and net inspection modules holds new information
    struct SensorUpdated
    {
        bool stv_updated; // whether stereovision module has new data
        bool ni_updated;  // whether net inspection module has new data
        bool of_updated;  // whether optical flow module has new data
        bool md_updated;  // whether marker detection module has new data
    };

    // Parameters
    AP_Int8  _net_shape;
    AP_Int16 _tracking_distance;
    AP_Float _tracking_distance_tolerance;
    AP_Int32 _tracking_meshcount;
    AP_Int8  _control_var;
    AP_Float _tracking_velocity;
    AP_Int8  _velocity_ctrl;
    AP_Int8  _use_optical_marker_termination;
    AP_Float _opt_flow_cutoff_freq;
    AP_Float _opt_flow_vertical_dist;
    AP_Float _start_tracking_depth;
    AP_Float _finish_tracking_depth;
    AP_Float _detect_net_forw_trust;
    AP_Float _manual_adjustment_duration;
    AP_Float _climb_rate;

    uint32_t _last_stereo_update_ms;
    uint32_t _last_mesh_data_update_ms;
    uint32_t _last_opt_flow_update_ms;
    uint32_t _last_marker_detection_update_ms;

    bool _nettr_toggle_velocity;
    float _nettr_direction;
    float _nettr_default_vel_factor;
    float _nettr_opt_flow_vel_factor;

    float _opt_flow_sum_x;
    float _opt_flow_sum_y;

    // stores the accumulated yaw value at start of each new 360 degrees loop. ROV should switch directions of lateral movements after 360 degrees scan
    float _initial_yaw;    

    // the heading (yaw angle in radians) at start of net tracking
    float _home_yaw;

    // the altitude at start of net tracking
    float _home_altitude;

    // stores the absolute distance, the image has travelled along y-axis. This is stored when switching to throttle-state to track the distance that the image is "moving upwards" during throttling
    float _initial_opt_flow_sumy;

    // whether to terminate net tracking after the next loop (true, when a termination marker was detected)
    bool _terminate;

    // filter
    LowPassFilterVector2f _opt_flow_filt;  // low pass filtering of optical flow input

    // 360 degrees loop progress in percent
    float _loop_progress;

    // the desired translational movement of the vehicle
    float _forward_out;
    float _lateral_out;
    float _throttle_out;

    // sensor information
    SensorIntervals _sensor_intervals;
    SensorUpdated _sensor_updates;

    /////////////// state specific variables ////////////////

    // current net cleaning State
    State *_current_state;

    // previous net cleaning State
    State *_prev_state;

    // array containing state specification as pointers to the states, set all elements to null
    State * _states[StateID::MAX_NUM_STATES] = { nullptr };

    // flag is true, when the current state is running for the first time
    bool _first_run;

    // true if the task of the current state is fulfilled
    bool _state_logic_finished;

    // stores time stamp of last state execution
    uint32_t _last_state_execution_ms;

    // stores time stamp of first execution of current state
    uint32_t _first_state_execution_ms;

    // the time stamp of when the task of the current state was fulfilled
    uint32_t _state_logic_finished_ms;

    // debugging timestamps
    uint32_t _last_debug_opt_flow_ms = 0;
    uint32_t _last_debug_dyaw_ms = 0;

    LowPassFilterFloat _net_detect_yaw_filt; // low-pass-filter for yaw measurements during net detection
    LowPassFilterFloat _net_detect_yaw_rate_filt; // low-pass-filter for yaw rate during net detection
    float _net_detect_yaw_rate_dir; // which direction to rotate during net detection (cw: -1, ccw: +1)

public:
};
