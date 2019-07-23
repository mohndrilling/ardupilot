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
#define AP_NETTRACKING_CTRL_VAR_DEFAULT ControlVar::ctrl_distance

class AP_NetTracking {
public:
    AP_NetTracking( AP_AHRS_View &ahrs,
                    AC_AttitudeControl_Sub& attitude_control,
                    AC_PosControl_Sub& pos_control,
                    AP_StereoVision& stereo_vision) :
        _ahrs(ahrs),
        _attitude_control(attitude_control),
        _pos_control(pos_control),
        _stereo_vision(stereo_vision)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    // Empty destructor to suppress compiler warning
    virtual ~AP_NetTracking() {}
    
    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // perform net tracking: calls attitude and pos controller to maintain orthonormal heading and distance
    void perform_net_tracking(float &forward_out, float &lateral_out);

protected:

    // References to external libraries
    const AP_AHRS_View&         _ahrs;
    AC_AttitudeControl_Sub&     _attitude_control;
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

    // Parameters
    AP_Int8 _net_shape;
    AP_Int16 _tracking_distance;
    AP_Int32 _tracking_meshcount;
    AP_Int8 _control_var;
    AP_Float _tracking_velocity;

    uint32_t last_stereo_update_ms = 0;
    float net_track_vel;
    bool nettr_toggle_velocity = false;
    float nettr_direction = 1.0f;

public:
};
