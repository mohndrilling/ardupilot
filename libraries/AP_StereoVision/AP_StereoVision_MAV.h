#pragma once

#include "AP_StereoVision_Backend.h"

class AP_StereoVision_MAV : public AP_StereoVision_Backend
{

public:
    // constructor
    AP_StereoVision_MAV(AP_StereoVision &frontend);

    // consume STEREO_VISION_ODOM MAVLink message
    void handle_stereo_vision_msg(const mavlink_message_t *msg) override;
    void handle_net_inspection_msg(const mavlink_message_t *msg) override;
    void handle_phase_correlation_msg(const mavlink_message_t *msg) override;
    void handle_marker_detection_msg(const mavlink_message_t *msg) override;
};
