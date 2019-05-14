#pragma once

#include "AP_StereoVision_Backend.h"

class AP_StereoVision_MAV : public AP_StereoVision_Backend
{

public:
    // constructor
    AP_StereoVision_MAV(AP_StereoVision &frontend);

    // consume VISION_POSITION_DELTA MAVLink message
    void handle_msg(const mavlink_message_t *msg) override;
};
