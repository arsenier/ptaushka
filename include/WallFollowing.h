#pragma once

#include "ASMR.h"

#define WF_LEFT_REFERENCE 50
#define WF_RIGHT_REFERENCE 50
#define WF_LEFT_THRESHOLD 20
#define WF_RIGHT_THRESHOLD 20

float wf_kp = 0.6;

// 6 = k*10 => k = 6/10

float wf_straight_tick(SensorData data)
{
    float left = data.dist_left;

    float err = WF_LEFT_REFERENCE - left;

    float theta_i0 = err * wf_kp;

    return theta_i0;
}
