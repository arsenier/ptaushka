#pragma once

#include <Arduino.h>

float get_v_batt()
{
    int BATTERY_VOLTS_RAW = analogRead(A7);
    float BATTERY_VOLTS_V = BATTERY_VOLTS_RAW * 5.0 / 1024;
    float V_BATT_V = 2 * BATTERY_VOLTS_V;
    return V_BATT_V;
}
