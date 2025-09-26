#pragma once

#include <Arduino.h>
#include <VoltageSensor.h>

void drive(float u_l, float u_r)
{
    int left_dir = u_l > 0;
    int left_pwm = constrain(255 * fabs(u_l) / get_v_batt(), 0, 255);

    int right_dir = u_r < 0;
    int right_pwm = constrain(255 * fabs(u_r) / get_v_batt(), 0, 255);

    digitalWrite(7, left_dir);
    analogWrite(9, left_pwm);

    digitalWrite(8, right_dir);
    analogWrite(10, right_pwm);
}
