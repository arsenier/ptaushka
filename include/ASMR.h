#pragma once

#include <stdint.h>
#include <Arduino.h>

#include "Config.h"
#include "Odometer.h"
#include "Mixer.h"

struct ASMR_Entry
{
    union
    {
        uint8_t raw;

        uint8_t cyc_type : 2;

        struct
        {
            uint8_t cyc_type : 2;
            uint8_t stidle_mode : 6;
        } stidle;

        struct
        {
            uint8_t cyc_type : 2;
            uint8_t forw_mode : 1;
            uint8_t forw_dist : 5;
        } forw;

        struct
        {
            uint8_t cyc_type : 2;
            uint8_t turn_mode : 2;
            uint8_t turn_source : 1;
            uint8_t turn_angle : 2;
            uint8_t turn_dir : 1;
        } turn;
    };
};

struct SensorData
{
    float odom_S;
    float odom_theta;
    float time;
};

struct CyclogramOutput
{
    float theta_i0;
    float v_0;
    bool is_completed;
};

enum ASMR_CYC : uint8_t
{
    STOP = 0b00000000,
    IDLE = 0b00000001,

    SWD05 = 0b01000001,
    SWD1 = 0b01000010,

    SS90SEL = 0b10010010,
    SS90SER = 0b10010011,
};

ASMR_Entry asmr_prog_buffer[ASMR_PROG_BUFFER_SIZE] = {
    SWD1,
    // SS90SEL,
    // SWD1,
    // SS90SER,
    // SWD05,
    STOP,
};

enum ASMR_CYC_Type
{
    STIDLE = 0b00,
    FORW = 0b01,
    TURN = 0b10
};

size_t asmr_prog_counter = 0;

void asmr_init()
{
    asmr_prog_counter = 0;
}

void asmr_cyc_stidle(CyclogramOutput *output, SensorData data, ASMR_Entry cyc)
{
    output->v_0 = 0;
    output->theta_i0 = 0;

    if (cyc.stidle.stidle_mode == 0)
    {
        output->is_completed = false;
    }
    else
    {
        output->is_completed = true;
    }
}

void asmr_cyc_forw(CyclogramOutput *output, SensorData data, ASMR_Entry cyc)
{
    output->v_0 = MAX_VEL;
    output->theta_i0 = 0;

    // uint8_t dist_half_int = cyc.forw.forw_dist;
    uint8_t dist_half_int = cyc.raw & 0b00011111;

    float dist_mul = 1.0;

    if (cyc.forw.forw_mode == 1)
    {
        dist_mul = M_SQRT2;
    }

    float dist = dist_half_int * 0.5 * CELL_WIDTH * dist_mul;

    // Serial.print("cyc: ");
    // Serial.print(cyc.raw, BIN);
    // Serial.print(" dist_half_int: ");
    // Serial.print(dist_half_int);
    // Serial.print(" dist_mul: ");
    // Serial.print(dist_mul);
    // Serial.print(" dist: ");
    // Serial.println(dist);

    output->is_completed = data.odom_S > dist;
}

void asmr_cyc_turn(CyclogramOutput *output, SensorData data, ASMR_Entry cyc)
{
}

void asmr_tick()
{
    // Read sensors
    odom_tick();

    // Run cyclogram
    SensorData data = {
        .odom_S = odom_get_S(),
        .odom_theta = odom_get_theta(),
        .time = micros(), // !!!
    };

    CyclogramOutput output = {0};

    ASMR_Entry current_cyc = asmr_prog_buffer[asmr_prog_counter];

    // RUN CYCLOGRAM
    uint8_t cyc_type = current_cyc.raw >> 6;

    switch (cyc_type)
    {
    case STIDLE:
        asmr_cyc_stidle(&output, data, current_cyc);
        break;
    case FORW:
        asmr_cyc_forw(&output, data, current_cyc);
        break;
    case TURN:
        asmr_cyc_turn(&output, data, current_cyc);
        break;
    default:
        break;
    }

    if (output.is_completed)
    {
        asmr_prog_counter++;
        odom_reset();
    }

    // Write motors
    mixer_tick(output.v_0, output.theta_i0);
}

size_t asmr_get_prog_counter()
{
    return asmr_prog_counter;
}

ASMR_Entry *asmr_get_prog_buffer()
{
    return asmr_prog_buffer;
}
