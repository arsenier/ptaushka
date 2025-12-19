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

    SWD = 0b01000000,
    SWD05 = 0b01000001,
    SWD1 = 0b01000010,

    SS90SEL = 0b10010010,
    SS90SER = 0b10010011,

    TURN_CYC = 0b10000000,
};

#define FROM_STRAIGHT 0
#define FROM_DIAG 0b00001000

#define SHORTEST 0
#define EXPLORE 0b00010000
#define IN_PLACE 0b00100000

#define T45 0
#define T90 0b00000010
#define T135 0b00000100
#define T180 0b00000110

#define TURN_LEFT 0b00000000
#define TURN_RIGHT 0b00000001

ASMR_Entry asmr_prog_buffer[ASMR_PROG_BUFFER_SIZE] = {
    // SWD1,
    TURN_CYC + SHORTEST + FROM_STRAIGHT + T90 + TURN_LEFT,
    // SWD1,
    TURN_CYC + SHORTEST + FROM_STRAIGHT + T90 + TURN_LEFT,
    // SWD1,
    TURN_CYC + SHORTEST + FROM_STRAIGHT + T90 + TURN_LEFT,
    // SWD1,
    TURN_CYC + SHORTEST + FROM_STRAIGHT + T90 + TURN_LEFT,
    STOP,
};

enum ASMR_CYC_Type : uint8_t
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

/*
1.0.x.x.x.x.x.x
    ^^^ ^ ^^^ |- Направление поворота
     |  |  |  |- 0: LEFT
     |  |  |  `- 1: RIGHT
     |  |  |
     |  |  |- Угол поворота
     |  |  |- 00: 45deg
     |  |  |- 01: 90deg
     |  |  |- 10: 135deg
     |  |  `- 11: 180deg
     |  |
     |  |- Откуда приходим в поворот (из прямой или диагонали)
     |  |- 0: STRAIGHT
     |  `- 1: DIAG
     |
     |- Вид поворота 90
     |- 00: Shortest
     |- 01: Explore
     `- 10: In-place
*/

const float turn_smooth_distances[][2] = {
    [0] = {0, 0}, // 45deg
    [1] = {CELL_WIDTH - TURN_RADIUS_SHORTEST,
           CELL_WIDTH *M_SQRT1_2 - TURN_RADIUS_SHORTEST}, // 90deg
    [2] = {0, 0},                                         // 135deg
    [3] = {0, 0},                                         // 180deg
};

void asmr_cyc_turn(CyclogramOutput *output, SensorData data, ASMR_Entry cyc)
{
    uint8_t turn_type = (cyc.raw & 0b00110000) >> 4;   // Тип поворота
    uint8_t turn_source = (cyc.raw & 0b00001000) >> 3; // Откуда приходим в поворот
    uint8_t turn_angle = (cyc.raw & 0b00000110) >> 1;  // Угол поворота
    uint8_t turn_dir = cyc.raw & 0b00000001;           // Направление поворота (0 - лево, 1 - право)

    uint8_t turn_dest = (~turn_angle & 0b1) ^ turn_source; // Куда приходим из поворота

    float turn_delta_theta = (45 + 45 * turn_angle) * DEG_TO_RAD;

    // Serial.print("cyc type: ");
    // Serial.print(turn_type);
    // Serial.print(" source: ");
    // Serial.print(turn_source);
    // Serial.print(" angle: ");
    // Serial.print(turn_angle);
    // Serial.print(" dir: ");
    // Serial.print(turn_dir);
    // Serial.print(" dest: ");
    // Serial.println(turn_dest);

    if (turn_type == 0)
    {
        float first_dist = turn_smooth_distances[turn_angle][turn_source];
        float turn_dist = turn_delta_theta * TURN_RADIUS_SHORTEST;
        float second_dist = turn_smooth_distances[turn_angle][turn_dest];

        if (data.odom_S < first_dist)
        {
            asmr_cyc_forw(output, data, ASMR_Entry{(FORW << 6) | (turn_source << 5)});
        }
        else if (data.odom_S < first_dist + turn_dist)
        // else if (fabs(data.odom_theta) < turn_delta_theta)
        {
            output->v_0 = MAX_VEL;
            float turn_vel = MAX_VEL / TURN_RADIUS_SHORTEST;
            output->theta_i0 = turn_dir ? -MAX_ANG_VEL : MAX_ANG_VEL;
        }
        else if (data.odom_S < first_dist + turn_dist + second_dist)
        {
            asmr_cyc_forw(output, data, ASMR_Entry{(FORW << 6) | (turn_dest << 5)});
        }

        output->is_completed = data.odom_S > first_dist + turn_dist + second_dist;
    }

    // Serial.print("TURNING: ");
    // Serial.print("output.v_0: ");
    // Serial.print(output->v_0);
    // Serial.print(" output.theta_i0: ");
    // Serial.println(output->theta_i0);
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

    // Serial.print("output.v_0: ");
    // Serial.print(output.v_0);
    // Serial.print(" output.theta_i0: ");
    // Serial.println(output.theta_i0);
}

size_t asmr_get_prog_counter()
{
    return asmr_prog_counter;
}

ASMR_Entry *asmr_get_prog_buffer()
{
    return asmr_prog_buffer;
}
