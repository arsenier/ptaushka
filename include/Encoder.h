#pragma once

#include <Arduino.h>

#define ENC_LEFT_CLK_A_PIN 2
#define ENC_LEFT_B_PIN 4

#define ENC_LEFT_DIR 1 // {-1, 1}

#define ENC_TICK_TO_RAD (2 * M_PI / (48 * 30))

int8_t enc_l_tt[4][4] = {0}; // Transition table

float enc_l_phi = 0;

volatile int enc_l_counter = 0;

void enc_l_interrupt()
{
    int B = digitalRead(ENC_LEFT_B_PIN);
    // 00000000 0000000B

    int CA = digitalRead(ENC_LEFT_CLK_A_PIN);
    // 00000000 0000000c

    int A = CA ^ B;
    // 00000000 0000000B
    // 00000000 0000000c
    //^
    // 00000000 0000000A

    int AB = A << 1 | B;
    // 00000000 0000000A
    //<<1
    // 00000000 000000A0
    //|B
    // 00000000 000000AB

    static int old_AB = AB;

    enc_l_counter += enc_l_tt[old_AB][AB];

    old_AB = AB;
}

void enc_l_init()
{
    pinMode(ENC_LEFT_CLK_A_PIN, INPUT);
    pinMode(ENC_LEFT_B_PIN, INPUT);

    enc_l_tt[0b00][0b01] = ENC_LEFT_DIR;
    enc_l_tt[0b01][0b11] = ENC_LEFT_DIR;
    enc_l_tt[0b11][0b10] = ENC_LEFT_DIR;
    enc_l_tt[0b10][0b00] = ENC_LEFT_DIR;

    enc_l_tt[0b00][0b10] = -ENC_LEFT_DIR;
    enc_l_tt[0b10][0b11] = -ENC_LEFT_DIR;
    enc_l_tt[0b11][0b01] = -ENC_LEFT_DIR;
    enc_l_tt[0b01][0b00] = -ENC_LEFT_DIR;

    attachInterrupt(
        digitalPinToInterrupt(ENC_LEFT_CLK_A_PIN),
        enc_l_interrupt,
        CHANGE
    );
}

int enc_l_get_ticks()
{
    noInterrupts();
    int ret = enc_l_counter;
    interrupts();
    return ret;
}

void enc_l_tick()
{
    noInterrupts();
    int counter = enc_l_counter;
    enc_l_counter = 0;
    interrupts();

    enc_l_phi += ENC_TICK_TO_RAD * counter;
}

float enc_l_get_phi()
{
    return enc_l_phi;
}
