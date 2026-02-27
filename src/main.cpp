#include <Arduino.h>
#include <argviz.h>

#include "VoltageSensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "Config.h"
#include "VelEstimator.h"
#include "ServoMotor.h"
#include "Mixer.h"
#include "Screens.h"
#include "Odometer.h"
#include "ASMR.h"
#include "DistSensors.h"

#include "Maze.h"
#include "MazeDrawer.h"

#include "Tests.h"

void setup()
{
  Serial.begin(115200);

  m_init();
  vs_init();
  enc_l_init();
  enc_r_init();
  asmr_init();
  dist_init();
  maze_init();

  interrupts();

  argviz_init(Serial);
  argviz_registerScreen(0, volts);
  argviz_registerScreen(1, encoders);
  argviz_registerScreen(2, servos);
  argviz_registerScreen(0, mixer);
  argviz_registerScreen(4, asmr);
  // argviz_start();

  // test_maze();
  test_navigator();
}

void loop()
{
  // Timer
  static uint32_t timer = micros();
  while (micros() - timer < Ts_us)
    ;
  timer = micros();

  // Sense
  // odom_tick();

  // enc_l_tick();
  // enc_r_tick();
  // ve_l_tick(enc_l_get_phi());
  // ve_r_tick(enc_r_get_phi());

  // Plan

  // Act
  // servo_tick(left_w0, right_w0);
  // mixer_tick(v_0, theta_i0);
  asmr_tick();
}
