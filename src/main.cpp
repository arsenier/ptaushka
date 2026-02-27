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

  maze_set_wall(Vec2{0, 0}, Maze::CellWalls{Maze::WALL, Maze::WALL, Maze::OPEN, Maze::OPEN});
  maze_set_wall(Vec2{1, 1}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::WALL});
  maze_set_wall(Vec2{0, 2}, Maze::CellWalls{Maze::OPEN, Maze::OPEN, Maze::OPEN, Maze::WALL});
  maze_set_wall(Vec2{1, 3}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::WALL});
  maze_set_wall(Vec2{2, 2}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::WALL});
  maze_set_wall(Vec2{3, 1}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::WALL, Maze::OPEN});
  maze_set_wall(Vec2{2, 0}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::OPEN});
  maze_set_wall(Vec2{4, 0}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::OPEN});
  maze_set_wall(Vec2{5, 1}, Maze::CellWalls{Maze::WALL, Maze::OPEN, Maze::OPEN, Maze::WALL});
  maze_set_wall(Vec2{5, 3}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::WALL});
  maze_set_wall(Vec2{3, 3}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::OPEN});
  maze_set_wall(Vec2{4, 2}, Maze::CellWalls{Maze::OPEN, Maze::WALL, Maze::OPEN, Maze::WALL});

  Serial.println();
  draw_maze(MAZE_WIDTH, MAZE_HEIGHT);

  uint32_t time0 = micros();
  solver_init();
  solver_set_start_goal(Vec2{0, 0}, Vec2{10, 10});
  uint32_t time1 = micros();

  Serial.print("Solver init: ");
  Serial.print(time1 - time0);
  Serial.println(" us");

  bool solved = false;
  do
  {
    uint32_t time0s = micros();
    solved = solver_solve();
    uint32_t time1s = micros();
    Serial.print("Solver solve: ");
    Serial.print(time1s - time0s);
    Serial.println(" us");
  } while (!solved);

  draw_maze_with_solver(MAZE_WIDTH, MAZE_HEIGHT);

  while (1)
    ;
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
