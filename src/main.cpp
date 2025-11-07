#include <Arduino.h>
#include <argviz.h>

#include "VoltageSensor.h"
#include "Motor.h"
#include "Encoder.h"
#include "Config.h"
#include "VelEstimator.h"
#include "ServoMotor.h"

int left_u = 0;
int right_u = 0;

float left_w0 = 0;
float right_w0 = 0;

SCREEN(volts,
       {
         ROW("Vbatt [mV]: %d", int(vs_get_v_batt() * 1000));
         CLICK_ROW(
             [](CLICK_STATE state)
             {
               switch (state)
               {
               case CLICK_LEFT:
                 left_u--;
                 break;
               case CLICK_RIGHT:
                 left_u++;
                 break;
               case CLICK_DOWN:
                 left_u = 0;
                 break;
               default:
                 break;
               }
             },
             "left u [V]: %d", left_u)
         CLICK_ROW(
             [](CLICK_STATE state)
             {
               switch (state)
               {
               case CLICK_LEFT:
                 right_u--;
                 break;
               case CLICK_RIGHT:
                 right_u++;
                 break;
               case CLICK_DOWN:
                 right_u = 0;
                 break;
               default:
                 break;
               }
             },
             "right u [V]: %d", right_u)
       })

SCREEN(encoders,
       {
         ROW("Left phi[mrad]: %d", (int)(enc_l_get_phi() * 1000));
         ROW("Left wf[mrad/s]: %d", (int)(ve_l_get_w_est_f() * 1000));
         ROW("Right phi[mrad]: %d", (int)(enc_r_get_phi() * 1000));
         ROW("Right wf[mrad/s]: %d", (int)(ve_r_get_w_est_f() * 1000));
       })

SCREEN(servos,
       {CLICK_ROW([](CLICK_STATE state)
                  {
                     switch (state)
                     {
                     case CLICK_LEFT:
                       left_w0--;
                       break;
                     case CLICK_RIGHT:
                       left_w0++;
                       break;
                     case CLICK_DOWN:
                       left_w0 = 0;
                       break;
                     default:
                       break;
                     } },
                  "left_w0: %s", String(left_w0).c_str())
            ROW("left_w: %s", String(ve_l_get_w_est_f()).c_str())
                CLICK_ROW([](CLICK_STATE state)
                          {
                     switch (state)
                     {
                     case CLICK_LEFT:
                       right_w0--;
                       break;
                     case CLICK_RIGHT:
                       right_w0++;
                       break;
                     case CLICK_DOWN:
                       right_w0 = 0;
                       break;
                     default:
                       break;
                     } },
                          "right_w0: %s", String(right_w0).c_str())
                    ROW("right_w: %s", String(ve_r_get_w_est_f()).c_str())})

void setup()
{
  Serial.begin(115200);

  m_init();
  vs_init();
  enc_l_init();
  enc_r_init();

  interrupts();

  argviz_init(Serial);
  argviz_registerScreen(0, volts);
  argviz_registerScreen(1, encoders);
  argviz_registerScreen(2, servos);
  argviz_start();
}

void loop()
{
  // Timer
  static uint32_t timer = micros();
  while (micros() - timer < Ts_us)
    ;
  timer = micros();

  // Sense

  // Plan

  // Act
  servo_tick(left_w0, right_w0);
}
