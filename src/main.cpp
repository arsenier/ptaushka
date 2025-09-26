#include <Arduino.h>
#include <argviz.h>

#include "VoltageSensor.h"
#include "Motor.h"

int left_u = 0;
int right_u = 0;

SCREEN(volts,
  {
    ROW("Vbatt [mV]: %d", int(get_v_batt() * 1000));
    CLICK_ROW(
      [](CLICK_STATE state){
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
      "left u [V]: %d", left_u
    )
    CLICK_ROW(
      [](CLICK_STATE state){
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
      "right u [V]: %d", right_u
    )
  }
)

void setup()
{
  Serial.begin(115200);

  argviz_init(Serial);
  argviz_registerScreen(0, volts);
  argviz_start();
}

void loop()
{
  drive(left_u, right_u);

  delay(1);
}
