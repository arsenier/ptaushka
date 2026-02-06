## Добавление драйвера датчиков расстояния

1. Взять готовый тестовый код работы с датчиками расстояния
2. Добавить директиву `#pragma once` в начало файла
3. Переименовать функцию `void setup()` в `void dist_init()`
4. Удалить функцию `void loop()`
5. Добавить функции для чтения значений датчиков расстояния

```diff
+#pragma once
#include <Arduino.h>

...

-void setup()
+void dist_init()
{
-   Serial.begin(9600);
    pinMode(EMITTER_F, OUTPUT);
    pinMode(EMITTER_S, OUTPUT);
    digitalWrite(EMITTER_F, 0); // be sure the emitter is off
    digitalWrite(EMITTER_S, 0); // be sure the emitter is off
    analogueSetup();            // increase the ADC conversion speed
    setupSystick();
    updateTime = millis() + updateInterval;
}

+int dist_get_left()
+{
+    noInterrupts(); // Начало критической секции (запрет прерываний)
+    int result = gSensorLeft;
+    interrupts(); // Конец критической секции (включение прерываний)
+    return result;
+}
+
+int dist_get_right()
+{
+    noInterrupts();
+    int result = gSensorRight;
+    interrupts();
+    return result;
+}
+
+int dist_get_fleft()
+{
+    noInterrupts();
+    int result = gFSensorLeft;
+    interrupts();
+    return result;
+}
+
+int dist_get_fright()
+{
+    noInterrupts();
+    int result = gFSensorRight;
+    interrupts();
+    return result;
+}
```

6. Добавить экран для отладки датчиков расстояния

```diff
// Screens.h
#pragma once

#include "argviz.h"
#include "Encoder.h"
#include "VelEstimator.h"
#include "VoltageSensor.h"
#include "Odometer.h"
#include "ASMR.h"
+#include "DistSensors.h"

....

+SCREEN(dist,
+       {
+           ROW("dist_left: %d", dist_get_left());
+           ROW("dist_right: %d", dist_get_right());
+           ROW("dist_fleft: %d", dist_get_fleft());
+           ROW("dist_fright: %d", dist_get_fright());
+       })

```

## Добавление движения вдоль стенок

1. Добавить драйвер датчиков расстояния
2. Обновить структуру `SensorData`, добавив туда поля для хранения значений датчиков расстояния
```diff
struct SensorData
{
    float odom_S;
    float odom_theta;
    float time;
+   int dist_left;
+   int dist_right;
+   int dist_fleft;
+   int dist_fright;
};
```
3. Добавить сохранение текущих значений датчиков расстояния в функцию `asmr_tick()`

```diff
void asmr_tick()
{
    // Read sensors
    odom_tick();

    // Run cyclogram
    SensorData data = {
        .odom_S = odom_get_S(),
        .odom_theta = odom_get_theta(),
        .time = micros(), // !!!
+       .dist_left = dist_get_left(),
+       .dist_right = dist_get_right(),
+       .dist_fleft = dist_get_fleft(),
+       .dist_fright = dist_get_fright(),
    };

    ...
}
```

4. Реализовать регулятор для движения вдоль стенок в циклограмме `asmr_cyc_forw()`

```c
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
```

## Небольшой рефакторинг

Переносим настройки стенок из `WallFollowing.h` в `Config.h`

```diff
// WallFollowing.h

-#define WF_LEFT_REFERENCE 50
-#define WF_RIGHT_REFERENCE 50
-#define WF_LEFT_THRESHOLD 20
-#define WF_RIGHT_THRESHOLD 20
```

```diff
// Config.h

+// Wall parameters
+#define WF_LEFT_REFERENCE 50
+#define WF_RIGHT_REFERENCE 50
+#define WF_LEFT_THRESHOLD 20
+#define WF_RIGHT_THRESHOLD 20
```

Добавим в `SensorData` новые поля для статуса стенки

```diff
// ASMR.h

struct SensorData
{
    float odom_S;
    float odom_theta;
    float time;
    int dist_left;
    int dist_right;
    int dist_fleft;
    int dist_fright;
+   bool is_wall_left;
+   bool is_wall_right;
+   bool is_wall_fleft;
+   bool is_wall_fright;
};

...

void asmr_tick()
{
    // Read sensors
    odom_tick();

    // Run cyclogram
    SensorData data = {
        .odom_S = odom_get_S(),
        .odom_theta = odom_get_theta(),
        .time = micros(), // !!!
        .dist_left = dist_get_left(),
        .dist_right = dist_get_right(),
        .dist_fleft = dist_get_fleft(),
        .dist_fright = dist_get_fright(),
    };

+   data.is_wall_left = data.dist_left > WF_LEFT_THRESHOLD;
+   data.is_wall_right = data.dist_right > WF_RIGHT_THRESHOLD;
+   data.is_wall_fleft = false;
+   data.is_wall_fright = false;

    ...
}
```

## Движение по реальным коридорам лабиринта

При движении по лабиринту возможны ситуации когда стенка есть либо слева, либо справа, либо и там и там, либо их нет ни с одной стороны. В каждой ситуации регулятор соответствующей стенки должен работать только если эта стенка видна.

Первое что мы сделаем - напишем регуляторы для левой и правой стенок.

```diff
-float wf_kp = 0.6;
+float wf_kp_left = 0.6;
+float wf_kp_right = -wf_kp_left;

float wf_straight_tick(SensorData data)
{
    float left = data.dist_left;
+   float right = data.dist_right;

-   float err = WF_LEFT_REFERENCE - left;
+   float err_left = WF_LEFT_REFERENCE - left;
+   float err_right = WF_RIGHT_REFERENCE - right;

-   float theta_i0 = err * wf_kp;
+   float theta_i0_left = err_left * wf_kp_left;
+   float theta_i0_right = err_right * wf_kp_right;

+   float theta_i0 = 0;
+   size_t counter = 0;

+   if(data.is_wall_left)
+   {
+       theta_i0 += theta_i0_left;
+       counter++;
+   }
+   if(data.is_wall_right)
+   {
+       theta_i0 += theta_i0_right;
+       counter++;
+   }

+   if(counter != 0)
+   {
+       theta_i0 /= counter;
+   }

    return theta_i0;
}
```


