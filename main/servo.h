#ifndef SERVO_H
#define SERVO_H

#define SERVO_PULSE_GPIO_0             1        // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_1             2        // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_2             3        // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

void servo_tasks_init();
void fivetimesInterpolation(int myservo, float thetai, float thetaf, float t);
void put_servo_data(int servo_num, float start, float end, float T);

#endif // SERVO_H
