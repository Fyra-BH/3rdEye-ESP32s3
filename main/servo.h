#ifndef SERVO_H
#define SERVO_H

#define SERVO_PULSE_GPIO_0             5        // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_1             2        // GPIO connects to the PWM signal line
#define SERVO_PULSE_GPIO_2             3        // GPIO connects to the PWM signal line

void servo_tasks_init();
void fivetimesInterpolation(int myservo, float thetai, float thetaf, float t);
void put_servo_data(int servo_num, float start, float end, float T);

#endif // SERVO_H
