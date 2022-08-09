#include <Arduino.h>
#include <PWMServo.h>
#include "motor.h"

PWMServo left_motor;
PWMServo right_motor;

void init_motors()
{
    left_motor.attach(L_SERVO_PIN, MIN_US, MAX_US);
    right_motor.attach(R_SERVO_PIN, MIN_US, MAX_US);
}

/**
 * @brief Takes in speed in percent [-100%, 100%]
 */
void set_motor_speed_left(int32_t speed)
{
    #ifdef LEFT_MOTOR_REVERSE
    speed*=-1;
    #endif // LEFT_MOTOR_REVERSE
    speed = constrain(speed, -100, 100);
    int32_t pos = map(speed, -100, 100, 0, 180);
    left_motor.write(pos);
}

/**
 * @brief Takes in speed in percent [-100%, 100%]
 */
void set_motor_speed_right(int32_t speed)
{
    #ifdef RIGHT_MOTOR_REVERSE
    speed*=-1;
    #endif // RIGHT_MOTOR_REVERSE
    speed = constrain(speed, -100, 100);
    int32_t pos = map(speed, -100, 100, 0, 180);
    right_motor.write(pos);
}