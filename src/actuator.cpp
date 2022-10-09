#include <Arduino.h>
#include "actuator.h"
#include <Servo.h>

#define MAIN_SERVO_PIN      32
#define P_SERVO_LEFT_PIN    30
#define P_SERVO_RIGHT_PIN   31

#define ELECTROMAG_PIN      14      // A0  - CON68
#define FR_ELECTROMAG_PIN   26      // A12 - CON74
#define WEIGHT_DROP_PIN     24      // A10 - CON72

Servo main_servo;
Servo pincer_servo_left;
Servo pincer_servo_right;

void init_actuators()
{
    init_servos();
    init_electromag();
    init_weight_drop();
    set_actuators_default_values();
}

void init_servos()
{
    main_servo.attach(MAIN_SERVO_PIN);
    pincer_servo_left.attach(P_SERVO_LEFT_PIN);
    pincer_servo_right.attach(P_SERVO_RIGHT_PIN);
}

void set_actuators_default_values()
{
    set_main_servo_angle(MAX_ANGLE_MAIN + 50);
    set_electromag(false);
    set_pincer_servos_angle(MIN_ANGLE_PINCER);
}

void init_weight_drop(void)
{
    pinMode(WEIGHT_DROP_PIN, OUTPUT);
}

void set_main_servo_angle(int32_t angle)
{
    main_servo.write(90 + angle);
}

void set_pincer_servos_angle(int32_t angle)
{
    pincer_servo_left.write(90 + angle + ANGLE_OFFSET + LEFT_OFFSET);
    pincer_servo_right.write(90 - angle + ANGLE_OFFSET);
}

void init_electromag()
{
    pinMode(ELECTROMAG_PIN, OUTPUT);
    pinMode(FR_ELECTROMAG_PIN, OUTPUT);
}

void set_front_electromag(boolean set)
{
    digitalWrite(FR_ELECTROMAG_PIN, set);
}

void set_electromag(boolean set)
{
    digitalWrite(ELECTROMAG_PIN, set);
}

void set_weight_drop(boolean set)
{
    digitalWrite(WEIGHT_DROP_PIN, set);
}
