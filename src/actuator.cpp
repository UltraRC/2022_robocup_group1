#include <Arduino.h>
#include "actuator.h"
#include <Servo.h>

Servo main_servo;
Servo pincer_servo_left;
Servo pincer_servo_right;
Servo rear_servo;

void init_actuators()
{
    init_main_servo();
    init_pincer_servos();
    init_rear_servo();
    init_electromag();
    init_weight_drop();
    set_actuators_default_values();
}

void set_actuators_default_values()
{
    // set_main_servo_angle((MIN_ANGLE_MAIN + MAX_ANGLE_MAIN) / 2.0);
    set_main_servo_angle(MAX_ANGLE_MAIN+50);
    set_electromag(false);
    set_pincer_servos_angle(MIN_ANGLE_PINCER-15);
}

void init_weight_drop(void)
{
    pinMode(WEIGHT_DROP_PIN, OUTPUT);
}

void init_main_servo()
{
    main_servo.attach(MAIN_SERVO_PIN);
}

void set_main_servo_angle(int32_t angle)
{
    main_servo.write(90 + angle);
}

void init_pincer_servos()
{
    pincer_servo_left.attach(P_SERVO_LEFT_PIN);
    pincer_servo_right.attach(P_SERVO_RIGHT_PIN);
}

void set_pincer_servos_angle(int32_t angle)
{
    pincer_servo_left.write(90 + angle);
    pincer_servo_right.write(90 - angle);
}

void init_rear_servo()
{
    rear_servo.attach(REAR_SERVO_PIN);
}

void init_electromag()
{
    pinMode(ELECTROMAG_PIN, OUTPUT);
}

void set_electromag(boolean set)
{
    if (set)
    {
        digitalWrite(ELECTROMAG_PIN, HIGH);
    }
    else
    {
        digitalWrite(ELECTROMAG_PIN, LOW);
    }
}

void set_weight_drop(boolean set)
{
    digitalWrite(WEIGHT_DROP_PIN, set);
}
