#include <Arduino.h>
#include <Servo.h>
#include "motor.h"
#include "PPMReader.h"
//big change

#define RC_IN 23
#define NUM_CHANNELS 6

#define MAIN_SERVO_PIN 32
#define P_SERVO_LEFT_PIN 30
#define P_SERVO_RIGHT_PIN 31
#define REAR_SERVO_PIN 33

#define ELECTROMAG_PIN 14

#define MAX_ANGLE_LEFT 80
#define MIN_ANGLE_LEFT -8
#define MIN_ANGLE_MAIN 50
#define MAX_ANGLE_MAIN -85

Servo main_servo;
Servo pincer_servo_left;
Servo pincer_servo_right;
Servo rear_servo;

PPMReader ppm(RC_IN, NUM_CHANNELS);
int32_t channels[NUM_CHANNELS];
boolean input;

void update_channels();
//functions to fill
void set_main_servo_angle(int32_t angle);
void init_main_servo();
void init_pincer_servos();
void init_rear_servo();
void init_electromag();
void set_electromag(boolean input);
void set_pincer_servos_angle(int32_t angle);
void set_rear_servo_angle(int32_t angle);
//void get_distance_IR_sensor();


void setup()
{
    init_motors();
    init_main_servo();
    init_pincer_servos();
    init_rear_servo();
    init_electromag();
    Serial.begin(9600);
}

void loop()
{
    update_channels();
    update_motors();

    int32_t left_speed = -0.5*channels[1] + 0.5*channels[2];
    int32_t right_speed = 0.5*channels[1] + 0.5*channels[2];
    left_speed  *= 2;
    right_speed *= 2;
    set_motor_velocity_left(map(left_speed, -100, 100, -45, 45));
    set_motor_velocity_right(map(right_speed, -100, 100, -45, 45));
    
    
    int32_t main_servo_angle = map(channels[0], -100, 100, MIN_ANGLE_MAIN, MAX_ANGLE_MAIN);
    Serial.printf("Angle: %d\n", main_servo_angle);
    int32_t pincer_servos_angle = map(channels[3], -100, 100, MIN_ANGLE_LEFT, MAX_ANGLE_LEFT);
    //int32_t rear_servo_angle = (channels[3] + 100)*90/200;
    if (channels[4] > 50) {
        input = 0;
    } else {
        input = 1;
    }
    set_main_servo_angle(main_servo_angle);
    set_electromag(input);
    set_pincer_servos_angle(pincer_servos_angle);
    //set_rear_servo_angle(rear_servo_angle);
}

void update_channels()
{
    for(uint8_t i=0; i<NUM_CHANNELS; i++)
    {
        int32_t val = ppm.latestValidChannelValue(i+1, 0);
        val = map(val, 1000, 2000, -100, 100); // Change from us-PWM to %
        channels[i] = val;
    }
}

void init_main_servo()
{
    main_servo.attach(MAIN_SERVO_PIN);
}

void set_main_servo_angle(int32_t angle)
{
        main_servo.write(90+angle);
}

void init_pincer_servos()
{
    pincer_servo_left.attach(P_SERVO_LEFT_PIN);
    pincer_servo_right.attach(P_SERVO_RIGHT_PIN);
}

void set_pincer_servos_angle(int32_t angle)
{
   pincer_servo_left.write(90+angle);
   pincer_servo_right.write(90-angle);
}

void init_rear_servo()
{
    rear_servo.attach(REAR_SERVO_PIN);
}

void set_rear_servo_angle(int32_t angle)
{
    rear_servo.write(angle);
}

void init_electromag()
{
    pinMode(ELECTROMAG_PIN, OUTPUT);
}

void set_electromag(boolean input)
{
    if (input) {
        digitalWrite(ELECTROMAG_PIN, HIGH);
    } else {
        digitalWrite(ELECTROMAG_PIN, LOW);
    }
}