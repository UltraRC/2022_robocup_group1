#include <Arduino.h>
#include <Servo.h>
#include "motor.h"
#include "PPMReader.h"
// big change

#define RC_IN               +23
#define NUM_CHANNELS        +6

#define MAIN_SERVO_PIN      +32
#define P_SERVO_LEFT_PIN    +30
#define P_SERVO_RIGHT_PIN   +31
#define REAR_SERVO_PIN      +33

#define ELECTROMAG_PIN      +14

#define MIN_ANGLE_PINCER    +80
#define MAX_ANGLE_PINCER    -5
#define MIN_ANGLE_MAIN      +50
#define MAX_ANGLE_MAIN      -76

Servo main_servo;
Servo pincer_servo_left;
Servo pincer_servo_right;
Servo rear_servo;

PPMReader ppm(RC_IN, NUM_CHANNELS);
int32_t channels[NUM_CHANNELS];

typedef enum {
    start = 0,
    pickup_weight
} state_t;

state_t state = start;
uint64_t time_since_state_transition = 0;

void state_tracker();
void pickup_weight_state();
void set_actuators_default_values();

void update_channels();
// functions to fill
void set_main_servo_angle(int32_t angle);
void init_main_servo();
void init_pincer_servos();
void init_rear_servo();
void init_electromag();
void set_electromag(boolean input);
void set_pincer_servos_angle(int32_t angle);
void set_rear_servo_angle(int32_t angle);
// void get_distance_IR_sensor();


void setup()
{
    init_motors();
    init_main_servo();
    init_pincer_servos();
    init_rear_servo();
    init_electromag();
    set_actuators_default_values();
    // Serial.begin(9600);
}

void loop()
{
    state_tracker();        // Calculates the time since the last state-transition
    update_channels();
    update_motors();
       
    int32_t left_speed, right_speed;

    switch (state)
    {
    case start:
        set_actuators_default_values();
        left_speed = -0.5 * channels[1] + 0.5 * channels[2];
        right_speed = 0.5 * channels[1] + 0.5 * channels[2];
        left_speed *= 2;
        right_speed *= 2;
        set_motor_velocity_left(map(left_speed, -100, 100, -45, 45));
        set_motor_velocity_right(map(right_speed, -100, 100, -45, 45));

        // main_servo_angle = map(channels[0], -100, 100, MIN_ANGLE_MAIN, MAX_ANGLE_MAIN);
        // pincer_servos_angle = map(channels[3], -100, 100, MIN_ANGLE_PINCER, MAX_ANGLE_PINCER);
        if (channels[4] > 50)
        {
            state = pickup_weight;
        }
        break;

    case pickup_weight:
        pickup_weight_state();
        break;
    
    default:
        break;
    }    
}

void pickup_weight_state()
{
    typedef enum
    {
        start_state = 0,
        task1,
        task2,
        task3,
        task4,
        task5
    } sub_state_t;

    static sub_state_t sub_state = start_state;
    static sub_state_t last_sub_state = start_state;
    
    static uint64_t last_task_transition = 0;

    if(sub_state != last_sub_state)
    {
        last_sub_state = sub_state;
        last_task_transition = millis();
    }

    uint64_t time_since_task_transition = millis() - last_task_transition;

    // ------------------- State-machine tasks below ---------------------
    // Serial.printf("sub-state = %d\n", sub_state);
    switch (sub_state)
    {
    case start_state:
        sub_state = task1;
        break;

    case task1:                                 // Move fowards
        set_motor_velocity_left(17);
        set_motor_velocity_right(17);
        if(time_since_task_transition > 1300)
        {
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            sub_state = task2;
        }
        break;
    
    case task2:                                 // Close, and then open pincers
        set_pincer_servos_angle(MAX_ANGLE_PINCER);
        if(time_since_task_transition > 700)
        {
            set_pincer_servos_angle(MIN_ANGLE_PINCER);
            sub_state = task3;
        }
        break;

    case task3:                                 // Move fowards, slowly
        set_motor_velocity_left(10);
        set_motor_velocity_right(10);
        if(time_since_task_transition > 1200)
        {
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            sub_state = task4;
        }
        break;
    
    case task4:                                 // Turn on electro-magnet, and lower main-servo
        set_electromag(true);
        set_main_servo_angle(MAX_ANGLE_MAIN);
        if(time_since_task_transition > 500)
        {
            set_main_servo_angle(MIN_ANGLE_MAIN);
            sub_state = task5;
        }
        break;
    
    case task5:                                 // Drop weight on the ramp and move backwards
        set_motor_velocity_left(-10);
        set_motor_velocity_right(-10);
        if(time_since_task_transition > 1500)
        {
            set_electromag(false);
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            sub_state = start_state;
            state = start;
        }
        break;
    
    default:
        break;
    }
}

void state_tracker()
{
    static state_t last_state = start;
    static uint64_t last_state_transition_time = 0;
    
    time_since_state_transition = millis() - last_state_transition_time;
    
    if(state != last_state)
    {
        last_state_transition_time = millis();
        last_state = state;
    }
}

void set_actuators_default_values()
{
    // set_main_servo_angle((MIN_ANGLE_MAIN + MAX_ANGLE_MAIN) / 2.0);
    set_main_servo_angle(MAX_ANGLE_MAIN+15);
    set_electromag(false);
    set_pincer_servos_angle(MIN_ANGLE_PINCER);
}

void update_channels()
{
    for (uint8_t i = 0; i < NUM_CHANNELS; i++)
    {
        int32_t val = ppm.latestValidChannelValue(i + 1, 0);
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