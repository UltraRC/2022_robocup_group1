#include <Arduino.h>
#include "PPMReader.h"
#include "motor.h"
#include "actuator.h"
#include "sensor.h"
#include "wall_follow.h"

#define RC_IN 23
#define NUM_CHANNELS 6

PPMReader ppm(RC_IN, NUM_CHANNELS);
int32_t channels[NUM_CHANNELS];

typedef enum
{
    start = 0,
    pickup_weight,
    approach_weight,
    face_weight_right,
    face_weight_left,
    follow_wall
} state_t;

void start_state();
void pickup_weight_state();
void approach_weight_state();
void face_weight_right_state();
void face_weight_left_state();
void state_tracker();
void update_channels();
void print_state(state_t state);
void wall_follow_state();

state_t state = start; // System state variable. E.g. Weight-collection state, navigation state.
uint64_t time_since_last_state_transition = 0;
uint32_t weight_dis[NUM_WEIGHT_DETECTION_DIRECTIONS];
uint32_t turn_to_weight_time = 2400;    // [ms]
weight_detect_direction_t direction_to_turn = fowards;

void setup()
{
    delay(1500);
    Serial.begin(9600);
    // while(!Serial)
    // {
    //     ;       // Wait until serial connection
    // }
    Serial.printf("Serial initiated!!\n\n");

    init_motors();
    init_actuators();
    init_sensors();
    init_wall_follow();
}

void loop()
{
    update_sensors();
    state_tracker();   // Calculates the time since the last state-transition !!MAY NOT BE NECESSARY!!
    update_channels(); // Updates the array of channel values from the remote control
    update_motors();   // Runs the PID loop using the encoders and controls the speed of the motors

    static uint64_t ls = 0;
    if(millis() - ls > 500)
    {
        ls = millis();
        uint32_t distance_l, distance_m, distance_r;
        uint32_t count_l = get_consecutive_weight_detections(left_foward,  &distance_l);
        uint32_t count_m = get_consecutive_weight_detections(fowards,      &distance_m);
        uint32_t count_r = get_consecutive_weight_detections(right_foward, &distance_r);

        Serial.printf("cl:%04u, dl:%04u\tcm:%04u, dm:%04u\tcr:%04u, dr:%04u\t\n", count_l, distance_l, count_m, distance_m, count_r, distance_r);
        // Serial.printf("Distance_middle: %u, count: %u\n", distance_m, count_m);
    }


    switch (state)
    {
    case start:
        start_state();
        state = follow_wall;
        break;

    case pickup_weight:
        pickup_weight_state();
        break;

    case approach_weight:
        approach_weight_state();
        break;

    case face_weight_right:
        face_weight_right_state();
        break;

    case face_weight_left:
        face_weight_left_state();
        break;

    case follow_wall:
        wall_follow_state();
        break;

    default:
        break;
    }
}

void wall_follow_state()
{
    int8_t reverse_velocity = -50;
    uint16_t reverse_time = 300;
    int8_t turn_vel = -100;
    uint16_t turn_time = 500;
    uint16_t wall_distance = 370;
    uint16_t num_dects = 10;
    side_t wall_to_follow = left_wall;

    bool weight_detected_left       = get_consecutive_weight_detections(left_foward, &weight_dis[left_foward])      > 3;
    bool weight_detected_right      = get_consecutive_weight_detections(right_foward, &weight_dis[right_foward])    > 3;
    bool weight_detected_middle     = get_consecutive_weight_detections(fowards, &weight_dis[fowards])              > num_dects;

    typedef enum
    {
        start_task = 0,
        rev,
        turn
    } task_t;

    // Serial.printf("Front_bottom_sensor: %u\n", get_sensor_distance(front_bottom));

    static task_t current_task = start_task;
    static task_t last_task = start_task;

    static uint64_t last_task_transition_time = 0;

    if (current_task != last_task)
    {
        last_task = current_task;
        last_task_transition_time = millis();
    }

    uint64_t time_since_task_transition = millis() - last_task_transition_time;
    

    // ------------------- State-machine tasks below ---------------------
    switch (current_task)
    {
    case start_task:
        update_wall_follow(wall_to_follow);

        
        if (weight_detected_left)
        {
            state = face_weight_left;
            direction_to_turn = left_foward;
            break;
        }

        if (weight_detected_right)
        {
            state = face_weight_right;
            direction_to_turn = right_foward;
            break;
        }

        if (weight_detected_middle)
        {
            state = approach_weight;
            direction_to_turn = fowards;
            break;
        }
            
        if (get_sensor_distance(front_top) < wall_distance)
        {
            current_task = rev;
        }
        break;

    case rev:
        set_motor_velocity_left(reverse_velocity);
        set_motor_velocity_right(reverse_velocity);

        if (time_since_task_transition > reverse_time)
        {
            current_task = turn; // Reset current_task before changing state
        }
        break;

    case turn:
        set_motor_velocity_left(turn_vel * wall_to_follow);
        set_motor_velocity_right(-turn_vel * wall_to_follow);

        if (time_since_task_transition > turn_time)
        {
            current_task = start_task; // TODO This code is cursed, please fix it
        }
        break;

    default:
        break;
    }
}

void start_state()
{
    int32_t left_speed, right_speed;

    // set_actuators_default_values();
    left_speed = -0.5 * channels[1] + 0.5 * channels[2];
    right_speed = 0.5 * channels[1] + 0.5 * channels[2];
    left_speed *= 2;
    right_speed *= 2;
    set_motor_velocity_left(map(left_speed, -100, 100, -45, 45));
    set_motor_velocity_right(map(right_speed, -100, 100, -45, 45));

    // if(weight_detected())
    // {
    //     set_main_servo_angle(MAX_ANGLE_MAIN+15);
    //     if(channels[4] < -50) state = pickup_weight;
    // } else
    // {
    //     set_actuators_default_values();
    //     state = spin;
    // }

    // if(channels[5] < -50)
    // {
    //     set_weight_drop(true);
    // } else
    // {
    //     set_weight_drop(false);
    // }

    // if(weight_detected() && channels[4] < -50) state = pickup_weight;      // Transition state if button is pressed on remote

    // if (channels[4] > 50) state = pickup_weight;      // Transition state if button is pressed on remote
}

void pickup_weight_state()
{
    typedef enum
    {
        start_task = 0,
        task1,
        task2,
        task3,
        task4,
        task5,
        task6
    } task_t;

    static task_t current_task = start_task;
    static task_t last_task = start_task;

    static uint64_t last_task_transition_time = 0;

    if (current_task != last_task)
    {
        last_task = current_task;
        last_task_transition_time = millis();
    }

    uint64_t time_since_task_transition = millis() - last_task_transition_time;

    // ------------------- State-machine tasks below ---------------------
    switch (current_task)
    {
    case start_task: // Default task
        // set_pincer_servos_angle(MIN_ANGLE_PINCER);
        current_task = task1;
        break;

    case task1: // Move fowards
        set_motor_velocity_left(17);
        set_motor_velocity_right(17);
        if (time_since_task_transition > 1300)
        {
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            current_task = task2;
        }
        break;

    case task2: // Close then open pincers
        set_pincer_servos_angle(MAX_ANGLE_PINCER);
        set_motor_velocity_left(-15);
        set_motor_velocity_right(-15);
        if (time_since_task_transition > 1000)
        {
            set_main_servo_angle(MAX_ANGLE_MAIN + 12);
            set_pincer_servos_angle(MIN_ANGLE_PINCER);
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            current_task = task3;
        }
        break;

    case task3: // Move fowards, slowly
        set_front_electromag(true); // Turn on front electromag
        set_motor_velocity_left(10);
        set_motor_velocity_right(10);
        if (time_since_task_transition > 1700)
        {
            set_pincer_servos_angle(MAX_ANGLE_PINCER);
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            current_task = task4;
        }
        break;

    case task4: // Turn on electro-magnet, and lower main-servo
        set_electromag(true);
        set_front_electromag(false); // Turn off front electromag
        set_main_servo_angle(MAX_ANGLE_MAIN);
        set_pincer_servos_angle(MIN_ANGLE_PINCER);
        if (time_since_task_transition > 450)
        {
            set_main_servo_angle(MIN_ANGLE_MAIN);
            current_task = task5;
        }
        break;

    case task5: // Drop weight on the ramp and move backwards
        set_motor_velocity_left(-10);
        set_motor_velocity_right(-10);
        if (time_since_task_transition > 1500)
        {
            set_electromag(false);
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            current_task = task6;
        }
        break;

    case task6:
        set_actuators_default_values();
        if (time_since_task_transition > 500)
        {
            current_task = start_task;
            state = start;
        }
        break;

    default:
        break;
    }
}

void approach_weight_state()
{
    //** Add an arbitrary number of states to complete complete a sequence of events **//
    typedef enum
    {
        start_task = 0,
        task1,
    } task_t;
    
    uint32_t mrad = 15;     // [rad/s]
    uint32_t v = mrad * 55; // [mm/s]
    uint32_t time =  weight_dis[direction_to_turn] * 1000 / v; 
    set_motor_velocity_left(mrad);
    set_motor_velocity_right(mrad);

    static task_t current_task = start_task;
    static task_t last_task = start_task;

    static uint64_t last_task_transition_time = 0;

    if (current_task != last_task)
    {
        last_task = current_task;
        last_task_transition_time = millis();
    }

    uint64_t time_since_task_transition = millis() - last_task_transition_time;

    // ------------------- State-machine tasks below ---------------------
    switch (current_task)
    {
    case start_task:
        current_task = task1;
        break;

    case task1:

        if (time_since_task_transition > time)
        {
            state = pickup_weight;
        }
        break;

    default:
        break;
    }
}

void face_weight_right_state()
{
    //** Add an arbitrary number of states to complete complete a sequence of events **//
    typedef enum
    {
        start_task = 0,
        task1,
    } task_t;

    static task_t current_task = start_task;
    static task_t last_task = start_task;

    static uint64_t last_task_transition_time = 0;

    if (current_task != last_task)
    {
        last_task = current_task;
        last_task_transition_time = millis();
    }

    uint64_t time_since_task_transition = millis() - last_task_transition_time;

    // ------------------- State-machine tasks below ---------------------
    switch (current_task)
    {
    case start_task:
        current_task = task1;
        break;

    case task1:
        //** Some initilization code!! **//
        set_motor_velocity_left(17);
        set_motor_velocity_right(-17);
        if (time_since_task_transition > turn_to_weight_time)
        {
            state = approach_weight;
        }
        break;

    default:
        break;
    }
}

void face_weight_left_state()
{
    //** Add an arbitrary number of states to complete complete a sequence of events **//
    typedef enum
    {
        start_task = 0,
        task1,
    } task_t;

    static task_t current_task = start_task;
    static task_t last_task = start_task;

    static uint64_t last_task_transition_time = 0;

    if (current_task != last_task)
    {
        last_task = current_task;
        last_task_transition_time = millis();
    }

    uint64_t time_since_task_transition = millis() - last_task_transition_time;

    // ------------------- State-machine tasks below ---------------------
    switch (current_task)
    {
    case start_task:
        current_task = task1;
        break;

    case task1:
        //** Some initilization code!! **//
        set_motor_velocity_left(-17);
        set_motor_velocity_right(17);
        if (time_since_task_transition > turn_to_weight_time)
        {
            state = approach_weight;
        }
        break;

    default:
        break;
    }
}

void generic_state()
{
    //** Add an arbitrary number of states to complete complete a sequence of events **//
    typedef enum
    {
        start_task = 0,
        task1,
    } task_t;

    static task_t current_task = start_task;
    static task_t last_task = start_task;

    static uint64_t last_task_transition_time = 0;

    if (current_task != last_task)
    {
        last_task = current_task;
        last_task_transition_time = millis();
    }

    uint64_t time_since_task_transition = millis() - last_task_transition_time;

    // ------------------- State-machine tasks below ---------------------
    switch (current_task)
    {
    case start_task:
        current_task = task1;
        break;

    case task1:
        //** Some initilization code!! **//

        if (time_since_task_transition > 1000) // Allows for non-blocking delays
        {
            //** Do a thing after waiting for 1-second!! **//

            current_task = start_task; // Reset current_task before changing state
            state = start;             // Change state
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

    time_since_last_state_transition = millis() - last_state_transition_time;

    if (state != last_state)
    {
        // print_state(state); // Print the current state
        last_state_transition_time = millis();
        last_state = state;
    }
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

void print_state(state_t state)
{
    switch (state)
    {
    case start:
        Serial.printf("state is: \"start\"\n");
        break;
    case pickup_weight:
        Serial.printf("state is: \"pickup weight\"\n");
        break;

    case follow_wall:
        Serial.printf("state is: \"follow_wall\"\n");
        break;

    case face_weight_left:
        Serial.printf("state is: \"face_weight_left\"\n");
        break;

    case face_weight_right:
        Serial.printf("state is: \"face_weight_right\"\n");
        break;

    case approach_weight:
        Serial.printf("state is: \"approach_weight\"\n");
        break;

    default:
        break;
    }
}