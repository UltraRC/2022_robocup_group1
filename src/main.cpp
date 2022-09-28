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
    spin,
    move_forward,
    follow_wall
} state_t;

void start_state();
void pickup_weight_state();
void spin_state();
void move_forward_state();
void state_tracker();
void update_channels();
void print_state(state_t state);
void follow_wall_state();

state_t state = start;      // System state variable. E.g. Weight-collection state, navigation state.
uint64_t time_since_last_state_transition = 0;

void setup()
{
    // Serial.begin(9600);
    // while(!Serial)
    // {
    //     ;       // Wait until serial connection
    // }
    // Serial.printf("Serial initiated!!\n\n");

    init_motors();
    init_actuators();
    init_sensors();
    init_wall_follow();
}

void loop()
{
    update_sensors();
    state_tracker();        // Calculates the time since the last state-transition !!MAY NOT BE NECESSARY!!
    update_channels();      // Updates the array of channel values from the remote control
    update_motors();        // Runs the PID loop using the encoders and controls the speed of the motors

    switch (state)
    {
    case start:
        start_state();
        state = follow_wall;
        break;

    case pickup_weight:
        pickup_weight_state();
        break;
    case spin:
        spin_state();
        break;
    case move_forward:
        move_forward_state();
        break;
    case follow_wall:
        update_wall_follow();
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
    case start_task:                            // Default task
        // set_pincer_servos_angle(MIN_ANGLE_PINCER);
        current_task = task1;
        break;

    case task1:                                 // Move fowards
        set_motor_velocity_left(17);
        set_motor_velocity_right(17);
        if (time_since_task_transition > 1300)
        {
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            current_task = task2;
        }
        break;

    case task2:                                 // Close then open pincers
        set_pincer_servos_angle(MAX_ANGLE_PINCER);
        set_motor_velocity_left(-15);
        set_motor_velocity_right(-15);
        if (time_since_task_transition > 1000)
        {
            set_main_servo_angle(MAX_ANGLE_MAIN+12);
            set_pincer_servos_angle(MIN_ANGLE_PINCER);
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            current_task = task3;
        }
        break;

    case task3:                                 // Move fowards, slowly
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

    case task4:                                 // Turn on electro-magnet, and lower main-servo
        set_electromag(true);
        set_main_servo_angle(MAX_ANGLE_MAIN);
        set_pincer_servos_angle(MIN_ANGLE_PINCER);
        if (time_since_task_transition > 450)
        {
            set_main_servo_angle(MIN_ANGLE_MAIN);
            current_task = task5;
        }
        break;

    case task5:                                 // Drop weight on the ramp and move backwards
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
        if(time_since_task_transition > 500)
        {
            current_task = start_task;
            state = start;
        }
        break;

    default:
        break;
    }
}


void spin_state()
{
    //** Add an arbitrary number of states to complete complete a sequence of events **//
    typedef enum
    {
        start_task = 0,
        task1
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
    
    case task1: //spins robot until weight detected
        //** Some initilization code!! **//
        set_motor_velocity_left(-10);
        set_motor_velocity_right(10);

        if(weight_detected())       // Allows for non-blocking delays
        {
            //** Do a thing after waiting for 1-second!! **//
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);

            current_task = start_task;  // Reset current_task before changing state
            state = pickup_weight;              // Change state
        }
        if (time_since_task_transition > 5000) 
        {
            current_task = start_task;  // Reset current_task before changing state
            state = move_forward;  
        }
        break;
    
    default:
        break;
    }
}

void move_forward_state()
{
    //** Add an arbitrary number of states to complete complete a sequence of events **//
    typedef enum
    {
        start_task = 0,
        task1
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
        //when distance sensors in place, set motor velocity for left and write proportional to distance to wall when below threshold
        set_motor_velocity_left(10);
        set_motor_velocity_right(10);
        if(time_since_task_transition > 2000)       // Allows for non-blocking delays
        {
            //** Do a thing after waiting for 1-second!! **//

            current_task = start_task;  // Reset current_task before changing state
            state = spin;              // Change state
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

        if(time_since_task_transition > 1000)       // Allows for non-blocking delays
        {
            //** Do a thing after waiting for 1-second!! **//

            current_task = start_task;  // Reset current_task before changing state
            state = start;              // Change state
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
        // print_state(state);          // Print the current state
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
    
    default:
        break;
    }
}