#include <Arduino.h>
#include "PPMReader.h"
#include "motor.h"
#include "actuator.h"

#define RC_IN 23
#define NUM_CHANNELS 6

PPMReader ppm(RC_IN, NUM_CHANNELS);
int32_t channels[NUM_CHANNELS];

void start_state();
void pickup_weight_state();

void state_tracker();
void update_channels();
typedef enum
{
    start = 0,
    pickup_weight
} state_t;

state_t state = start;      // System state variable. E.g. Weight-collection state, navigation state.
uint64_t time_since_last_state_transition = 0;

void setup()
{
    init_motors();
    init_actuators();
    // Serial.begin(9600);
}

void loop()
{
    state_tracker();        // Calculates the time since the last state-transition !!MAY NOT BE NECESSARY!!
    update_channels();      // Updates the array of channel values from the remote control
    update_motors();        // Runs the PID loop using the encoders and controls the speed of the motors

    switch (state)
    {
    case start:
        start_state();
        break;

    case pickup_weight:
        pickup_weight_state();
        break;

    default:
        break;
    }
}

void start_state()
{
    int32_t left_speed, right_speed;

    set_actuators_default_values();
    left_speed = -0.5 * channels[1] + 0.5 * channels[2];
    right_speed = 0.5 * channels[1] + 0.5 * channels[2];
    left_speed *= 2;
    right_speed *= 2;
    set_motor_velocity_left(map(left_speed, -100, 100, -45, 45));
    set_motor_velocity_right(map(right_speed, -100, 100, -45, 45));

    if (channels[4] > 50)
        state = pickup_weight;      // Transition state if button is pressed on remote
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
        task5
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

    case task2:                                 // Close, and then open pincers
        set_pincer_servos_angle(MAX_ANGLE_PINCER);
        if (time_since_task_transition > 600)
        {
            set_main_servo_angle(MAX_ANGLE_MAIN+15);
            set_pincer_servos_angle(MIN_ANGLE_PINCER);
            current_task = task3;
        }
        break;

    case task3:                                 // Move fowards, slowly
        set_motor_velocity_left(10);
        set_motor_velocity_right(10);
        if (time_since_task_transition > 1200)
        {
            set_motor_velocity_left(0);
            set_motor_velocity_right(0);
            current_task = task4;
        }
        break;

    case task4:                                 // Turn on electro-magnet, and lower main-servo
        set_electromag(true);
        set_main_servo_angle(MAX_ANGLE_MAIN);
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
            current_task = start_task;
            state = start;
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