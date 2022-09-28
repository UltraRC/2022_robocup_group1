#include <Arduino.h>
#include "wall_follow.h"
#include "sensor.h"
#include "PID.h"
#include "motor.h"


#define PID_UPDATE_RATE     10000 // [Hz]

#define KP                  1
#define KI                  0
#define KD                  0
#define TAU                 0
#define CTRL_LIM_MIN        -1000000
#define CTRL_LIM_MAX        +1000000
#define INT_LIM_MIN         -200
#define INT_LIM_MAX         +200

#define WALL_FOLLOW_DIST    500 // [mm]
#define WALL_FOLLOW_VEL     8   // [rad/s]

PIDController wall_follow_pid;

void init_wall_follow()
{
    wall_follow_pid = {KP,KI,KI,TAU,CTRL_LIM_MIN,CTRL_LIM_MAX,INT_LIM_MIN,INT_LIM_MAX, PID_UPDATE_RATE/1000000.0};
    PIDController_Init(&wall_follow_pid);
}

void update_wall_follow()
{
    static uint64_t last_time = 0;
    int64_t delta_time;
    delta_time = micros()-last_time;

    double phi = sin(50*PI/180);                // Angle modifier
    double alpha = 0.90;                        // Recursive filter constant

    static double last_A = 0;
    static double last_B = 0;

    double A, B, measured_wall_distance;
    
    if(delta_time >= PID_UPDATE_RATE)
    {
        last_time = micros();                   // Reset timer

        A = alpha*last_A + (1-alpha)*get_sensor_right();
        B = alpha*last_B + (1-alpha)*get_sensor_front_right()*phi;


        last_A = A;
        last_B = B;

        measured_wall_distance = (A+B) / 2.0;     // Measure distance from the wall

        // Serial.printf("A: %f, B: %f, phi:%f, distance: %f\n", A, B, phi, measured_wall_distance);

        float control = PIDController_Update(&wall_follow_pid, WALL_FOLLOW_DIST, measured_wall_distance);

        control /= 1000;

        set_motor_velocity_left(WALL_FOLLOW_VEL + control);
        set_motor_velocity_right(WALL_FOLLOW_VEL - control);

        Serial.printf("%f\t%f\n", control, measured_wall_distance);
    }

}