#include <Arduino.h>
#include "wall_follow.h"
#include "sensor.h"
#include "PID.h"
#include "motor.h"

#define PID_UPDATE_PERIOD 10000 // [us]

#define KP 60
#define KI 0.02
#define KD 0
#define TAU 0
#define CTRL_LIM_MIN -1000000
#define CTRL_LIM_MAX +1000000
#define INT_LIM_MIN -30000
#define INT_LIM_MAX +30000

// #define WALL_FOLLOW_DIST    400 // [mm]
int16_t WALL_FOLLOW_DIST = 0;
#define WALL_FOLLOW_VEL 35 // [rad/s]

PIDController wall_follow_pid;

void init_wall_follow()
{
    wall_follow_pid = {KP, KI, KI, TAU, CTRL_LIM_MIN, CTRL_LIM_MAX, INT_LIM_MIN, INT_LIM_MAX, PID_UPDATE_PERIOD / 1000000.0};
    PIDController_Init(&wall_follow_pid);
}

void update_wall_follow(side_t side)
{
    sensor_t sensor = front_right_top;
    if (side == left_wall)
    {
        sensor = front_left_top;
    }

    // WALL_FOLLOW_DIST = 450 + 200*sin(millis()/500.0);
    WALL_FOLLOW_DIST = 500;

    static uint64_t last_time = 0;
    int64_t delta_time;
    delta_time = micros() - last_time;

    double phi = sin(50 * PI / 180); // Angle modifier
    double alpha = 0.90;             // Recursive filter constant    // TODO possibly not necessary

    static double last_A = 0;
    static double last_B = 0;

    double A, B, measured_wall_distance;

    if (delta_time >= PID_UPDATE_PERIOD)
    {
        last_time = micros(); // Reset timer

        A = alpha * last_A + (1 - alpha) * get_sensor_distance(right);
        B = alpha * last_B + (1 - alpha) * get_sensor_distance(sensor) * phi;

        last_A = A;
        last_B = B;

        // measured_wall_distance = (A+B) / 2.0;     // Measure distance from the wall
        // measured_wall_distance = A;
        measured_wall_distance = B;

        // measured_wall_distance = min(measured_wall_distance, get_sensor_distance(front_bottom));

        // Serial.printf("A: %f, B: %f, phi:%f, distance: %f\n", A, B, phi, measured_wall_distance);

        float control = PIDController_Update(&wall_follow_pid, WALL_FOLLOW_DIST, measured_wall_distance);
        control /= 1000; // Control needs to be on the order of radians per second, so it is scaled

        set_motor_velocity_left(WALL_FOLLOW_VEL - control * (int)side);
        set_motor_velocity_right(WALL_FOLLOW_VEL + control * (int)side);
    }
}