#include <Arduino.h>
#include <PWMServo.h>
#include "motor.h"
#include "PID.h"
#include "encoder.h"

#define MIN_US 1010 // Motor controller PWM on-time
#define MAX_US 2000

#define MAX_VELOCITY 55 // Rad/s


// --------------- Motor Configuration ---------------
#define L_SERVO_PIN 0
#define R_SERVO_PIN 1

// ******* Uncomment macros to enable macros *******
//#define LEFT_MOTOR_REVERSE
#define RIGHT_MOTOR_REVERSE

// --------------- PID Motor Control= Configuration ---------------
#define KP_LEFT_MOTOR       0.7
#define KI_LEFT_MOTOR       10
#define KD_LEFT_MOTOR       0
#define TAU_LEFT            0       // Filtering constant for derivative term
#define CTRL_LEFT_LIM_MIN   -100    // Limit control output to a range 
#define CTRL_LEFT_LIM_MAX   +100 
#define INT_LEFT_LIM_MIN    -200    // Limit integrator to range to avoid wind-up
#define INT_LEFT_LIM_MAX    +200

#define KP_RIGHT_MOTOR      0.7
#define KI_RIGHT_MOTOR      10
#define KD_RIGHT_MOTOR      0
#define TAU_RIGHT           0
#define CTRL_RIGHT_LIM_MIN  -100
#define CTRL_RIGHT_LIM_MAX  +100
#define INT_RIGHT_LIM_MIN   -200
#define INT_RIGHT_LIM_MAX   +200

#define COTRL_SMPL_PERIOD   5000   // (200 Hz) TODO Maybe reduce this? It is very high

PWMServo left_motor;
PWMServo right_motor;

PIDController pid_left_motor;
PIDController pid_right_motor;

double left_velocity_set;       // Target velocity
double right_velocity_set;

void init_motors()
{
    init_encoder();

    left_motor.attach(L_SERVO_PIN, MIN_US, MAX_US);
    right_motor.attach(R_SERVO_PIN, MIN_US, MAX_US);
    
    pid_left_motor = {KP_LEFT_MOTOR,KI_LEFT_MOTOR,KD_LEFT_MOTOR,TAU_LEFT,CTRL_LEFT_LIM_MIN,CTRL_LEFT_LIM_MAX,INT_LEFT_LIM_MIN,INT_RIGHT_LIM_MAX,COTRL_SMPL_PERIOD/1000000.0};
    pid_right_motor = {KP_RIGHT_MOTOR,KI_RIGHT_MOTOR,KD_RIGHT_MOTOR,TAU_RIGHT,CTRL_RIGHT_LIM_MIN,CTRL_RIGHT_LIM_MAX,INT_RIGHT_LIM_MIN,INT_RIGHT_LIM_MAX,COTRL_SMPL_PERIOD/1000000.0};
    
    PIDController_Init(&pid_left_motor);
    PIDController_Init(&pid_right_motor);
}

void set_motor_velocity_left(double set)
{
    left_velocity_set = constrain(set, -MAX_VELOCITY, MAX_VELOCITY);
}

void set_motor_velocity_right(double set)
{
    right_velocity_set = constrain(set, -MAX_VELOCITY, MAX_VELOCITY);
}

/**
 * @brief Takes in speed in percent [-100%, 100%]
 */
void set_motor_power_left(int32_t speed)
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
void set_motor_power_right(int32_t speed)
{
    #ifdef RIGHT_MOTOR_REVERSE
    speed*=-1;
    #endif // RIGHT_MOTOR_REVERSE
    speed = constrain(speed, -100, 100);
    int32_t pos = map(speed, -100, 100, 0, 180);
    right_motor.write(pos);
}

void update_motors()
{
    static uint64_t last_time = 0;
    int64_t delta_time;
    double velocity_left, velocity_right;

    delta_time = micros() - last_time;

    if(delta_time >= COTRL_SMPL_PERIOD)
    {
        last_time = micros();       // Reset timer
        
        update_encoder();

        velocity_left = get_velocity_left();
        velocity_right = get_velocity_right();

        float control_left  = PIDController_Update(&pid_left_motor,  left_velocity_set,  velocity_left);
        float control_right = PIDController_Update(&pid_right_motor, right_velocity_set, velocity_right);
      
        set_motor_power_left(control_left);
        set_motor_power_right(control_right);
    }
}