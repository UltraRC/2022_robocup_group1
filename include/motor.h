#ifndef MOTOR_H
#define MOTOR_H

#define MIN_US 1010 // Motor controller PWM on-time
#define MAX_US 2000

// ******* Uncomment macros to enable features *******

// --------------- Motor Configuration ---------------
#define L_SERVO_PIN 0
#define R_SERVO_PIN 1

#define LEFT_MOTOR_REVERSE
//#define RIGHT_MOTOR_REVERSE

// --------------- Functions ---------------
void init_motors();
void set_motor_speed_left(int32_t speed);
void set_motor_speed_right(int32_t speed);

#endif // MOTOR_H