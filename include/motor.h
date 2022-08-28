#ifndef MOTOR_H
#define MOTOR_H

// --------------- Functions ---------------
void init_motors();
void update_motors();
void set_motor_speed_left(int32_t speed);
void set_motor_speed_right(int32_t speed);

void set_motor_velocity_left(double set);
void set_motor_velocity_right(double set);

#endif // MOTOR_H