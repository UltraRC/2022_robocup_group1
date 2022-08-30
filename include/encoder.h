#ifndef ENCODER_H
#define ENCODER_H

void doEncoder1A();
void doEncoder2A();

void calc_encoder_velocty();
void controller();

void init_encoder();
void update_endoder();

double get_velocity_left();
double get_velocity_right();

#endif // ENCODER_H