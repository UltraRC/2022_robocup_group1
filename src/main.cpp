#include <Arduino.h>
#include "motor.h"
#include "PPMReader.h"

#define RC_IN 23
#define NUM_CHANNELS 6

PPMReader ppm(RC_IN, NUM_CHANNELS);
int32_t channels[NUM_CHANNELS];

void update_channels();

void setup()
{
    init_motors();
    Serial.begin(9600);
}

void loop()
{
    update_channels();
    update_motors();
    int32_t left_speed = -0.5*channels[1] + 0.5*channels[2];
    int32_t right_speed = 0.5*channels[1] + 0.5*channels[2];
    left_speed  *= 2;
    right_speed *= 2;
    set_motor_velocity_left(map(left_speed, -100, 100, -45, 45));
    set_motor_velocity_right(map(right_speed, -100, 100, -45, 45));
}

void update_channels()
{
    for(uint8_t i=0; i<NUM_CHANNELS; i++)
    {
        int32_t val = ppm.latestValidChannelValue(i+1, 0);
        val = map(val, 1000, 2000, -100, 100); // Change from us-PWM to %
        channels[i] = val;
    }
}