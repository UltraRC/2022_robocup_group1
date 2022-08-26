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
    //Serial.begin(9600);
}

void loop()
{
    update_channels();
    //Serial.print(channels[1]);
    //Serial.print("\t");
    //Serial.print(channels[2]);
    //Serial.print("\n");
    delay(20);
    int32_t left_speed = 1*channels[1] - 1*channels[2];
    int32_t right_speed = -1*channels[1] - 1*channels[2];
    left_speed *= 2;
    right_speed *= 2;
    set_motor_speed_left(left_speed);
    set_motor_speed_right(right_speed);
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