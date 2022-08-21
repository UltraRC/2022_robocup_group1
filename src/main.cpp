#include <Arduino.h>
#include "motor.h"
#include "PPMReader.h"

#include <PWMServo.h> // REMOVE

#define L_ARM_PIN 30
#define R_ARM_PIN 31

#define RC_IN 23
#define NUM_CHANNELS 6

PWMServo arm_left;
PWMServo arm_right;
PWMServo thicc_motor;

PPMReader ppm(RC_IN, NUM_CHANNELS);
int32_t channels[NUM_CHANNELS];

void update_channels();

void setup()
{
    init_motors();
    arm_left.attach(L_ARM_PIN); // Servo pins : 30 - 33
    arm_right.attach(R_ARM_PIN);
    thicc_motor.attach(32);
    Serial.begin(9600);
}

void loop()
{
    update_channels();
    //Serial.print(channels[1]);
    //Serial.print("\t");
    //Serial.print(channels[2]);
    //Serial.print("\n");

    for(uint8_t i=0; i<6; i++)
    {
        Serial.print("Channel "); Serial.print(i); Serial.print(": "); Serial.print(channels[i]); Serial.print("\t");
    }
    Serial.println();

    delay(20);
    int32_t left_speed = 0.5*channels[1] - 0.5*channels[2];
    int32_t right_speed = -0.5*channels[1] - 0.5*channels[2];
    left_speed *= 2;
    right_speed *= 2;
    set_motor_speed_left(left_speed);
    set_motor_speed_right(right_speed);

    int32_t servo_set = map(channels[0], -100, 100, 0, 180);
    //arm_left.write(servo_set);
    //arm_right.write(180-servo_set);
    //thicc_motor.write(90);

    //Serial.println(servo_set);
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