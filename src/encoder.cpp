#include <Arduino.h>
#include "encoder.h"
#include "motor.h"

#define MICROSECONDS_PER_SECOND 1000000
#define ENCODER_PPS             663.0

// --------------- Encoder Reverse Macros ---------------
//** If defined, the veclocity for a particular encoder will be reversed **
#define LEFT_ENCODER_REVERSE
//#define RIGHT_ENCODER_REVERSE

// --------------- Pin Definitions ---------------
#define encoder_1_A             4   // Encoder pin-assignments
#define encoder_1_B             5
#define encoder_2_A             2
#define encoder_2_B             3

volatile int64_t encoderPos1 =  0;
int64_t lastReportedPos1 =      1;
volatile int64_t encoderPos2 =  0;
int64_t lastReportedPos2 =      1;

boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;

double velocity_left =          0;
double velocity_right =         0;

void init_encoder()
{   
    pinMode(49, OUTPUT);                                    // Pin 49 is used to enable IO power
    digitalWrite(49, 1);                                    // Enable IO power on main CPU board

    pinMode(encoder_1_A, INPUT);                            // Set encoder pins as inputs
    pinMode(encoder_1_B, INPUT); 
    pinMode(encoder_2_A, INPUT); 
    pinMode(encoder_2_B, INPUT); 

    attachInterrupt(encoder_1_A, doEncoder1A, CHANGE);      // Set up an interrupt for each encoder
    attachInterrupt(encoder_2_A, doEncoder2A, CHANGE);  
}

/**
 * @brief Calculates wheel velocity in pulse per second
 * ==> There are 663 pulses per rotation
 */
void calc_encoder_velocty()
{
    static int64_t last_position_left = 0;                       // Last position value of the encoder
    static int64_t last_position_right = 0;
    static uint64_t last_calc_time = 0;                     // Last time this code ran (micro-seconds)
    int64_t delta_calc_time = micros() - last_calc_time;

    last_calc_time = micros();


    velocity_left = MICROSECONDS_PER_SECOND * (encoderPos1 - last_position_left)  / delta_calc_time;
    velocity_right = MICROSECONDS_PER_SECOND * (encoderPos2 - last_position_right) / delta_calc_time;
    
    last_position_left = encoderPos1;
    last_position_right = encoderPos2;
    
    //Serial.printf("V_left: %.3f\n", velocity_left);

    velocity_left  *= 2*PI / 663.0; // Convert to rad/s
    velocity_right *= 2*PI / 663.0;
}

void update_endoder()
{
    calc_encoder_velocty();
}

/**
 * @brief Returns [rad/s]
 */
double get_velocity_left()
{
    #ifdef LEFT_ENCODER_REVERSE
    return -1*velocity_left;
    #endif // LEFT_ENCODER_REVERSE
    return velocity_left;
}

/**
 * @brief Returns [rad/s]
 */
double get_velocity_right()
{
    #ifdef RIGHT_ENCODER_REVERSE
    return -1*velocity_right;
    #endif // RIGHT_ENCODER_REVERSE
    return velocity_right;
}

// Interrupt on A changing state
void doEncoder1A()
{
    // Test transition
    A_set1 = digitalRead(encoder_1_A) == HIGH;
    // and adjust counter + if A leads B
    encoderPos1 += (A_set1 != B_set1) ? +1 : -1;

    B_set1 = digitalRead(encoder_1_B) == HIGH;
    // and adjust counter + if B follows A
    encoderPos1 += (A_set1 == B_set1) ? +1 : -1;
}


// Interrupt on A changing state
void doEncoder2A()
{
    // Test transition
    A_set2 = digitalRead(encoder_2_A) == HIGH;
    // and adjust counter + if A leads B
    encoderPos2 += (A_set2 != B_set2) ? +1 : -1;
    
    B_set2 = digitalRead(encoder_2_B) == HIGH;
    // and adjust counter + if B follows A
    encoderPos2 += (A_set2 == B_set2) ? +1 : -1;
}