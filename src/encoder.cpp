#include <Arduino.h>
#include "encoder.h"
#include "motor.h"

#define encoder_1_A  2 // Encoder pin-assignments
#define encoder_1_B  3
#define encoder_2_A  4
#define encoder_2_B  5

volatile int64_t encoderPos1 = 0;
 int64_t lastReportedPos1 = 1;
volatile int64_t encoderPos2 = 0;
int64_t lastReportedPos2 = 1;

boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;

double velocity = 0;                                        // 
double set_point = 0;

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

    Serial.begin(9600);                                     // Set up serial communications    
}

/**
 * @brief Calculates wheel velocity in pulse per second
 * ==> There are 663 pulses per rotation
 */
void calc_encoder_velocty()
{
    static int64_t last_position = 0;                       // Last position value of the encoder
    static uint64_t last_calc_time = 0;                     // Last time this code ran (micro-seconds)
    static uint64_t last_print_time = 0;

    int64_t delta_calc_time = micros() - last_calc_time;
    uint64_t delta_print_time = micros() - last_print_time;

    if(delta_calc_time > 10000)
    {
        last_calc_time = micros();
        int64_t delta_position = encoderPos1 - last_position;
        last_position = encoderPos1;
        velocity = 1000000 * delta_position / delta_calc_time;
        velocity /= 663;
    }

    if(delta_print_time > 3000000) // Delay 1s
    {
        //Serial.println(velocity);
        static int state = 1;
        state*=-1;
        if(state > 0)
        {
            set_point = 6;
        } else 
        {
            set_point = 3;
        }
        last_print_time = micros();
    }
}


void update_endoder()
{
    //If there has been a change in value of either encoder then print the 
    //  encoder values to the serial port
    if ((lastReportedPos1 != encoderPos1)||(lastReportedPos2 != encoderPos2)) 
    {
        // Serial.print("Index:");
        // Serial.print(encoderPos1, DEC);
        // Serial.print(":");
        // Serial.print(encoderPos2, DEC);
        // Serial.println();
        lastReportedPos1 = encoderPos1;
        lastReportedPos2 = encoderPos2;
    }

    controller();
}

void controller()
{
    static double cum_error = 0;
    double motor_control;

    double kp = 3;
    double ki = 0.003;

    //double set_point = 1.5; // hz
    double error = set_point - velocity;
    cum_error += error;
    motor_control = kp*error + ki*cum_error;
    set_motor_speed_right(-motor_control);
    //Serial.println(velocity);
    Serial.print("Error: ");        Serial.print(error);        Serial.print("\t");
    Serial.print("c_error: ");      Serial.print(cum_error);    Serial.print("\t");
    Serial.print("Control: ");      Serial.print(motor_control);Serial.print("\n");
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