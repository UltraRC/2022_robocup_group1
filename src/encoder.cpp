#include <Arduino.h>

#define encoder_1_A  19 // Encoder pin-assignments
#define encoder_1_B  29
#define encoder_2_A  18
#define encoder_2_B  28

void doEncoder1A();
void doEncoder2A();

volatile unsigned int encoderPos1 = 0;
unsigned int lastReportedPos1 = 1;
volatile unsigned int encoderPos2 = 0;
unsigned int lastReportedPos2 = 1;

boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;

void encoder_init()
{   
    pinMode(49, OUTPUT);                 //Pin 49 is used to enable IO power
    digitalWrite(49, 1);                 //Enable IO power on main CPU board

    pinMode(encoder_1_A, INPUT);       //Set encoder pins as inputs
    pinMode(encoder_1_B, INPUT); 
    pinMode(encoder_2_A, INPUT); 
    pinMode(encoder_2_B, INPUT); 

    attachInterrupt(4, doEncoder1A, CHANGE);  //Set up an interrupt for each encoder
    attachInterrupt(5, doEncoder2A, CHANGE);

    Serial.begin(9600);                       //Set up serial communications    
}


void encoder_update()
{
    //If there has been a change in value of either encoder then print the 
    //  encoder values to the serial port
    if ((lastReportedPos1 != encoderPos1)||(lastReportedPos2 != encoderPos2)) 
    {
        Serial.print("Index:");
        Serial.print(encoderPos1, DEC);
        Serial.print(":");
        Serial.print(encoderPos2, DEC);
        Serial.println();
        lastReportedPos1 = encoderPos1;
        lastReportedPos2 = encoderPos2;
    }

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