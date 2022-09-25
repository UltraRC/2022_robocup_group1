#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>
#include "sensor.h"

/*******************************************************
 ***************** TOF - SENSOR CONFIG *****************
 *******************************************************/
#define VL53L0X_ADDRESS_START 0x30      // Addresses to start indexing sensor addresses
#define VL53L1X_ADDRESS_START 0x35

const uint8_t sensorCount = 1;          // Number of TOF sensors in system

const uint8_t xshutPinsL0[8] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint8_t xshutPinsL1[8] = {4, 5, 6, 7};

const byte SX1509_ADDRESS = 0x3F;       // Adress for io-expander
SX1509 io;                              // Create an SX1509 object to be used throughout

VL53L0X sensorsL0[sensorCount];
VL53L1X sensorsL1[sensorCount];

uint16_t sensor_L0_values[sensorCount];
uint16_t sensor_L1_values[sensorCount];

bool is_weight_detected = false;
// *****************************************************

void init_sensors()
{
    init_tof_sensors();
}

void update_sensors(void)
{
    static uint64_t last_time = 0;

    if(millis()-last_time > 100)
    {
        update_tof_sensors();
        last_time = millis();
    }
}

void init_tof_sensors()
{
    io.begin(SX1509_ADDRESS);

    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    // Disable/reset all sensors by driving their XSHUT pins low.
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        io.pinMode(xshutPinsL0[i], OUTPUT);
        io.digitalWrite(xshutPinsL0[i], LOW);
        io.pinMode(xshutPinsL1[i], OUTPUT);
        io.digitalWrite(xshutPinsL1[i], LOW);
    }

    // L0 Enable, initialize, and start each sensor, one by one.
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        io.digitalWrite(xshutPinsL0[i], HIGH);
        delay(10);

        sensorsL0[i].setTimeout(500);
        if (!sensorsL0[i].init())
        {
            Serial.print("Failed to detect and initialize sensor L0 ");
            Serial.println(i);
            while (1)
               ;
        }

        // Each sensor must have its address changed to a unique value other than
        // the default of 0x29 (except for the last one, which could be left at
        // the default). To make it simple, we'll just count up from 0x2A.
        sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

        sensorsL0[i].startContinuous(50);
    }

    // L1 Enable, initialize, and start each sensor, one by one.
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        io.digitalWrite(xshutPinsL1[i], HIGH);
        delay(10);

        sensorsL1[i].setTimeout(500);
        if (!sensorsL1[i].init())
        {
            Serial.print("Failed to detect and initialize sensor L1 ");
            Serial.println(i);
            while (1)
                ;
        }

        // Each sensor must have its address changed to a unique value other than
        // the default of 0x29 (except for the last one, which could be left at
        // the default). To make it simple, we'll just count up from 0x2A.
        sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

        sensorsL1[i].startContinuous(20);
    }
}

void update_tof_sensors()
{
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        sensor_L0_values[i] = sensorsL0[i].readRangeContinuousMillimeters();
        if (sensorsL0[i].timeoutOccurred()) Serial.println(" TIMEOUT");
    }

    for (uint8_t i = 0; i < sensorCount; i++)
    {
        sensor_L1_values[i] = sensorsL1[i].readRangeContinuousMillimeters();
        if (sensorsL1[i].timeoutOccurred()) Serial.println(" TIMEOUT");
    }

    const double VL53L0X_max_distance = 950;    // [mm]
    double delta = 40;                         // [mm]

    double L0X_sample = sensorsL0[0].readRangeContinuousMillimeters();
    double L1X_sample = sensorsL1[0].readRangeContinuousMillimeters();

    // double L0X_sample = sensorsL0[0].readRangeSingleMillimeters();
    // double L1X_sample = sensorsL1[0].readRangeSingleMillimeters();

    L0X_sample = L0X_sample > VL53L0X_max_distance ? VL53L0X_max_distance : L0X_sample;

    static int32_t consecutive_weight_detections = 0;
    const int32_t consecutive_weight_detections_threshold = 3;

    if(L1X_sample - L0X_sample > delta && L0X_sample < VL53L0X_max_distance)
    {
        consecutive_weight_detections++;
    } else
    {
        consecutive_weight_detections = 0;
    }

    if (consecutive_weight_detections >= consecutive_weight_detections_threshold)
    {
        is_weight_detected = true;
    } else
    {
        is_weight_detected = false;
    }
    

    Serial.printf("%f\t%f\t%f\n", L0X_sample, L1X_sample, L1X_sample-L0X_sample);
}

bool weight_detected()
{
    return is_weight_detected;
}