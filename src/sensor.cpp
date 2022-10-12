#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include "sensor.h"

#define SENSOR_UPDATE_PERIOD 65 // [ms]
#define HOME_BASE_COLOR_VARIANCE 10 //+/- this num to be within reasonable range

// ****** TOF - SENSOR CONFIG ******
#define VL53L0X_ADDRESS_START 0x30 // Addresses to start indexing sensor addresses
#define VL53L1X_ADDRESS_START 0x35

#define LIMIT_SWITCH_PIN 15

//#define COLOR_SENSOR_ADDRESS  //SCL3M, SDA3M
#define GREEN_R 66
#define GREEN_G 117
#define GREEN_B 60

#define BLUE_R 35
#define BLUE_G 77
#define BLUE_B 135

const uint8_t L0_sensor_count = 2;
const uint8_t L1_sensor_count = 6;

const uint8_t xshutPinsL0[8] = {0, 1};
const uint8_t xshutPinsL1[8] = {2, 3, 4, 5, 6, 7};

const byte SX1509_ADDRESS = 0x3F; // Adress for io-expander
SX1509 io;

VL53L0X sensorsL0[L0_sensor_count]; // Arrays of sensor objects
VL53L1X sensorsL1[L1_sensor_count];

uint16_t sensor_L0_values[L0_sensor_count]; // Arrays of sensor measurments
uint16_t sensor_L1_values[L1_sensor_count];

uint16_t sensor_values[L0_sensor_count + L1_sensor_count]; // TODO this number used to be +1

bool tof_critical_error = false;

// ****** Colour - SENSOR CONFIG ******
uint16_t clear, red, green, blue; // color values
float r, g, b;
uint16_t color_delay_count = 0;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // Create an SX1509 object to be used throughout

// ****** Weight pickup variables ******
bool weight_in_range = false;
bool right_weight_detected = false;
bool left_weight_detected = false;
bool centre_weight_detected = false;

// ****** limit switch variable ******
bool limit_switch_on = false;

// Distances between the top and bottom weight detection sensors
#define LEFT_SENSOR_MOUNTED_OFFSET 50   // [mm]
#define FOWARD_SENSOR_MOUNTED_OFFSET 95 // [mm]
#define RIGHT_SENSOR_MOUNTED_OFFSET 50  // [mm]

uint32_t sensor_mounted_offsets[NUM_WEIGHT_DETECTION_DIRECTIONS] = {
    LEFT_SENSOR_MOUNTED_OFFSET,
    FOWARD_SENSOR_MOUNTED_OFFSET,
    RIGHT_SENSOR_MOUNTED_OFFSET};

uint32_t consecutive_weight_detections[NUM_WEIGHT_DETECTION_DIRECTIONS];
uint32_t weight_distances[NUM_WEIGHT_DETECTION_DIRECTIONS];

void init_sensors()
{
    init_tof_sensors();
    Serial.begin(9600);
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    if (tcs.begin(0x29, &Wire1)) // checking if color sensor working
    {
        Serial.println("Found sensor");
    }
    else
    {
        Serial.println("No TCS34725 found ... check your connections");
    }
}

void update_sensors(void)
{
    static uint64_t last_time = 0;

    if (millis() - last_time > SENSOR_UPDATE_PERIOD)
    {
        last_time = millis();
        update_tof_sensors();
        update_weight_distances();
        update_consecutive_weight_detections();
        update_limit_switch();
        if (color_delay_count < 10) 
        {
            color_delay_count ++;
        } else {
            update_color_sensor();
            Serial.printf("updated color, R: %u G: %u, B: %u", (int)r, (int)g, (int)b);
            color_delay_count = 0;
        }
        
    }
}

void update_limit_switch(void) 
{
    limit_switch_on = !digitalRead(LIMIT_SWITCH_PIN);
}

bool limit_switch(void) 
{
    return limit_switch_on;
}

bool at_blue_base(void)
{
    return (r < BLUE_R + HOME_BASE_COLOR_VARIANCE &&
    r > BLUE_R - HOME_BASE_COLOR_VARIANCE &&
    g < BLUE_G + HOME_BASE_COLOR_VARIANCE &&
    g > BLUE_G - HOME_BASE_COLOR_VARIANCE &&
    b < BLUE_B + HOME_BASE_COLOR_VARIANCE &&
    b > BLUE_B - HOME_BASE_COLOR_VARIANCE);
}

bool at_green_base(void)
{
    return (r < GREEN_R + HOME_BASE_COLOR_VARIANCE &&
    r > GREEN_R - HOME_BASE_COLOR_VARIANCE &&
    g < GREEN_G + HOME_BASE_COLOR_VARIANCE &&
    g > GREEN_G - HOME_BASE_COLOR_VARIANCE &&
    b < GREEN_B + HOME_BASE_COLOR_VARIANCE &&
    b > GREEN_B - HOME_BASE_COLOR_VARIANCE);
}

void update_color_sensor(void)
{

    tcs.setInterrupt(false); // turn on LED

    delay(10); // takes 50ms to read

    tcs.getRawData(&red, &green, &blue, &clear);

    tcs.setInterrupt(true); // turn off LED

    // Figure out some basic hex code for visualization
    uint32_t sum = clear;
    r = red;
    r /= sum;
    g = green;
    g /= sum;
    b = blue;
    b /= sum;
    r *= 256;
    g *= 256;
    b *= 256;
}

void update_weight_distances()
{
    // centre
    if (get_sensor_distance(front_bottom) < get_sensor_distance(front_top) - 250 && get_sensor_distance(front_bottom) < 1000)
    {
        centre_weight_detected = true;
        if (get_sensor_distance(front_bottom) < 200)
        {
            weight_in_range = true;
        }
    }
    else
    {
        centre_weight_detected = false;
        weight_in_range = false;
    }
    // left
    if (get_sensor_distance(front_left_bottom) < get_sensor_distance(front_left_top) - 250 && get_sensor_distance(front_left_bottom) < 1000)
    {
        left_weight_detected = true;
    }
    else
    {
        left_weight_detected = false;
    }
    // right
    if (get_sensor_distance(front_right_bottom) < get_sensor_distance(front_right_top) - 250 && get_sensor_distance(front_right_bottom) < 1000)
    {
        right_weight_detected = true;
    }
    else
    {
        right_weight_detected = false;
    }
}

void init_tof_sensors()
{
    io.begin(SX1509_ADDRESS);
    Wire.end();
    Wire.begin();
    Wire.setClock(400000); // use 400 kHz I2C

    // Disable/reset all sensors by driving their XSHUT pins low.
    for (uint8_t i = 0; i < L0_sensor_count; i++)
    {
        io.pinMode(xshutPinsL0[i], OUTPUT);
        io.digitalWrite(xshutPinsL0[i], HIGH);
    }

    for (uint8_t i = 0; i < L1_sensor_count; i++)
    {
        io.pinMode(xshutPinsL1[i], OUTPUT);
        io.digitalWrite(xshutPinsL1[i], HIGH);
    }

    for (uint8_t i = 0; i < L0_sensor_count; i++)
    {
        io.digitalWrite(xshutPinsL0[i], LOW);
        sensorsL0[i] = VL53L0X();
    }

    for (uint8_t i = 0; i < L1_sensor_count; i++)
    {
        io.digitalWrite(xshutPinsL1[i], LOW);
        sensorsL1[i] = VL53L1X();
    }

    // L0 Enable, initialize, and start each sensor, one by one.
    for (uint8_t i = 0; i < L0_sensor_count; i++)
    {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        io.digitalWrite(xshutPinsL0[i], HIGH);
        delay(10);

        sensorsL0[i].setTimeout(50);
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

        sensorsL0[i].startContinuous(20); // TODO potentially change this number
    }

    // L1 Enable, initialize, and start each sensor, one by one.
    for (uint8_t i = 0; i < L1_sensor_count; i++)
    {
        // Stop driving this sensor's XSHUT low. This should allow the carrier
        // board to pull it high. (We do NOT want to drive XSHUT high since it is
        // not level shifted.) Then wait a bit for the sensor to start up.
        // pinMode(xshutPins[i], INPUT);
        io.digitalWrite(xshutPinsL1[i], HIGH);
        delay(10);

        sensorsL1[i].setTimeout(50);
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

        sensorsL1[i].startContinuous(20); // TODO potentially change this number
    }
}

void update_tof_sensors()
{
    if(tof_critical_error)
    {
        init_tof_sensors();
        tof_critical_error = false;                 // Calling the init_tof_sensors() method should resolve the error timeout critical error.
    }

    for (uint8_t i = 0; i < L0_sensor_count; i++)
    {
        sensor_L0_values[i] = sensorsL0[i].readRangeContinuousMillimeters(); // TODO should this be continuous?
        if (sensorsL0[i].timeoutOccurred())
        {
            Serial.printf("TIMEOUT!! on L0[%d]\n", i);
            tof_critical_error = true;
        }
    }

    for (uint8_t i = 0; i < L1_sensor_count; i++)
    {
        sensor_L1_values[i] = sensorsL1[i].readRangeContinuousMillimeters();
        if (sensorsL1[i].timeoutOccurred())
        {
            Serial.printf("TIMEOUT!! on L1[%d]\n", i);
            tof_critical_error = true;
        }
    }

    sensor_values[(int)right]               = sensor_L0_values[0];
    sensor_values[(int)left]                = sensor_L0_values[1];

    sensor_values[(int)front_right_bottom]  = sensor_L1_values[0];
    sensor_values[(int)front_left_bottom]   = sensor_L1_values[1];
    sensor_values[(int)front_right_top]     = sensor_L1_values[2];
    sensor_values[(int)front_left_top]      = sensor_L1_values[3];
    sensor_values[(int)front_bottom]        = sensor_L1_values[4];
    sensor_values[(int)front_top]           = sensor_L1_values[5];
}

// ****************** Weight Detection Code ******************

/**
 * @brief Returns the number of times that weights have been consecutively
 * detected in a particular direction ==> (left, fowards, right).
 *
 * @param direction     => Defines which direction to test weight detection
 * @param distance_mm   => Returns how far away the weight is.
 * @return uint16_t     => Number of consectutive weight detections.
 */
uint32_t get_consecutive_weight_detections(weight_detect_direction_t direction, uint32_t *distance_mm)
{
    *distance_mm = weight_distances[(int)direction];
    return consecutive_weight_detections[(int)direction];
}

void update_consecutive_weight_detections()
{
    // static uint32_t consecutive_weight_detections[NUM_WEIGHT_DETECTION_DIRECTIONS];
    int32_t weight_detection_delta = 100; // [mm] ==> Threshold difference between top and bottom sensors to qualify as a weight detection
    uint32_t max_search_distance = 600;   // [mm]

    // consecutive_weight_detections[(int)left_foward]     = 0;
    // consecutive_weight_detections[(int)fowards]         = 0;
    // consecutive_weight_detections[(int)right_foward]    = 0;

    uint32_t distance_front_left_bottom = get_sensor_distance(front_left_bottom); // TODO this is not neccessary
    uint32_t distance_front_bottom = get_sensor_distance(front_bottom);
    uint32_t distance_front_right_bottom = get_sensor_distance(front_right_bottom);

    int32_t delta_left = get_sensor_distance(front_left_top) - distance_front_left_bottom - sensor_mounted_offsets[(int)left_foward];
    int32_t delta_foward = get_sensor_distance(front_top) - distance_front_bottom - sensor_mounted_offsets[(int)fowards];
    int32_t delta_right = get_sensor_distance(front_right_top) - distance_front_right_bottom - sensor_mounted_offsets[(int)right_foward];

    // ****** Front Left ******
    if (delta_left > weight_detection_delta && get_sensor_distance(front_left_bottom) < max_search_distance)
    {
        consecutive_weight_detections[(int)left_foward] += 1;
        weight_distances[(int)left_foward] = distance_front_left_bottom;
        // Serial.printf("Weight detected left\n");
    }
    else
    {
        consecutive_weight_detections[(int)left_foward] = 0;
        weight_distances[(int)left_foward] = 0;
    }

    // ****** Middle / Fowards ******
    if (delta_foward > weight_detection_delta && get_sensor_distance(front_bottom) < max_search_distance-150)
    {
        consecutive_weight_detections[(int)fowards] += 1;
        // Serial.printf("middle count: %u\n", consecutive_weight_detections[fowards]);
        weight_distances[(int)fowards] = distance_front_bottom;
        // Serial.printf("Weight detected middle\n");
    }
    else
    {
        consecutive_weight_detections[(int)fowards] = 0;
        weight_distances[(int)fowards] = 0;
    }

    // ****** Front Right ******
    if (delta_right > weight_detection_delta && get_sensor_distance(front_right_bottom) < max_search_distance)
    {
        consecutive_weight_detections[(int)right_foward] += 1;
        weight_distances[(int)right_foward] = distance_front_right_bottom;
        // Serial.printf("Weight detected right\n");
    }
    else
    {
        consecutive_weight_detections[(int)right_foward] = 0;
        weight_distances[(int)right_foward] = 0;
    }
}

uint16_t get_sensor_distance(sensor_t sensor)
{
    return sensor_values[(int)sensor];
}

bool is_right_weight_detected()
{
    return right_weight_detected;
}

bool is_left_weight_detected()
{
    return left_weight_detected;
}

bool is_centre_weight_detected()
{
    return centre_weight_detected;
}

bool is_weight_in_range()
{
    return weight_in_range;
}