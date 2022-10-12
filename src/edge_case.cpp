#include <Arduino.h>
#include "sensor.h"

bool ramp_dect(bool weight_left, bool weight_right, bool weight_front)
{
    return (weight_left && weight_front) || (weight_right && weight_front); 
}


bool repeted_pickup (bool atempted_pickup)
{
    static uint32_t prev_time = millis();
    static uint16_t count = 0;

    uint16_t max_count = 3;
    uint32_t delta_time = millis() - prev_time;

    if(delta_time < 1000)
    {
        count++;
    } else {
        count = 0;
    } 

    prev_time = millis();
    return (count >= max_count);
}


bool not_moving (void)
{
    static uint32_t left_old = get_sensor_distance(front_left_top);
    static uint32_t right_old = get_sensor_distance(front_right_top);
    static uint32_t front_old = get_sensor_distance(front_top);
    static uint8_t count = 0;

    uint32_t left_new = get_sensor_distance(front_left_top);
    uint32_t right_new = get_sensor_distance(front_right_top);
    uint32_t front_new = get_sensor_distance(front_top);

    uint32_t delta = 500;
    uint8_t max_count = 3;

    bool is_left_no_move = abs(left_old - left_new) < delta;
    bool is_right_no_move = abs(right_old - right_new) < delta;
    bool is_front_no_move = abs(front_old - front_new) < delta;

    if (is_left_no_move && is_right_no_move && is_front_no_move)
    {
        count++;
    } else {
        count = 0;
    }
    return count >= max_count;
}


