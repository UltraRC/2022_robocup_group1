#ifndef SENSOR_H
#define SENSOR_H

typedef enum {
    right = 0,
    front_right_top,
    front_right_bottom,
    left,
    front_left_top,
    front_left_bottom,
    front_top,
    front_bottom
} sensor_t;

void init_sensors(void);
void update_sensors(void);

void init_tof_sensors(void);
void update_tof_sensors(void);
bool weight_detected(void);

uint16_t get_sensor_distance(sensor_t sensor);

#endif // SENSOR_H