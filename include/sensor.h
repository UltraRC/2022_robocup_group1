#ifndef SENSOR_H
#define SENSOR_H

void init_sensors(void);
void update_sensors(void);

void init_tof_sensors(void);
void update_tof_sensors(void);
bool weight_detected(void);

uint16_t get_sensor_right(void);
uint16_t get_sensor_front_right(void);

#endif // SENSOR_H