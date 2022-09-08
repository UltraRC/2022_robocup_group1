#ifndef SENSOR_H
#define SENSOR_H

void init_sensors(void);
void update_sensors(void);

void init_tof_sensors(void);
void update_tof_sensors(void);
bool weight_detected(void);

#endif // SENSOR_H