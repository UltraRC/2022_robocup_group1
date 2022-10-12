#ifndef SENSOR_H
#define SENSOR_H

#define NUM_WEIGHT_DETECTION_DIRECTIONS 3

typedef enum
{
    right = 0,
    front_right_top,
    front_right_bottom,
    left,
    front_left_top,
    front_left_bottom,
    front_top,
    front_bottom
} sensor_t;

// An enumeration to which describes which direction
// weights are being detected.
// It refers to the three pairs of stacked weight detection sensors at the front of the robot.
typedef enum
{
    left_foward = 0,
    fowards,
    right_foward
} weight_detect_direction_t;

typedef struct
{
    uint32_t consecutive_weight_detections;
    uint32_t weight_distance;
} weight_detect_t;

void init_sensors(void);
void update_sensors(void);

void init_tof_sensors(void);
void update_tof_sensors(void);
uint16_t get_sensor_distance(sensor_t sensor);

void update_limit_switch(void);
bool limit_switch(void);

void update_color_sensor(void);
void update_weight_distances(void);
bool is_right_weight_detected(void);
bool is_left_weight_detected(void);
bool is_centre_weight_detected(void);
bool is_weight_in_range(void);
bool at_green_base(void);
bool at_blue_base(void);

// uint32_t get_consecutive_weight_detections(weight_detect_direction_t direction, uint32_t *distance_mm);
weight_detect_t get_consecutive_weight_detections(weight_detect_direction_t direction);

void update_consecutive_weight_detections(void);
bool weight_detected(weight_detect_direction_t direction);
uint32_t get_weight_distance(weight_detect_direction_t direction);
bool ramp_detected(void);
bool ramp_detected_timed(uint32_t time_threshold);

bool plastic_detected_timed(uint32_t time_threshold);
void plastic_weight_dected (void);

#endif // SENSOR_H