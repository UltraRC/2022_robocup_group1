#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

typedef enum {
    right_wall = 1,
    left_wall = -1
} side_t;

void init_wall_follow(void);
void update_wall_follow(side_t side, bool use_bottom_sensors);
void set_wall_follow_velocity(uint32_t velocity);
void set_wall_follow_distance(uint32_t distance);
#endif // WALL_FOLLOW_H