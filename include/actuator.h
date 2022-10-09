#ifndef ACTUATOR_H
#define ACTUATOR_H

    #define MIN_ANGLE_PINCER    +77
    #define MAX_ANGLE_PINCER    -5
    #define MIN_ANGLE_MAIN      +50
    #define MAX_ANGLE_MAIN      -60
    #define ANGLE_OFFSET -5
    #define LEFT_OFFSET -1

    void init_actuators(void);
    
    void init_servos(void);
    void init_electromag(void);
    void init_weight_drop(void);
    void set_actuators_default_values(void);

    void set_main_servo_angle(int32_t angle);
    void set_electromag(boolean input);
    void set_front_electromag(boolean set);
    void set_pincer_servos_angle(int32_t angle);
    void set_weight_drop(boolean set);
#endif // ACTUATOR_H