#ifndef ACTUATOR_H
#define ACTUATOR_H

    #define MAIN_SERVO_PIN      32
    #define P_SERVO_LEFT_PIN    30
    #define P_SERVO_RIGHT_PIN   31
    #define REAR_SERVO_PIN      33

    #define ELECTROMAG_PIN      14
    #define WEIGHT_DROP_PIN     24

    #define MIN_ANGLE_PINCER    +80
    #define MAX_ANGLE_PINCER    -5
    #define MIN_ANGLE_MAIN      +50
    #define MAX_ANGLE_MAIN      -65

    void init_actuators(void);
    
    void init_main_servo(void);
    void init_pincer_servos(void);
    void init_rear_servo(void);
    void init_electromag(void);
    void init_weight_drop(void);
    void set_actuators_default_values(void);

    void set_main_servo_angle(int32_t angle);
    void set_electromag(boolean input);
    void set_pincer_servos_angle(int32_t angle);
    void set_rear_servo_angle(int32_t angle);
    void set_weight_drop(boolean set);
#endif // ACTUATOR_H