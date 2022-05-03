#ifndef _MAPPER_H_
#define _MAPPER_H_

#include <map>
#include "mbed.h"
#include "Motor.h"
#include "VL53L0X.h"
#include "HALLFX_ENCODER.h"
#include "PID.h"

#define MOTOR_PWM_LEFT p22
#define MOTOR_PWM_RIGHT p21
#define MOTOR_FWD_LEFT p6
#define MOTOR_FWD_RIGHT p7
#define MOTOR_REV_LEFT p5
#define MOTOR_REV_RIGHT p8
#define ENCODER_LEFT_PIN p11
#define ENCODER_RIGHT_PIN p12
#define I2C_SDA_PIN p28
#define I2C_SCL_PIN p27
#define LIDAR_SHDN_CENTER p26
#define LIDAR_SHDN_LEFT p25
#define LIDAR_SHDN_RIGHT p24


typedef enum {
    CENTER = 0,
    LEFT = 1,
    RIGHT = 2
} LIDAR_DIRECTION;

typedef struct lidars {
    VL53L0X *center;
    VL53L0X *left;
    VL53L0X *right;
} lidars;

typedef struct point {
        int32_t x;
        int32_t y;
} Point;

typedef struct state {
    int32_t x = 0;
    int32_t y = 0;
    int32_t lv = 0;
    int32_t rv = 0;
    float theta = M_PI / 2;
} State;

typedef struct measurement {
    int32_t lv;
    int32_t rv;
    float theta;
} Measurement;


inline void linearize_map(std::map<float, int32_t> &_map, float *m, float *b) {
    int n = _map.size();
    float xi;
    float yi;
    float sum_x = 0;
    float sum_y = 0;
    float sum_xy = 0;
    float sum_x_square = 0;
    float sum_y_square = 0;
    std::map<float, int32_t>::iterator itr;
    for (itr = _map.begin(); itr != _map.end(); itr++) {
        xi = (float)(itr->first);
        yi = (float)(itr->second);
        sum_x += xi;
        sum_y += yi;
        sum_xy += xi * yi;
        sum_x_square += xi * xi;
        sum_y_square += yi * yi;
    }
    float numerator = n * sum_xy - sum_x * sum_y;
    float denominator = n * sum_x_square - sum_x * sum_x;
    *m = numerator / denominator;
    numerator = sum_y * sum_x_square - sum_x * sum_xy;
    denominator = n * sum_x_square - sum_x * sum_x;
    *b = numerator / denominator;
}

// Bounds theta between 0 and 2*PI
inline float bound_theta(float theta) {
    if (theta > 2 * M_PI) theta -= (2 * M_PI) * floor(theta / (2 * M_PI));
    if (theta < 0) theta += (2 * M_PI) * (floor(abs(theta) / (2 * M_PI)) + 1);
    return theta;
}



class Mapper {
public:
    Mapper();
    ~Mapper();
    void init_state();
    void start_state_update(float dt);
    void print_cal(Serial &out);
    void default_cal();
    void calibrate_left_wheel();
    void calibrate_right_wheel();
    int plot_object(LIDAR_DIRECTION dir, Point &p);
    int read_dist(LIDAR_DIRECTION dir, uint32_t &dist);
    // Use these variables to control the robot
    int32_t target_speed = 0;  // Speed in mm/s, this is a target speed that is not corrected
    float target_theta = M_PI / 2;
    State state;
    bool control = true;  // Whether the robot tries to correct theta and speed
private:
    float _dt = 0;  // Time change between updates in seconds
    uint32_t _wheel_sep = 163;  // Separation between center of wheels in mm
    float _wheel_circ = 230.0;  // Wheel circumference in mm
    float _encoder_cpr = 360.0;  // Encoder count per rotation
    int32_t _soff_x_l = -65;  // Offset of sensors from center of robot in mm
    int32_t _soff_y_l = 40;
    int32_t _soff_x_c = 0;
    int32_t _soff_y_c = 80;
    int32_t _soff_x_r = 65;
    int32_t _soff_y_r = 40;
    Motor _wheel_l;
    Motor _wheel_r;
    float _pwm_l = 0.0;
    float _pwm_r = 0.0;
    int _wheel_dir_l = 1;  // 1 for forward, -1 for reverse
    int _wheel_dir_r = 1;
    HALLFX_ENCODER _encoder_left;
    HALLFX_ENCODER _encoder_right;
    DevI2C _i2c;
    DigitalOut _lidar_shdn_center;
    DigitalOut _lidar_shdn_left;
    DigitalOut _lidar_shdn_right;
    lidars _lidars;
    uint32_t _map_thresh_mm = 500;  // Objects must be this close to be mapped
    Ticker _update_poll;
    PID *_pid = NULL;
    // PWM speed map variables
    std::map<float, int32_t> _pwm_speed_map_l;
    std::map<float, int32_t> _pwm_speed_map_r;
    float _pwm_speed_m_l;
    float _pwm_speed_b_l;
    float _pwm_speed_m_r;
    float _pwm_speed_b_r;
    int32_t _v_off = 0;
    float _pwm_add_l = 0;
    float _pwm_add_r = 0;

    void _update_state();
    void _update_control(int32_t *_lv_diff, int32_t *_rv_diff);
    void _init_lidar();
    void _init_pid(int32_t speed);
    Measurement _get_measurements();
};

#endif
