#ifndef _MAPPER_SOURCE_
#define _MAPPER_SOURCE_

#include "mapper.h"


Mapper::Mapper():
_wheel_l(MOTOR_PWM_LEFT, MOTOR_FWD_LEFT, MOTOR_REV_LEFT),
_wheel_r(MOTOR_PWM_RIGHT, MOTOR_FWD_RIGHT, MOTOR_REV_RIGHT),
_encoder_left(ENCODER_LEFT_PIN),
_encoder_right(ENCODER_RIGHT_PIN),
_i2c(I2C_SDA_PIN, I2C_SCL_PIN),
_lidar_shdn_center(LIDAR_SHDN_CENTER),
_lidar_shdn_left(LIDAR_SHDN_LEFT),
_lidar_shdn_right(LIDAR_SHDN_RIGHT)
{
    _encoder_left.reset();
    _encoder_right.reset();
    _init_lidar();
}

Mapper::~Mapper() {
    delete _lidars.center;
    delete _lidars.left;
    delete _lidars.right;
    if (_pid != NULL) {
        delete _pid;
    }
}

// Returns 0 on map of object
// Returns -1 on fail to map an object
int Mapper::plot_object(LIDAR_DIRECTION dir, Point &p) {
    uint32_t dist;
    int status = read_dist(dir, dist);
    int soff_x = 0;  // Offsets of sensors from center of robot
    int soff_y = 0;
    if (status == VL53L0X_ERROR_NONE && dist <= _map_thresh_mm) {
        float s_theta = 0;  // Sensor theta
        switch (dir) {
            case CENTER:
                s_theta = state.theta;
                soff_x = _soff_x_c;
                soff_y = _soff_y_c;
                break;
            case LEFT:
                s_theta = state.theta + M_PI / 2;
                soff_x = _soff_x_l;
                soff_y = _soff_y_l;
                break;
            case RIGHT:
                s_theta = state.theta - M_PI / 2;
                soff_x = _soff_x_r;
                soff_y = _soff_y_r;
                break;
            default:
                error("INVALID LIDAR DIRECTION\r\n");
        };
        p.x = state.x + soff_y * cos(state.theta) + soff_x * cos(state.theta - M_PI / 2) + dist * cos(s_theta);
        p.y = state.y + soff_y * sin(state.theta) + soff_x * sin(state.theta - M_PI / 2) + dist * sin(s_theta);
        // p.x = state.x + dist * cos(s_theta);
        // p.y = state.y + dist * sin(s_theta);
        return 0;
    }
    return -1;
}

int Mapper::read_dist(LIDAR_DIRECTION dir, uint32_t &d) {
    uint32_t distance;
    int status = -1;
    switch (dir) {
        case CENTER:
            status = _lidars.center->get_distance(&distance);
            break;
        case LEFT:
            status = _lidars.left->get_distance(&distance);
            break;
        case RIGHT:
            status = _lidars.right->get_distance(&distance);
            break;
        default:
            error("INVALID LIDAR DIRECTION\r\n");
    };
    if (status == VL53L0X_ERROR_NONE) {
        d = distance;
    } else if (status != VL53L0X_ERROR_RANGE_ERROR) {
        error("LIDAR DIRECTION %d: ERROR %d", dir, status);
    }
    return status;
}

void Mapper::print_cal(Serial &out) {
    std::map<float, int32_t>::iterator itr;
    out.printf("Left samples:\r\n");
    for (itr = _pwm_speed_map_l.begin(); itr != _pwm_speed_map_l.end(); ++itr) {
        out.printf("_pwm_speed_map_l.insert(std::pair<float, int32_t>(%.1f, %d));\r\n", itr->first, itr->second);
    }
    out.printf("Right samples:\r\n");
    for (itr = _pwm_speed_map_r.begin(); itr != _pwm_speed_map_r.end(); ++itr) {
        out.printf("_pwm_speed_map_r.insert(std::pair<float, int32_t>(%.1f, %d));\r\n", itr->first, itr->second);
    }
    out.printf("\r\nLEFT:\r\n");
    out.printf("\t%f (mm/s) / V\r\n", _pwm_speed_m_l);
    out.printf("\t%f mm/s at 0 V\r\n", _pwm_speed_b_l);
    out.printf("RIGHT:\r\n");
    out.printf("\t%f (mm/s) / v\r\n", _pwm_speed_m_r);
    out.printf("\t%f mm/s at 0 V\r\n\r\n", _pwm_speed_b_r);
}

void Mapper::default_cal() {
    // Left samples:
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.1, 17));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.2, 58));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.3, 40));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.4, 78));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.5, 145));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.6, 188));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.7, 290));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.8, 337));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.9, 388));
    _pwm_speed_map_l.insert(std::pair<float, int32_t>(1.0, 403));
    // Right samples:
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.3, 40));
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.4, 183));
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.5, 68));
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.6, 211));
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.7, 280));
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.8, 295));
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.9, 298));
    _pwm_speed_map_r.insert(std::pair<float, int32_t>(1.0, 308));

    // This is the dictionary-like data structure "map", not plotted points
    linearize_map(_pwm_speed_map_l, &_pwm_speed_m_l, &_pwm_speed_b_l);
    linearize_map(_pwm_speed_map_r, &_pwm_speed_m_r, &_pwm_speed_b_r);
}

// Spin robot around, mapping PWM values to
// wheel velocities on the current surface,
// use linear regression to find line of best
// fit for (pwm, velocity) graph
void Mapper::calibrate_left_wheel() {
    float pwm_val;
    int32_t av_vel;
    int samples = 5;
    int32_t prev_speed;

    // First get robot up to full speed
    for (int i = 0; i <= 10; i++) {
        _wheel_l.speed(i * 0.1);
        wait(0.01);
    }
    // Slow down by 0.1 pwm increments, waiting
    // for speed to be constant, sample speed
    // and get average speed to insert into the
    // pwm to speed mappings.
    for (int i = 10; i >= 0; i--) {
        pwm_val = i * 0.1;
        _wheel_l.speed(pwm_val);
        wait(0.1);
        do {
            prev_speed = state.lv;
            wait(0.1);
        } while (abs(state.lv - prev_speed) > 3);
        av_vel = 0;
        for (int i = 0; i < samples; i++) {
            av_vel += state.lv;
            wait(0.1);
        }
        av_vel = av_vel / samples;
        // Only add the value if it maintains movement
        if (av_vel > 5) {
            _pwm_speed_map_l.insert(std::pair<float, int32_t>(pwm_val, av_vel));
        }
    }
    // This is the dictionary-like data structure "map", not plotted points
    linearize_map(_pwm_speed_map_l, &_pwm_speed_m_l, &_pwm_speed_b_l);
}

void Mapper::calibrate_right_wheel() {
    // See left wheel cal
    float pwm_val;
    int32_t av_vel;
    int samples = 5;
    int32_t prev_speed;

    for (int i = 0; i <= 10; i++) {
        _wheel_r.speed(i * 0.1);
        wait(0.01);
    }
    for (int i = 10; i >= 0; i--) {
        pwm_val = i * 0.1;
        _wheel_r.speed(pwm_val);
        wait(0.1);
        do {
            prev_speed = state.rv;
            wait(0.1);
        } while (abs(state.rv - prev_speed) > 3);
        av_vel = 0;
        for (int i = 0; i < samples; i++) {
            av_vel += state.rv;
            wait(0.1);
        }
        av_vel = av_vel / samples;
        if (av_vel > 5) {
            _pwm_speed_map_r.insert(std::pair<float, int32_t>(pwm_val, av_vel));
        }
    }
    linearize_map(_pwm_speed_map_r, &_pwm_speed_m_r, &_pwm_speed_b_r);
}

void Mapper::init_state() {
    target_speed = 0;
    target_theta = M_PI / 2;
    state.x = 0;
    state.y = 0;
    state.lv = target_speed;
    state.rv = target_speed;
    state.theta = target_theta;
    _init_pid(target_speed);
}

void Mapper::start_state_update(float dt) {
    _dt = dt;
    init_state();
    _update_poll.attach<Mapper, void(Mapper::*)()>(this, &Mapper::_update_state, _dt);
}

// Update state state of the robot.
// If control is on, also update
// wheel speeds to match target
// speed and theta.
void Mapper::_update_state() {
    // State will be inacurate if any negative speeds are used!
    Measurement m = _get_measurements();
    State nx;
    nx.theta = bound_theta(state.theta + (_dt * m.rv - _dt * m.lv) / _wheel_sep);
    nx.x = state.x + (0.5 * _dt * m.lv + 0.5 * _dt * m.rv) * cos(nx.theta);
    nx.y = state.y + (0.5 * _dt * m.lv + 0.5 * _dt * m.rv) * sin(nx.theta);
    nx.lv = m.lv;
    nx.rv = m.rv;

    // Control variables, amount that we are changing
    // the speed on the left and right wheels
    int32_t lv_diff = 0;
    int32_t rv_diff = 0;
    if (control) {
        _update_control(&lv_diff, &rv_diff);
    }

    state = nx;
}

void Mapper::_init_pid(int32_t speed) {
    if (_pid != NULL) {
        delete _pid;
    }
    /*               s,  mm/s,  mm/s,  (mm/s)/rad  , (mm/s)/(rad/s),  (mm/s)/(rad*s)   */
    _pid = new PID(_dt,   800,  -800,    800 / M_PI,    20 / M_PI  ,    400 /  M_PI);
}

void Mapper::_update_control(int32_t *_lv_diff, int32_t *_rv_diff) {
    static int32_t last_speed = 0;  // keep track of a target speed change
    *_lv_diff = 0;  // Initialize change in velocities to 0
    *_rv_diff = 0;
    // If the target speed was changed, add the change from current
    // state to the diffs, update our base pwm values, and store in
    // static last speed.
    if (target_speed != last_speed) {
        *_lv_diff = target_speed - last_speed;
        *_rv_diff = target_speed - last_speed;
        _pwm_l = (target_speed != 0) ? (abs(target_speed) - _pwm_speed_b_l) / _pwm_speed_m_l : 0;
        _pwm_r = (target_speed != 0) ? (abs(target_speed) - _pwm_speed_b_r) / _pwm_speed_m_r : 0;
        if (target_speed < 0) {
            _pwm_l = -_pwm_l;
            _pwm_r = -_pwm_r;
        }
        _init_pid(target_speed);  // Maybe don't need to reset PID anymore
                                  // Initially because of different PIDs for
                                  // different speeds.  Just doing it now to
                                  // clear the integral.
        last_speed = target_speed;
    }

    // These are temporary thetas to make the difference in bounds [-PI, PI]
    float state_theta = state.theta;
    float targ_theta = target_theta;
    if ((targ_theta - state_theta) > M_PI) {
        targ_theta -= 2 * M_PI;
    } else if ((targ_theta - state_theta) < -M_PI) {
        state_theta -= 2 * M_PI;
    }
    _v_off = _pid->calculate(targ_theta, state_theta);  // Offset in velocities between wheel

    // Not sure if this works for negative target speeds
    if (abs(target_speed) > 0) {  // If moving forward only add pwm to base pwm
        // Left wheel needs to goes faster
        if (_v_off < 0 ) {
            // Take all extra speed off right wheel
            *_rv_diff -= _pwm_add_r * _pwm_speed_m_r;
            _pwm_add_r = 0;
            // Add extra pwm to left
            *_lv_diff += abs(_v_off) - (_pwm_add_l * _pwm_speed_m_l);  // This could be negative
            _pwm_add_l = (1 / _pwm_speed_m_l) * abs(_v_off);
        } else {
            *_lv_diff -= _pwm_add_l * _pwm_speed_m_l;
            _pwm_add_l = 0;
            *_rv_diff += abs(_v_off) - (_pwm_add_r * _pwm_speed_m_r);
            _pwm_add_r = (1 / _pwm_speed_m_r) * abs(_v_off);
        }
    } else {  // If stationary, one wheel goes back and the other forward
        *_rv_diff += (_v_off / 2) - (_pwm_add_r * _pwm_speed_m_r);
        _pwm_add_r = (1 / _pwm_speed_m_r) * (_v_off / 2);
        *_lv_diff += (- _v_off / 2) - (_pwm_add_l * _pwm_speed_m_l);  // This could be negative
        _pwm_add_l = (1 / _pwm_speed_m_l) * (- _v_off / 2);
    }

    // Finally set our pwm
    float pwm_l = _pwm_l + _pwm_add_l;
    float pwm_r = _pwm_r + _pwm_add_r;
    if (pwm_l < 0.0) {
        _wheel_dir_l = -1;
    } else {
        _wheel_dir_l = 1;
    }
    if (pwm_r < 0.0) {
        _wheel_dir_r = -1;
    } else {
        _wheel_dir_r = 1;
    }
    _wheel_l.speed(pwm_l <= 1.0 ? pwm_l : 1.0);
    _wheel_r.speed(pwm_r <= 1.0 ? pwm_r : 1.0);
}

Measurement Mapper::_get_measurements() {
    static float dpc = _wheel_circ / _encoder_cpr;  // distance per encoder count in mm
    // static float dpc = 0.5847;
    Measurement z;
    int l_ct = _encoder_left.read();
    int r_ct = _encoder_right.read();
    float ld = dpc * l_ct; //unit in mm
    float rd = dpc * r_ct; //unit in mm
    z.lv = ld / _dt * _wheel_dir_l;
    z.rv = rd / _dt * _wheel_dir_r;
    _encoder_left.reset();
    _encoder_right.reset();
    return z;
}

// Create the lidar objects, initialize boards, assign the boards unique I2C addresses
void Mapper::_init_lidar() {
    _lidars.center = new VL53L0X(_i2c, _lidar_shdn_center, p15);
    _lidars.left = new VL53L0X(_i2c, _lidar_shdn_left, p16);
    _lidars.right  = new VL53L0X(_i2c, _lidar_shdn_right, p17);
    int status;
    status = _lidars.center->init_sensor(0x01);
    if (status != 0) {
        error("FAILED TO INITIALIZE CENTER LIDAR\r\n");
    }
    status = _lidars.left->init_sensor(0x03);
    if (status != 0) {
        error("FAILED TO INITIALIZE LEFT LIDAR\r\n");
    }
    status = _lidars.right->init_sensor(0x05);
    if (status != 0) {
        error("FAILED TO INITIALIZE RIGHT LIDAR\r\n");
    }
}

#endif
