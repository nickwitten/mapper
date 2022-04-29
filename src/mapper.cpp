#ifndef _MAPPER_SOURCE_
#define _MAPPER_SOURCE_

#include "mapper.h"

Serial pc(USBTX, USBRX);

// Serial pc(USBTX, USBRX);

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

// Spin robot around, mapping PWM values to
// wheel velocities on the current surface,
// use linear regression to find line of best
// fit for (pwm, velocity) graph
void Mapper::calibrate_wheel_speed() {
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

    for (int i = 0; i <= 10; i++) {
        _wheel_r.speed(i * 0.1);
        wait(0.01);
    }
    // Slow down by 0.1 pwm increments, waiting
    // for speed to be constant, sample speed
    // and get average speed to insert into the
    // pwm to speed mappings.
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
        // Only add the value if it maintains movement
        if (av_vel > 5) {
            _pwm_speed_map_r.insert(std::pair<float, int32_t>(pwm_val, av_vel));
        }
    }

// // Bedroom floor
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.4, 150));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.5, 257));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.6, 337));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.7, 372));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.8, 495));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.9, 561));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(1.0, 605));
//
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.5, 129));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.6, 275));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.7, 349));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.8, 380));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.9, 498));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(1.0, 424));
//
//
// // Study table
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.3, 40));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.4, 132));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.5, 196));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.6, 308));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.7, 359));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.8, 439));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.9, 515));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(1.0, 508));
// 
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.5, 173));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.6, 222));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.7, 270));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.8, 354));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.9, 431));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(1.0, 464));
//
// // Study hallway
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.4, 48 ));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.5, 117));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.6, 168));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.7, 242));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.8, 319));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(0.9, 357));
//     _pwm_speed_map_l.insert(std::pair<float, int32_t>(1.0, 347));
// 
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.4, 22 ));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.5, 86 ));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.6, 135));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.7, 178));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.8, 242));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(0.9, 265));
//     _pwm_speed_map_r.insert(std::pair<float, int32_t>(1.0, 349));


    linearize_map(_pwm_speed_map_l, &_pwm_speed_m_l, &_pwm_speed_b_l);
    linearize_map(_pwm_speed_map_r, &_pwm_speed_m_r, &_pwm_speed_b_r);
}

void Mapper::start_state_update(float dt) {
    _dt = dt;
    _update_poll.attach<Mapper, void(Mapper::*)()>(this, &Mapper::update_state, _dt);
    _init_pid(target_speed);
}

void Mapper::update_state() {
    // State will be inacurate if any negative speeds are used!
    Measurement m = get_measurements();
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
        update_control(&lv_diff, &rv_diff);
    }

    // // x' = f(x)
    // State x_pred = fx(state, _dt);
    // // z' = z - h(x)
    // Measurement z_pred = hx(x_pred);
    // Measurement z = get_measurements();
    // Measurement residual;
    // residual.lv = z.lv - z_pred.lv;
    // residual.rv = z.rv - z_pred.rv;
    // residual.theta = z.theta - z_pred.theta;
    // // 
    // state = 
    prev_state = state;
    state = nx;
}

void Mapper::_init_pid(int32_t speed) {
    if (_pid != NULL) {
        delete _pid;
    }
    if (speed == 0) {
        /*              s, mm/s, mm/s, (mm/s)/rad  , (mm/s)/(rad/s),  (mm/s)/(rad*s)   */
        // _pid = new PID(_dt,  800, -800,   800 / M_PI,    20 / M_PI  ,    400 /  M_PI);  // Works well at 0 mm/s
        _pid = new PID(_dt,  1500, -1500,   1750 / M_PI,    75 / M_PI  ,    50 /  M_PI);  // Works well at 0 mm/s with differential offsets
    } else if (speed < 100) {
        _pid = new PID(_dt,  1000, -1000, 1000 / M_PI,      100 / M_PI,      50 / M_PI);  // Works well at 50 mm/s (works as if at 0 mm/s)
    } else {
        _pid = new PID(_dt,  1000, -1000, 800 / M_PI,      20 / M_PI,      50 / M_PI);  // Works well at 200 mm/s
    }
}

void Mapper::update_control(int32_t *_lv_diff, int32_t *_rv_diff) {
    static int32_t last_speed = 0;  // keep track of a target speed change
    *_lv_diff = 0;  // Initialize change in velocities to 0
    *_rv_diff = 0;
    // If the target speed was changed, add the change from current
    // state to the diffs, update our base pwm values, and store in
    // static last speed.
    if (target_speed != last_speed) {
        *_lv_diff = target_speed - last_speed;
        *_rv_diff = target_speed - last_speed;
        _pwm_l = (target_speed != 0) ? (target_speed - _pwm_speed_b_l) / _pwm_speed_m_l : 0;
        _pwm_r = (target_speed != 0) ? (target_speed - _pwm_speed_b_r) / _pwm_speed_m_r : 0;
        _init_pid(target_speed);
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

    if (target_speed > 0) {
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
    } else {
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

Measurement Mapper::get_measurements() {
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

State Mapper::fx(State _x) {
    State nx;
    nx.x += (0.5 * _dt * state.lv + 0.5 * _dt * state.rv) * cos(state.theta);
    nx.y += (0.5 * _dt * state.lv + 0.5 * _dt * state.rv) * sin(state.theta);
    nx.lv = state.lv;
    nx.rv = state.rv;
    // TODO: if used theta needs to be kept in [0, 2*PI]
    nx.theta += (_dt * state.rv - _dt * state.lv) / _wheel_sep;
    return nx;
}

Measurement Mapper::hx(State _x) {
    Measurement m;
    m.lv = _x.lv;
    m.rv = _x.rv;
    m.theta = _x.theta;
    return m;
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

// Create the lidar objects, initialize boards, assign the boards unique I2C addresses
void Mapper::_init_lidar() {
    _lidars.center = new VL53L0X(_i2c, _lidar_shdn_center, p15);
    _lidars.left = new VL53L0X(_i2c, _lidar_shdn_left, p16);
    _lidars.right  = new VL53L0X(_i2c, _lidar_shdn_right, p17);
    int status;
    status = _lidars.center->init_sensor(0x01);
    if (status != 0) {
        // error("FAILED TO INITIALIZE CENTER LIDAR\r\n");
    }
    status = _lidars.left->init_sensor(0x03);
    if (status != 0) {
        // error("FAILED TO INITIALIZE LEFT LIDAR\r\n");
    }
    status = _lidars.right->init_sensor(0x05);
    if (status != 0) {
        // error("FAILED TO INITIALIZE RIGHT LIDAR\r\n");
    }
}

#endif
