#include "mapper.h"
BusOut leds(LED1, LED2, LED3, LED4);


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
}

int Mapper::drive(float speed) {
    _speed = speed;
    _wheel_l.speed(speed);
    _wheel_r.speed(speed);
    return 0;
}

bool Mapper::check_moved_distance(uint32_t dist) {
    int lr = _encoder_left.read();
    int rr = _encoder_right.read();
    float ld = 0.5672320068 * (lr); //unit in mm
    float rd = 0.5672320068 * (rr);
    float total_d = (ld + rd) / 2;
    x = 0;
    y = total_d;
    return y >= dist - 45;
}

void Mapper::move_straight() {
    static uint16_t last_enc_count_l = 0;
    static uint16_t last_enc_count_r = 0;
    static uint16_t turn_l_n = 0;  // Amount of times in a row needs to turn left
    static uint16_t turn_r_n = 0;  // Amount of times in a row needs to turn right
    uint16_t count_l = _encoder_left.read();
    uint16_t count_r = _encoder_right.read();
    float dist_l = 0.5672320068 * (count_l - last_enc_count_l);  // unit in mm
    float dist_r = 0.5672320068 * (count_r - last_enc_count_r);
    last_enc_count_l = count_l;
    last_enc_count_r = count_r;
    if (dist_l < 0 || dist_r < 0) return;  // If encoder was reset
    if (dist_l > dist_r) {
        // need turn left
        turn_r_n = 0;
        _wheel_r.speed(_speed + 0.05 + 0.01 * ++turn_l_n);
        _wheel_l.speed(_speed);
    }
    else if (dist_l < dist_r) {
        // need to turn right
        turn_l_n = 0;
        _wheel_r.speed(_speed);
        _wheel_l.speed(_speed + 0.05 + 0.01 * ++turn_r_n);
    }
}

void Mapper::wheel_speed() {
    leds = ~leds;
    // Has to be slower or same rate as state update
    static uint16_t last_lv = 0;
    static uint16_t last_rv = 0;
    float pwm_inc = 0.02;
    // Check if current speed is less than set speed
    // and check if speed moved in right direction
    int tolerance = 10;
    if ((state.lv < _speed_mm_l - tolerance) && (state.lv <= last_lv)) {
        _pwm_l += pwm_inc;
        _pwm_l = (_pwm_l > 1.0) ? 1.0 : _pwm_l;
    }
    if ((state.lv > _speed_mm_l + tolerance) && (state.lv >= last_lv)) {
        _pwm_l -= pwm_inc;
        _pwm_l = (_pwm_l < 0) ? 0 : _pwm_l;
    }
    if ((state.rv < _speed_mm_r - tolerance) && (state.rv <= last_rv)) {
        _pwm_r += pwm_inc;
        _pwm_r = (_pwm_r > 1.0) ? 1.0 : _pwm_r;
    }
    if ((state.rv > _speed_mm_r + tolerance) && (state.rv >= last_rv)) {
        _pwm_r -= pwm_inc;
        _pwm_r = (_pwm_r < 0) ? 0 : _pwm_r;
    }
    _wheel_l.speed(_pwm_l);
    _wheel_r.speed(_pwm_r);
    last_lv = state.lv;
    last_rv = state.rv;
}



void Mapper::orientation() {
    float diff = abs(state.theta - target_theta);
    if (diff > 5 * M_PI / 180) {
        if (state.theta > target_theta) {
            // float new_speed = _speed + (_speed == 0) * 0.35 + 0.2 * (diff / (2*M_PI));
            // _wheel_l.speed(new_speed);
            // _wheel_r.speed(_speed + (_speed == 0) * 0.2);
            _speed_mm_l = _speed_mm + 20;
            _speed_mm_r = _speed_mm;
        } else {
            _speed_mm_l = _speed_mm;
            _speed_mm_r = _speed_mm + 20;
        }
    } else {
        _speed_mm_l = _speed_mm;
        _speed_mm_r = _speed_mm;
    }
}


int Mapper::move_forward(uint32_t dist) {
    //pc.printf("total_d: %f\n\r",total_d);
    _encoder_left.reset();
    _encoder_right.reset();
    _wheel_l.speed(0.3);
    _wheel_r.speed(0.3);
    while (!check_moved_distance(dist));
    _wheel_l.speed(0);
    _wheel_r.speed(0);
    return 0;
}

void Mapper::update_position() {
    // State will be inacurate if any negative speeds are used!
    Measurement m = get_measurements();
    State nx;
    nx.theta = state.theta + (_dt * m.rv - _dt * m.lv) / _wheel_sep;
    nx.x = state.x + (0.5 * _dt * m.lv + 0.5 * _dt * m.rv) * cos(nx.theta);
    nx.y = state.y + (0.5 * _dt * m.lv + 0.5 * _dt * m.rv) * sin(nx.theta);
    nx.lv = m.lv;
    nx.rv = m.rv;
    state = nx;
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
}

Measurement Mapper::get_measurements() {
    Measurement z;
    int lr = _encoder_left.read();
    int rr = _encoder_right.read();
    float ld = 0.5672320068 * (lr); //unit in mm
    float rd = 0.5672320068 * (rr);
    z.lv = ld / _dt;
    z.rv = rd / _dt;
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
    if (status == VL53L0X_ERROR_NONE && dist <= _map_thresh_mm) {
        float l_theta = 0;
        switch (dir) {
            case CENTER:
                l_theta = theta;
                break;
            case LEFT:
                l_theta = theta + M_PI / 2;
                break;
            case RIGHT:
                l_theta = theta - M_PI / 2;
                break;
            default:
                error("INVALID LIDAR DIRECTION\r\n");
        };
        p.x = x + cos(l_theta) * dist;
        p.y = y + sin(l_theta) * dist;
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
