#include "mapper.h"
#include "VL53L0X.h"
#include "mbed.h"


Mapper::Mapper():
_i2c(I2C_SDA_PIN, I2C_SCL_PIN),
_lidar_shdn_center(LIDAR_SHDN_CENTER),
_lidar_shdn_left(LIDAR_SHDN_LEFT),
_lidar_shdn_right(LIDAR_SHDN_RIGHT)
{
    _init_lidar();
}

Mapper::~Mapper() {
    delete _lidars.center;
    delete _lidars.left;
    delete _lidars.right;
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
