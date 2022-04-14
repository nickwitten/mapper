#include "mbed.h"
#include "XNucleo53L0A1.h"
#include <stdio.h>


#define I2C_SDA_PIN p28
#define I2C_SCL_PIN p27
#define LIDAR_SHDN_0 p26
#define LIDAR_SHDN_1 p25
#define LIDAR_SHDN_2 p24


class Mapper {
public:
    Mapper():
        _i2c(I2C_SDA_PIN, I2C_SCL_PIN),
        _lidar_shdn_0(LIDAR_SHDN_0),
        _lidar_shdn_1(LIDAR_SHDN_1),
        _lidar_shdn_2(LIDAR_SHDN_2)
    {
        _init_lidar();
    }

    ~Mapper() {
        delete _lidars;
    }

    uint32_t read_center_dist() {
        uint32_t distance;
        int status = _lidars->sensor_centre->get_distance(&distance);
        if (status == VL53L0X_ERROR_NONE) {
            return distance;
        } else {
            return INT32_MAX;
        }
    }

private:
    DevI2C _i2c;
    char _LIDAR_N = 3;
    DigitalOut _lidar_shdn_0;
    DigitalOut _lidar_shdn_1;
    DigitalOut _lidar_shdn_2;
    XNucleo53L0A1* _lidars = NULL;

    void _init_lidar() {
        int status;
        /* creates the 53L0A1 expansion board singleton obj */
        _lidars = XNucleo53L0A1::instance(&_i2c, A2, D8, D2);
        //must reset sensor for an mbed reset to work
        _lidar_shdn_0 = 0;
        _lidar_shdn_1 = 0;
        _lidar_shdn_2 = 0;
        wait(0.1);
        _lidar_shdn_0 = 1;
        _lidar_shdn_1 = 1;
        _lidar_shdn_2 = 1;
        wait(0.1);
        status = _lidars->init_board();
        while (status) {
            printf("Failed to init board! \r\n");
            status = _lidars->init_board();
        }
    }
};

Serial pc(USBTX, USBRX);

int main() {
    Mapper robot;
    while (1) {
        pc.printf("%ld\r\n", robot.read_center_dist());
    }
}
