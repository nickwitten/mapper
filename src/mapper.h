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
    Mapper();
    ~Mapper();
    uint32_t read_center_dist();

private:
    DevI2C _i2c;
    DigitalOut _lidar_shdn_0;
    DigitalOut _lidar_shdn_1;
    DigitalOut _lidar_shdn_2;
    XNucleo53L0A1* _lidars = NULL;
    void _init_lidar();
};
