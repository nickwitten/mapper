#include "mbed.h"
#include "VL53L0X.h"

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


class Mapper {
public:
    Mapper();
    ~Mapper();
    uint32_t read_dist(LIDAR_DIRECTION dir);
    int32_t x = 0;  // X coordinate relative to start
    int32_t y = 0;  // Y coordinate relative to start
private:
    DevI2C _i2c;
    DigitalOut _lidar_shdn_center;
    DigitalOut _lidar_shdn_left;
    DigitalOut _lidar_shdn_right;
    lidars _lidars;
    void _init_lidar();
};
