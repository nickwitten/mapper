#include "mbed.h"
#include "mapper.h"
#include "rtos.h"


Serial pc(USBTX, USBRX);
Mapper robot;


void plot_surrounding() {
    while(1) {
        Point measured_point = {0, 0};
        LIDAR_DIRECTION dirs[3] = {CENTER, LEFT, RIGHT};
        for (auto dir : dirs) {
            if (!robot.plot_object(dir, measured_point)) {
                pc.printf("[%ld, %ld],\r\n", measured_point.x, measured_point.y);
            }
        }
        Thread::wait(10);
    }
}

int main() {
    Thread t(&plot_surrounding);
    robot.move_forward(1000);
    t.terminate();
}
