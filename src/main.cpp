#include "mbed.h"
#include "mapper.h"
#include "rtos.h"


extern Serial pc;
Mapper robot;


void plot_surrounding() {
    Point measured_point = {0, 0};
    LIDAR_DIRECTION dirs[3] = {CENTER, LEFT, RIGHT};
    for (auto dir : dirs) {
        if (!robot.plot_object(dir, measured_point)) {
            pc.printf("[%ld, %ld],\r\n", measured_point.x, measured_point.y);
            if (dir == CENTER)  {
                float dist_from_robot = sqrt( pow(robot.state.x - measured_point.x, 2) + pow(robot.state.y - measured_point.y, 2) );
                if (dist_from_robot < 150) {
                    robot.drive(0.0);
                }
            }
        }
    }
}


int main() {
    Ticker update_poll;
    Ticker straight_poll;
    update_poll.attach<Mapper, void(Mapper::*)()>(&robot, &Mapper::update_position, 0.05);
    straight_poll.attach<Mapper, void(Mapper::*)()>(&robot, &Mapper::move_straight, 0.05);
    robot.drive(0.45);
    while (1) {
        // pc.printf("LV: %d\r\nRV: %d\r\n", robot.state.lv, robot.state.rv);
        // pc.printf("PWML: %f\r\nPWMR: %f\r\n", robot._pwm_l, robot._pwm_r);
        // pc.printf("X: %d\r\nY: %d\r\nTHETA: %.1f\r\n\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI);
        plot_surrounding();
    }
}
