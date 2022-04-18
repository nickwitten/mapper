#include "mbed.h"
#include "mapper.h"
#include <list>


Serial pc(USBTX, USBRX);
std::list<point> points;

int main() {
    Mapper robot;
    Point measured_point = {0, 0};
    LIDAR_DIRECTION dirs[3] = {CENTER, LEFT, RIGHT};
    while (1) {
        for (auto dir : dirs) {
            if (!robot.plot_object(dir, measured_point)) {
                points.push_back(measured_point);  // Make sure this makes a copy
                pc.printf("Coordinate added: %ld, %ld\r\n", points.back().x, points.back().y);
            }
        }
        // pc.printf("List size: %u\r\n", points.size());

        // SIMULATE SPIN
//         int status = robot.plot_object(LEFT, measured_point);
//         if (!status) {
//             pc.printf("Coordinate: %ld, %ld\r\n", measured_point.x, measured_point.y);
//             robot.theta += M_PI / 180 * 10;
//             wait(0.5);
//         }
    }
}
