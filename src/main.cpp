#include "mbed.h"
#include "mapper.h"
#include <list>


Serial pc(USBTX, USBRX);
typedef struct point {
    int32_t x;
    int32_t y;
} point;
std::list<point> points;

int main() {
    Mapper robot;
    point measured_point;
    while (1) {
        auto dist = robot.read_dist(RIGHT);
        if (dist <= 500) {
            measured_point.x = robot.x;
            measured_point.y = robot.y + dist;
            points.push_back(measured_point);
            pc.printf("Coordinate added: %ld, %ld\r\n", points.back().x, points.back().y);
        }
        pc.printf("List size: %u\r\n", points.size());
        // wait(1);
    }
}
