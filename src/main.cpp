#include "mbed.h"
#include "mapper.h"


Serial pc(USBTX, USBRX);

int main() {
    Mapper robot;
    while (1) {
        pc.printf("%ld\r\n", robot.read_center_dist());
    }
}
