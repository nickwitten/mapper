#include "mbed.h"
#include "mapper.h"
#include "rtos.h"


extern Serial pc;
Mapper robot;


// void plot_surrounding() {
//     while(1) {
//         Point measured_point = {0, 0};
//         LIDAR_DIRECTION dirs[3] = {CENTER, LEFT, RIGHT};
//         for (auto dir : dirs) {
//             if (!robot.plot_object(dir, measured_point)) {
//                 pc.printf("[%ld, %ld],\r\n", measured_point.x, measured_point.y);
//             }
//         }
//         Thread::wait(10);
//     }
// }


int main() {
    robot.start_state_update(0.05);
    robot.control = false;
    robot.calibrate_wheel_speed();
    robot.control = true;

    pc.printf("Calibrated:\r\n");
    std::map<float, int32_t>::iterator itr;
    for (itr = robot._pwm_speed_map_r.begin(); itr != robot._pwm_speed_map_r.end(); ++itr) {
        pc.printf("%.2f: %d\r\n", itr->first, itr->second);
    }
    pc.printf("LEFT:\r\n");
    pc.printf("\t%f (mm/s) / V\r\n", robot._pwm_speed_m_l);
    pc.printf("\t%f mm/s at 0 V\r\n", robot._pwm_speed_b_l);
    pc.printf("RIGHT:\r\n");
    pc.printf("\t%f (mm/s) / v\r\n", robot._pwm_speed_m_r);
    pc.printf("\t%f mm/s at 0 V\r\n", robot._pwm_speed_b_r);

    // pc.printf("Setting speed to 200 mm/s");
    // robot.target_speed = 200;
    // wait(10 * robot._dt);
    // pc.printf("PWML: %f\r\nPWMR: %f\r\n", robot._pwm_l, robot._pwm_r);
    // for (int i = 0; i < 5; i++) {
    //     pc.printf("LV: %d\r\nRV: %d\r\n", robot.state.lv, robot.state.rv);
    //     pc.printf("X: %d\r\nY: %d\r\nTHETA: %.1f\r\n\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI);
    //     wait(20 * robot._dt);
    // }

    // pc.printf("Setting speed to 0 mm/s");
    // robot.target_speed = 0;
    // wait(10 * robot._dt);
    // pc.printf("LV: %d\r\nRV: %d\r\n", robot.state.lv, robot.state.rv);
    // pc.printf("X: %d\r\nY: %d\r\nTHETA: %.1f\r\n\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI);

    pc.printf("Setting theta to 0\r\n");
    robot.target_theta = 0;
    robot.target_speed = 200;
    // robot.control = false;
    while (1) {
        pc.printf("X: %d, Y: %d, THETA: %.1f\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI);
        pc.printf("LV: %d, RV: %d\r\n", robot.state.lv, robot.state.rv);
        pc.printf("PWML: %f, PWMR: %f\r\n", robot._pwm_l, robot._pwm_r);
        pc.printf("VOFF: %d, PWMADDL: %.2f, PWMADDR: %.2f\r\n\r\n", robot.v_off, robot.pwm_add_l, robot.pwm_add_r);
        wait(0.5);
    }
}
