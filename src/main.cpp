#include "mbed.h"
#include "mapper.h"
#include "rtos.h"
#include <climits>


extern Serial pc;
Mapper robot;
BusOut leds(LED1, LED2, LED3, LED4);


Thread turn_t;
volatile int d_center = INT_MAX;
volatile int d_left = INT_MAX;
volatile int d_right = INT_MAX;
void turn() {
    while (1) {
        if (d_center <= 250) {
            robot.target_speed = 0;
            float targ_theta;
            if (d_right == d_left) {
                if (robot.target_theta == 0 || robot.target_theta == M_PI) {
                    targ_theta = M_PI / 2;
                } else {
                    targ_theta = robot.target_theta + M_PI / 2;
                }
                leds = ~leds;
            } else if (d_right > d_left) {
                targ_theta = robot.target_theta - M_PI / 2;
            } else {
                targ_theta = robot.target_theta + M_PI / 2;
            }

            if (targ_theta < 0) {
                robot.target_theta = targ_theta + 2 * M_PI;
            } else if (targ_theta > 2 * M_PI) {
                robot.target_theta = targ_theta - 2 * M_PI;
            } else {
                robot.target_theta = targ_theta;
            }

            Thread::wait(3000);
            robot.target_speed = 350;
        }
        Thread::yield();
    }
}

void plot_surrounding() {
    Point measured_point = {0, 0};
    if (!robot.plot_object(CENTER, measured_point)) {
        pc.printf("%d, %d\r\n", measured_point.x, measured_point.y);
        d_center = sqrt(pow(measured_point.x - robot.state.x, 2) + pow(measured_point.y - robot.state.y, 2));
    } else d_center = INT_MAX;
    if (!robot.plot_object(LEFT, measured_point)) {
        pc.printf("%d, %d\r\n", measured_point.x, measured_point.y);
        d_left = sqrt(pow(measured_point.x - robot.state.x, 2) + pow(measured_point.y - robot.state.y, 2));
    } else d_left = INT_MAX;
    if (!robot.plot_object(RIGHT, measured_point)) {
        pc.printf("%d, %d\r\n", measured_point.x, measured_point.y);
        d_right = sqrt(pow(measured_point.x - robot.state.x, 2) + pow(measured_point.y - robot.state.y, 2));
    } else d_right = INT_MAX;

    // for (auto dir : dirs) {
    //     if (!robot.plot_object(dir, measured_point)) {
    //         pc.printf("%d, %d\r\n", measured_point.x, measured_point.y);
    //     }
    // }
    //
}

void print_state() {
    pc.printf("X: %d, Y: %d, THETA: %.f, TARGET THETA: %.f\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI, robot.target_theta * 180 / M_PI);
    pc.printf("LV: %d, RV: %d\r\n", robot.state.lv, robot.state.rv);
    pc.printf("PWML: %f, PWMR: %f\r\n", robot._pwm_l, robot._pwm_r);
    pc.printf("VOFF: %d, PWMADDL: %.2f, PWMADDR: %.2f\r\n\r\n", robot._v_off, robot._pwm_add_l, robot._pwm_add_r);
}

void print_cal() {
    std::map<float, int32_t>::iterator itr;
    pc.printf("Left samples:\r\n");
    for (itr = robot._pwm_speed_map_l.begin(); itr != robot._pwm_speed_map_l.end(); ++itr) {
        pc.printf("robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(%.1f, %d));\r\n", itr->first, itr->second);
    }
    pc.printf("Right samples:\r\n");
    for (itr = robot._pwm_speed_map_r.begin(); itr != robot._pwm_speed_map_r.end(); ++itr) {
        pc.printf("robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(%.1f, %d));\r\n", itr->first, itr->second);
    }
    pc.printf("\r\nLEFT:\r\n");
    pc.printf("\t%f (mm/s) / V\r\n", robot._pwm_speed_m_l);
    pc.printf("\t%f mm/s at 0 V\r\n", robot._pwm_speed_b_l);
    pc.printf("RIGHT:\r\n");
    pc.printf("\t%f (mm/s) / v\r\n", robot._pwm_speed_m_r);
    pc.printf("\t%f mm/s at 0 V\r\n\r\n", robot._pwm_speed_b_r);
}


int main() {
    robot.start_state_update(0.05);
    robot.control = false;
    if (1) {
        robot.calibrate_wheel_speed();
        pc.printf("Calibrated:\r\n");
    }
    robot.control = true;
    pc.printf("reset\r\n");

    print_cal();

    plot_surrounding();  // Plot once before moving
    turn_t.start(Callback<void()>(&turn));
    robot.target_speed = 350;
    // robot.control = false;
    while (1) {
        // print_state();
        plot_surrounding();
        Thread::yield();
    }
}
