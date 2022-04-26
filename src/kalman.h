#include "mbed.h"
#include "mapper.h"
#include "rtos.h"


extern Serial pc;
Mapper robot;


Thread turn_t;
void turn() {
    robot.target_speed = 50;
    robot.target_theta -= M_PI / 2;
    Thread::wait(3);
    robot.target_speed = 300;
}

void plot_surrounding() {
    Point measured_point = {0, 0};
    LIDAR_DIRECTION dirs[3] = {CENTER, LEFT, RIGHT};
    for (auto dir : dirs) {
        if (!robot.plot_object(dir, measured_point)) {
            pc.printf("%d, %d\r\n", measured_point.x, measured_point.y);
            if (dir == CENTER) {
                if (sqrt(pow(measured_point.x - robot.state.x, 2) + pow(measured_point.y - robot.state.y, 2)) < 150) {
                    if (turn_t.get_state() == Thread::Inactive) {
                        turn_t.start(Callback<void()>(&turn));
                    }
                }
            }
        }
    }
}

void print_state() {
    pc.printf("X: %d, Y: %d, THETA: %.1f\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI);
    pc.printf("LV: %d, RV: %d\r\n", robot.state.lv, robot.state.rv);
    pc.printf("PWML: %f, PWMR: %f\r\n", robot._pwm_l, robot._pwm_r);
    pc.printf("VOFF: %d, PWMADDL: %.2f, PWMADDR: %.2f\r\n\r\n", robot.v_off, robot.pwm_add_l, robot.pwm_add_r);
}

void print_cal() {
    std::map<float, int32_t>::iterator itr;
    pc.printf("Left samples:\r\n");
    for (itr = robot._pwm_speed_map_l.begin(); itr != robot._pwm_speed_map_l.end(); ++itr) {
        pc.printf("%.1f, %d\r\n", itr->first, itr->second);
    }
    pc.printf("Right samples:\r\n");
    for (itr = robot._pwm_speed_map_r.begin(); itr != robot._pwm_speed_map_r.end(); ++itr) {
        pc.printf("%.1f, %d\r\n", itr->first, itr->second);
    }
    pc.printf("\r\nLEFT:\r\n");
    pc.printf("\t%f (mm/s) / V\r\n", robot._pwm_speed_m_l);
    pc.printf("\t%f mm/s at 0 V\r\n", robot._pwm_speed_b_l);
    pc.printf("RIGHT:\r\n");
    pc.printf("\t%f (mm/s) / v\r\n", robot._pwm_speed_m_r);
    pc.printf("\t%f mm/s at 0 V\r\n", robot._pwm_speed_b_r);
}


int main() {
    robot.start_state_update(0.05);
    robot.control = false;
    robot.calibrate_wheel_speed();
    robot.control = true;

    pc.printf("Calibrated:\r\n");
    print_cal();

    robot.target_speed = 300;
    // robot.control = false;
    while (1) {
        plot_surrounding();
    }
}
