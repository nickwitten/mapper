#include "mbed.h"
#include "mapper.h"
#include "rtos.h"
#include <climits>

//#define DEFAULT_AUTO


extern Serial pc;
Mapper robot;
BusOut leds(LED1, LED2, LED3, LED4);
Mutex pc_mutex;


volatile int d_center = INT_MAX;
volatile int d_left = INT_MAX;
volatile int d_right = INT_MAX;
void plot_surrounding() {
    Point measured_point = {0, 0};
    while (!pc_mutex.trylock()) Thread::yield();
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
    pc_mutex.unlock();
}

// for (auto dir : dirs) {
//     if (!robot.plot_object(dir, measured_point)) {
//         pc.printf("%d, %d\r\n", measured_point.x, measured_point.y);
//     }
// }
//

Thread auto_t;;
void autonomous() {
    int move_speed = 150;
    while (!pc_mutex.trylock()) Thread::yield();
    pc.printf("Autonomous commands starting\r\n");
    pc_mutex.unlock();
    while (1) {
        robot.target_speed = move_speed;
        if (d_center <= 300) {
            robot.target_speed = -10;
            Thread::wait(50);
            robot.target_speed = 0;
            float targ_theta;
            if (d_right == d_left) {
                if (robot.target_theta == 0 || robot.target_theta == M_PI) {
                    targ_theta = M_PI / 2;
                } else {
                    targ_theta = robot.target_theta + M_PI / 2;
                }
            } else if (d_right > d_left) {
                targ_theta = robot.target_theta - M_PI / 2;
            } else {
                targ_theta = robot.target_theta + M_PI / 2;
            }
            robot.target_theta = bound_theta(targ_theta);

            Thread::wait(3000);
        } else {
            Thread::wait(10);
        }
    }
}

void print_state() {
    while (!pc_mutex.trylock()) Thread::yield();
    pc.printf("X: %d, Y: %d, THETA: %.f, TARGET THETA: %.f, TARGET SPEED: %d\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI, robot.target_theta * 180 / M_PI, robot.target_speed);
    pc.printf("LV: %d, RV: %d\r\n", robot.state.lv, robot.state.rv);
    pc.printf("PWML: %f, PWMR: %f\r\n", robot._pwm_l, robot._pwm_r);
    pc.printf("VOFF: %d, PWMADDL: %.2f, PWMADDR: %.2f\r\n\r\n", robot._v_off, robot._pwm_add_l, robot._pwm_add_r);
    pc_mutex.unlock();
}

void print_cal() {
    while (!pc_mutex.trylock()) Thread::yield();
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
    pc_mutex.unlock();
}

void turn_left() {
    if (robot.target_speed > 0) robot.target_speed = 100;
    robot.target_theta = bound_theta(robot.target_theta + M_PI / 2);
}

void turn_right() {
    if (robot.target_speed > 0) robot.target_speed = 100;
    robot.target_theta = bound_theta(robot.target_theta - M_PI / 2);
}

void forward() {
    robot.target_speed += 100;
}

void stop() {
    robot.target_speed -= 100;
}

void toggle_automation() {
    Thread::State state = auto_t.get_state();
    if (state == Thread::Inactive || state == Thread::Deleted) {
        if (auto_t.start(Callback<void()>(&autonomous)) != 0) {
            error("Autonomous thread could not be started.");
        }
    } else {
        robot.target_speed = 0;
        auto_t.terminate();
    }
}

void dispatch() {
    while (pc.readable()) {
        char IN = pc.getc();
        switch (IN) {
            case 13:  // enter key
                print_state();
                break;
            case 'k':
                forward();
                print_state();
                break;
            case 'j':
                stop();
                print_state();
                break;
            case 'l':
                turn_right();
                print_state();
                break;
            case 'h':
                turn_left();
                print_state();
                break;
            case 'a':
                toggle_automation();
                break;
            default:
                break;
        }
    }
}

int main() {
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.1, 17));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.3, 40));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.4, 78));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.5, 145));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.6, 188));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.7, 290));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.8, 337));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(0.9, 388));
    robot._pwm_speed_map_l.insert(std::pair<float, int32_t>(1.0, 403));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(0.3, 40));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(0.4, 183));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(0.5, 68));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(0.6, 211));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(0.7, 280));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(0.8, 295));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(0.9, 298));
    robot._pwm_speed_map_r.insert(std::pair<float, int32_t>(1.0, 308));
    linearize_map(robot._pwm_speed_map_l, &robot._pwm_speed_m_l, &robot._pwm_speed_b_l);
    linearize_map(robot._pwm_speed_map_r, &robot._pwm_speed_m_r, &robot._pwm_speed_b_r);
#ifdef DEFAULT_AUTO
    auto_t.start(Callback<void()>(&autonomous));
#endif

    pc.printf("reset\r\n");
    robot.start_state_update(0.05);
    if (0) {
        robot.control = false;
        robot.calibrate_wheel_speed();
        print_cal();
        robot.control = true;
    }
    robot.control = true;
    while (1) {
        dispatch();
        plot_surrounding();
        Thread::wait(10);;
    }
}