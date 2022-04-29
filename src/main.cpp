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

Thread auto_t;;
void autonomous() {
    int move_speed = 150;
    int stuck_ct = 0;  // Amount of time spent at 0 mm/s
    bool force_turn = false;  // If stuck we need to force the turn
    int16_t loop_ct = 0;  // for leds
    while (!pc_mutex.trylock()) Thread::yield();
    pc.printf("Autonomous commands starting\r\n");
    pc_mutex.unlock();
    while (1) {
        leds = 1 << ((int)floor(loop_ct++ / 10) % 4);
        robot.target_speed = move_speed;
        if (!robot.state.lv && !robot.state.rv) {
            if (++stuck_ct >= 200) {  // at least two seconds
                force_turn = true;
                stuck_ct = 0;
            }
        }
        // robot needs to turn if too close to an object
        if (d_center <= 300 || d_right <= 200 || d_left <=200 || force_turn) {
            leds = 0xF;
            robot.target_speed = -10;  // stop
            Thread::wait(50);
            robot.target_speed = 0;
            // Find the direction with the most empty space and turn to it
            float targ_theta;
            if (d_right == d_left) {
                if (robot.target_theta == 0 || robot.target_theta == M_PI) {
                    targ_theta = M_PI / 2;  // Try to keep moving in +y direction
                } else {
                    targ_theta = robot.target_theta + M_PI / 2;
                }
            } else if (d_right > d_left) {
                targ_theta = robot.target_theta - M_PI / 2;
            } else {
                targ_theta = robot.target_theta + M_PI / 2;
            }
            robot.target_theta = bound_theta(targ_theta);

            force_turn = false;
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
    // pc.printf("PWML: %f, PWMR: %f\r\n", robot._pwm_l, robot._pwm_r);
    // pc.printf("VOFF: %d, PWMADDL: %.2f, PWMADDR: %.2f\r\n\r\n", robot._v_off, robot._pwm_add_l, robot._pwm_add_r);
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
            case 'r':  // Reset
                robot.init_state();
                robot.print_state();
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

    pc.printf("reset\r\n");
    robot.start_state_update(0.05);

    while (pc.readable()) pc.getc();
    pc.printf("Calibrate? [y,n]\r\n");
    if (pc.getc() == 'y') {
        robot.control = false;
        pc.printf("Press any key to calibrate the left wheel\r\n");
        pc.getc();
        robot.calibrate_left_wheel();
        print_cal();
        while (pc.readable()) pc.getc();
        pc.printf("Press any key to calibrate the right wheel\r\n");
        pc.getc();
        robot.calibrate_right_wheel();
        print_cal();
        robot.control = true;
    }
#ifdef DEFAULT_AUTO
    auto_t.start(Callback<void()>(&autonomous));
#endif
    robot.control = true;
    while (1) {
        dispatch();
        plot_surrounding();
        Thread::wait(10);;
    }
}
