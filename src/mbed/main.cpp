#include "mbed.h"
#include "mapper.h"
#include "rtos.h"
#include <climits>


Serial pi(USBTX, USBRX);
Mutex pi_mutex;
Mapper robot;
BusOut leds(LED1, LED2, LED3, LED4);
DigitalIn HEADLESS(p23, PullDown);  // Run without connection to Pi


volatile int d_center = INT_MAX;
volatile int d_left = INT_MAX;
volatile int d_right = INT_MAX;
void plot_surrounding() {
    Point measured_point = {0, 0};
    while (!pi_mutex.trylock()) Thread::yield();
    if (!robot.plot_object(CENTER, measured_point)) {
        pi.printf("%d, %d\r\n", measured_point.x, measured_point.y);
        d_center = sqrt(pow(measured_point.x - robot.state.x, 2) + pow(measured_point.y - robot.state.y, 2));
    } else d_center = INT_MAX;
    if (!robot.plot_object(LEFT, measured_point)) {
        pi.printf("%d, %d\r\n", measured_point.x, measured_point.y);
        d_left = sqrt(pow(measured_point.x - robot.state.x, 2) + pow(measured_point.y - robot.state.y, 2));
    } else d_left = INT_MAX;
    if (!robot.plot_object(RIGHT, measured_point)) {
        pi.printf("%d, %d\r\n", measured_point.x, measured_point.y);
        d_right = sqrt(pow(measured_point.x - robot.state.x, 2) + pow(measured_point.y - robot.state.y, 2));
    } else d_right = INT_MAX;
    pi_mutex.unlock();
}

Thread auto_t;
void autonomous() {
    int move_speed = 100;
    int stuck_ct = 0;  // Amount of time spent at 0 mm/s
    bool force_turn = false;  // If stuck we need to force the turn
    int16_t loop_ct = 0;  // for leds
    while (!pi_mutex.trylock()) Thread::yield();
    pi.printf("Autonomous commands starting\r\n");
    pi_mutex.unlock();
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
        if (d_center <= 275 || d_right <= 0 || d_left <= 0 || force_turn) {
            leds = 0xF;
            robot.target_speed = -10;  // stop
            Thread::wait(50);
            robot.target_speed = 0;
            Thread::wait(450);
            // Find the direction with the most empty space and turn to it
            float targ_theta;
            if ((d_right == d_left) || (d_right > 200 && d_left > 200)) {
                if (robot.target_theta <= (float)M_PI && robot.target_theta != (float)(M_PI / 2.0)) {
                    targ_theta = M_PI / 2;  // Try to keep moving in +y direction
                } else {
                    if (d_right > d_left) {
                        targ_theta = robot.target_theta - M_PI / 2;
                    } else {
                        targ_theta = robot.target_theta + M_PI / 2;
                    }
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
    while (!pi_mutex.trylock()) Thread::yield();
    pi.printf("X: %d, Y: %d, THETA: %.f, TARGET THETA: %.f, TARGET SPEED: %d\r\n", robot.state.x, robot.state.y, robot.state.theta * 180 / M_PI, robot.target_theta * 180 / M_PI, robot.target_speed);
    pi.printf("LV: %d, RV: %d\r\n", robot.state.lv, robot.state.rv);
    pi_mutex.unlock();
}

void restart_map() {
    while (!pi_mutex.trylock()) Thread::yield();
    pi.printf("reset\r\n");
    pi_mutex.unlock();
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
    while (pi.readable()) {
        char IN = pi.getc();
        switch (IN) {
            case 13:  // Enter key
                print_state();
                break;
            case 65:  // Up key
            case 'k':
                forward();
                print_state();
                break;
            case 66:  // Down key
            case 'j':
                stop();
                print_state();
                break;
            case 67:  // Right key
            case 'l':
                turn_right();
                print_state();
                break;
            case 68:  // Left key
            case 'h':
                turn_left();
                print_state();
                break;
            case 'a':
                toggle_automation();
                break;
            case 'r':  // Reset
                robot.init_state();
                restart_map();
                print_state();
            default:
                break;
        }
    }
}

int main() {
    robot.start_state_update(0.05);
    if (HEADLESS) {
        robot.default_cal();
        auto_t.start(Callback<void()>(&autonomous));
    } else {
        while (pi.readable()) pi.getc();
        pi.printf("Calibrate? [y,n]\r\n");
        if (pi.getc() == 'y') {
            robot.control = false;
            pi.printf("Press any key to calibrate the left wheel\r\n");
            pi.getc();
            robot.calibrate_left_wheel();
            robot.print_cal(pi);
            while (pi.readable()) pi.getc();
            pi.printf("Press any key to calibrate the right wheel\r\n");
            pi.getc();
            robot.calibrate_right_wheel();
            robot.print_cal(pi);
            robot.control = true;
        } else {
            robot.default_cal();
        }
    }
    robot.init_state();  // Restart robot state
    restart_map();  // Restart plotting map

    while (1) {
        dispatch();
        plot_surrounding();
        Thread::wait(10);;
    }
}
