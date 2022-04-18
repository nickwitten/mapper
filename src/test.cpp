#include "mbed.h"
#include "Motor.h"
#include "HALLFX_ENCODER.h"
#include "mapper.h"

Motor ml(p22, p7, p8); // pwm, fwd, rev
Motor mr(p21, p5, p6);
DigitalOut led1(LED1);
Serial pc(USBTX, USBRX); // tx, rx
HALLFX_ENCODER left_encoder(p10);
HALLFX_ENCODER right_encoder(p9);


int dist = 0;
int old_L = 0;
int old_R = 0;
void initialize_movement() {
    old_L = 0;
    old_R = 0;
    ml.speed(0.3);
    mr.speed(0.3);
    left_encoder.reset();
    right_encoder.reset();
}
void move_straight() {
    //led1 = 1;
    int lr = left_encoder.read();
    int rr = right_encoder.read();
    float ld = 0.05672320068 * (lr); //unit in cm
    float rd = 0.05672320068 * (rr);
    if (ld > rd) {
        //need turn left
        mr.speed(0.4);
        ml.speed(0.2);
    }
    else if (ld < rd) {
        //need to turn right
        ml.speed(0.4);
        mr.speed(0.2);
    }
    old_L += lr;
    old_R += rr;
    left_encoder.reset();
    right_encoder.reset();
    //pc.printf("old_L: %f, old_R: %f\n\r", old_L,old_R);
}
bool turn_angle(float angle) {
    int lr = left_encoder.read();
    int rr = right_encoder.read();
    if (lr > (int)385 * angle / 180.0 && rr > (int)385 * angle / 180.0) {
        ml.speed(0);
        mr.speed(0);
        return false;
    }
    //wait(0.5);
    //pc.printf("L: %d, R: %d\n\r", lr, rr);
    return true;
}
bool move_forward(float dist) {
    int lr = left_encoder.read();
    int rr = right_encoder.read();
    float ld = 0.05672320068 * (lr + old_L); //unit in cm
    float rd = 0.05672320068 * (rr + old_R);
    float total_d = (ld + rd) / 2;
    //pc.printf("total_d: %f\n\r",total_d);
    if (total_d >= dist - 4.5) {
        ml.speed(0);
        mr.speed(0);
        return false;
    }
    wait(0.05);

    return true;
}
int main() {
    /*Mapper robot;
    float angle = 0.0;
    dist = robot.read_dist(CENTER);
    if (dist<5）{
        angle = 90.0;
        while (turn_angle(angle)) {
            ml.speed(0.3);
            mr.speed(-0.3);
        }
    }
    move_straight();
    */
    //move1
    initialize_movement();
    while (move_forward(20.0)) {
        move_straight();
    }
    //turn angle clockwise
    float angle = 180.0;
    while (turn_angle(angle)) {
        ml.speed(0.3);
        mr.speed(-0.3);
    }
    //move2
    initialize_movement();
    while (move_forward(20.0)) {
        move_straight();
    }
}
