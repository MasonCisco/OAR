#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>
#include <lgpio.h>

//Top Left Arm
static const int servo_TL = 22;
static const int motor_TLA = 5;
static const int motor_TLB = 6;

//Top Right Arm
static const int servo_TR = 23;
static const int motor_TRA = 20;
static const int motor_TRB = 21;

//Bottom Left Arm
static const int servo_BL = 27;
static const int motor_BLA = 26;
static const int motor_BLB = 16;

//Bottom Right Arm
static const int servo_BR = 17;
static const int motor_BRA = 13;
static const int motor_BRB = 19;

const int outputPins[] = {
    servo_TL, motor_TLA, motor_TLB,
    servo_TR, motor_TRA, motor_TRB,
    servo_BL, motor_BLA, motor_BLB,
    servo_BR, motor_BRA, motor_BRB,
};


int main() {

    float duty = 50.0f;
    int h = lgGpiochipOpen(4);
    if (h < 0) return 1;

    for (int pin : outputPins){
        lgGpioClaimOutput(h, 0, pin, 0);
    }

    lgTxServo(h,servo_TL, 1500, 50, 0, 0);
    lgTxPwm(h, motor_TLA, 1500, duty, 0, 0);

    sleep(5);

    lgTxServo(h, servo_TL, 0, 50, 0, 0);
    lgTxPwm(h, motor_TLA, 1000, duty, 0, 0);
    return 0;
}




