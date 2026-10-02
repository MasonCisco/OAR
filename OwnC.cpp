#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>
#include <lgpio.h>
#include <cstdio>
#include <vector>
#include <csignal>
#include <cerrno>

std::vector<int> axes (8,0), buttons(16,0);

volatile std::sig_atomic_t stop = 0;
void onSignal(int) { stop = 1; }

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

    int h = lgGpiochipOpen(0);
    if (h < 0) return 1;

     for (int pin : outputPins){
        lgGpioClaimOutput(h, 0, pin, 0);
    }

    float duty = 50.0f;
    
    int js = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if(js < 0){
        std::perror("open joystick");
        std::fprintf(stderr, "Is the controller paired?\n");
    lgGpiochipClose(h);
    return 1;
    }   

    std::signal(SIGINT, onSignal);
    bool running = true;
    while (running && !stop){
    js_event e;
    while (read(js,&e, sizeof(e)) == (ssize_t)sizeof(e)){
        bool init = e.type & JS_EVENT_INIT;
        switch(e.type & ~JS_EVENT_INIT){
            case JS_EVENT_AXIS:
                if(e.number < axes.size()) axes[e.number] = e.value;
                if(!init)
                    std::printf("axis %u: %6d\n", e.number, e.value);
                break;

            case JS_EVENT_BUTTON:
                if(e.number < buttons.size()) buttons[e.number] = e.value;
                if(!init)
                    std::printf("Button %u: %s\n", e.number, e.value ? "PRESSED" : "released");
                break;
        }
    }

    if (e.number == 0 && e.value > 10000){
        lgTxServo(h, servo_TL, 1500, 50, 0, 0);
        lgTxPwm(h, motor_TLA, 1500, duty, 0, 0);
    }
    else{
        lgTxServo(h, servo_TL, 0, 50, 0, 0);
        lgTxPwm(h, motor_TLA, 0, duty, 0, 0);
    }

    if (errno != EAGAIN) running = false;
    usleep(10000);

    }

    lgTxServo(h, servo_TL, 0, 50, 0, 0);
    lgTxPwm(h, motor_TLA, 0, duty, 0, 0);
    close(js);
    lgGpiochipClose(h);
    return 0;
}




