// OwnC.cpp
// Build:  g++ OwnC.cpp -o main -llgpio -lm
// Run:    ./main

#include <lgpio.h>
#include <linux/joystick.h>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <thread>
#include <cstdlib> 

// ============================================================
//  SETTINGS
// ============================================================
static const int    GPIO_CHIP   = 0;       // Pi 4 and earlier: 0, Pi 5: 4
static const int    AXIS        = 0;       // joystick axis that drives everything
static const int    DEADZONE_ON = 8000;    // stick must pass this to start moving
static const int    DEADZONE_OFF = 6000;   // stick must fall below this to stop (hysteresis)
static const double MAX_DUTY    = 50.0;    // top motor duty (0-100). Lower = gentler
static const double RAMP_PER_TICK = 0.05;  // speed change per tick (0.05 @ 10 ms = 0.2 s to full)
static const int    MOTOR_HZ    = 1000;
static const int    SERVO_HZ    = 50;
static const int    TICK_MS     = 10;

// true  = servos are held at SERVO_US the whole time (what the stress test does)
// false = servos only get pulses while the arms are moving (your original behavior)
static const bool   SERVO_ALWAYS_ON = true;

// ============================================================
//  PINS (BCM numbers)
// ============================================================
static const int SERVO_PINS[4] = { 22, 17, 23, 27 };   // TL, TR, BL, BR

struct Motor { int in1; int in2; };
static const Motor MOTORS[4] = {
    {  5,  6 },   // Top Left
    { 13, 19 },   // Top Right
    { 20, 21 },   // Bottom Left
    { 26, 16 }    // Bottom Right
};

// ============================================================

static volatile sig_atomic_t g_running = 1;
static void onSignal(int) { g_running = 0; }

static int  g_motorDir[4]  = { 0, 0, 0, 0 };   // 0 = stop, 1 = fwd, -1 = rev
static bool g_servoOn[4]   = { false, false, false, false };

static void setMotor(int h, int idx, double speed /* -1..1 */) {
    const Motor& m = MOTORS[idx];
    const double deadband = 0.03;

    int dir = 0;
    if (speed >= deadband)       dir = 1;
    else if (speed <= -deadband) dir = -1;

    // Direction changed: shut both pins off and drive them low first
    if (dir != g_motorDir[idx]) {
        lgTxPwm(h, m.in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, m.in2, MOTOR_HZ, 0.0, 0, 0);
        lgGpioWrite(h, m.in1, 0);
        lgGpioWrite(h, m.in2, 0);
        g_motorDir[idx] = dir;
    }
    if (dir == 0) return;

    double duty = std::min(std::fabs(speed) * MAX_DUTY, MAX_DUTY);
    lgTxPwm(h, (dir > 0) ? m.in1 : m.in2, MOTOR_HZ, duty, 0, 0);
}

static int g_servoPos[4] = {0, 0, 0, 0};

static void setServo(int h, int idx, int pos) {
    if (pos == g_servoPos[idx]) return; 
        lgTxServo(h, SERVO_PINS[idx], pos, SERVO_HZ, 0, 0);
        g_servoPos[idx] = pos;
}

static void stopAll(int h) {
    for (int i = 0; i < 4; i++) {
        lgTxServo(h, SERVO_PINS[i], 0, SERVO_HZ, 0, 0);
        lgTxPwm(h, MOTORS[i].in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, MOTORS[i].in2, MOTOR_HZ, 0.0, 0, 0);
        g_servoPos[i] = 0;
        g_motorDir[i] = 0;
        g_servoOn[i] = false;
    }
}

int main() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    int fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::perror("open joystick");
        std::fprintf(stderr, "Is the controller paired?\n");
        return 1;
    }

    int h = lgGpiochipOpen(GPIO_CHIP);
    if (h < 0) {
        std::fprintf(stderr, "lgGpiochipOpen(%d) failed: %s\n", GPIO_CHIP, lguErrorText(h));
        close(fd);
        return 1;
    }
    for (int i = 0; i < 4; i++) {
        int pins[3] = { SERVO_PINS[i], MOTORS[i].in1, MOTORS[i].in2 };
        for (int p : pins) {
            int rc = lgGpioClaimOutput(h, 0, p, 0);
            if (rc < 0) {
                std::fprintf(stderr, "Could not claim GPIO %d: %s\n", p, lguErrorText(rc));
                lgGpiochipClose(h);
                close(fd);
                return 1;
            }
        }
    }

    // Start servos at center and give them a moment, like the stress test
    if (SERVO_ALWAYS_ON) {
        for (int i = 0; i < 4; i++) setServo(h, i, 1500);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    int    axisValue = 0;                       // latest raw value of AXIS
    int    axis1Value = 0;
    int    target    = 0;                       // -1, 0, 1  (your deadzone logic)
    double speed[4]  = { 0, 0, 0, 0 };          // ramped speed per motor

    std::printf("Running. Ctrl+C to stop.\n");

    while (g_running) {
        // ---- read all pending events ----
        js_event e;
        ssize_t n;

        while ((n = read(fd, &e, sizeof(e))) == (ssize_t)sizeof(e)) {
            if ((e.type & ~JS_EVENT_INIT) != JS_EVENT_AXIS) continue;
            if (e.number == AXIS) axisValue = e.value;
            if (e.number == 1) axis1Value = e.value;
        }

        if (n == 0 || (n < 0 && errno != EAGAIN)) {
            std::fprintf(stderr, "\nController lost (%s). Stopping.\n",
                         n == 0 ? "EOF" : std::strerror(errno));
            break;
        }

        // ---- your logic: deadzone decides direction (with hysteresis) ----
        if (axisValue >  DEADZONE_ON)        target =  1;
        else if (axisValue < -DEADZONE_ON)   target = -1;
        else if (std::abs(axisValue) < DEADZONE_OFF) target = 0;
        // between OFF and ON: keep the previous target

        // ---- the other program's process: ramp, then apply every tick ----
        bool anyMoving = false;
        for (int i = 0; i < 4; i++) {
            double diff = std::clamp((double)target - speed[i], -RAMP_PER_TICK, RAMP_PER_TICK);
            speed[i] += diff;
            setMotor(h, i, speed[i]);
            if (std::fabs(speed[i]) >= 0.03) anyMoving = true;
        }

        int servoPosTLBR = 1500;
        int servoPosTRBL = 1500;

        if(axis1Value > 10000 && axisValue > 1000){ //Quad 1
             servoPosTLBR = 500;
             servoPosTRBL = 1500;
        }
        else if(axis1Value < -10000 && axisValue > 1000) { //Quad 2
            servoPosTLBR = 1500;
            servoPosTRBL = 2300;
        }
        else if(axis1Value < -10000 && axisValue < -1000){ //Quad 3
             servoPosTLBR = 500;
             servoPosTRBL = 1500;
        }
        else if(axis1Value > 10000 && axisValue < -1000){ //Quad 4
             servoPosTLBR = 1500;
             servoPosTRBL = 2300;
        }
        

        for(int i = 0; i < 2; i++) setServo(h, i, servoPosTLBR);
        for(int i = 2; i < 4; i++) setServo(h, i, servoPosTRBL);

        std::this_thread::sleep_for(std::chrono::milliseconds(TICK_MS));
    }

    std::printf("\nStopping all outputs...\n");
    stopAll(h);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (int i = 0; i < 4; i++) {
        lgGpioWrite(h, SERVO_PINS[i], 0);
        lgGpioWrite(h, MOTORS[i].in1, 0);
        lgGpioWrite(h, MOTORS[i].in2, 0);
    }
    lgGpiochipClose(h);
    close(fd);
    std::printf("Done.\n");
    return 0;
}