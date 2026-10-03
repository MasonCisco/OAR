#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>
#include <lgpio.h>
#include <cstdio>
#include <cmath>
#include <vector>
#include <csignal>
#include <cerrno>
#include <cstring>
#include <algorithm>

// ============================================================
//  SETTINGS
// ============================================================
static const int    GPIO_CHIP = 0;        // Pi 4 and earlier: 0, Pi 5: 4
static const int    deadzone  = 8000;
static const int    AXIS      = 0;
static const double MAX_DUTY  = 50.0;     // top motor duty (0-100). Lower = gentler
static const int    MOTOR_HZ  = 1000;
static const int    SERVO_HZ  = 50;
static const int    SERVO_US  = 1500;
static const int    TICK_US   = 20000;    // 20 ms loop

// Fraction of full speed added per tick. 0.01 = ~1 s from 0 to 50% duty.
// Lower = softer start and slower reversal.
static const double RAMP_PER_TICK = 0.01;

// ============================================================
//  PINS (BCM numbers)
// ============================================================
static const int SERVO_PINS[4] = { 22, 23, 27, 17 };   // TL, TR, BL, BR

struct Motor { int in1; int in2; };
static const Motor MOTORS[4] = {
    {  5,  6 },   // Top Left
    { 20, 21 },   // Top Right
    { 26, 16 },   // Bottom Left
    { 13, 19 }    // Bottom Right
};

// ============================================================
//  STATE
// ============================================================
std::vector<int> axes(8, 0), buttons(16, 0);

volatile std::sig_atomic_t stop = 0;
void onSignal(int) { stop = 1; }

static int    g_motorDir[4]   = { 0, 0, 0, 0 };
static double g_speed[4]      = { 0, 0, 0, 0 };   // -1.0 .. 1.0, ramped
static bool   g_servoOn[4]    = { false, false, false, false };

// ============================================================
//  OUTPUT HELPERS
// ============================================================
static void setMotor(int h, int idx, double speed) {
    const Motor& m = MOTORS[idx];
    const double deadband = 0.05;

    int dir = 0;
    if (speed >= deadband)       dir = 1;
    else if (speed <= -deadband) dir = -1;

    if (dir != g_motorDir[idx]) {
        lgTxPwm(h, m.in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, m.in2, MOTOR_HZ, 0.0, 0, 0);
        lgGpioWrite(h, m.in1, 0);
        lgGpioWrite(h, m.in2, 0);
        g_motorDir[idx] = dir;
    }
    if (dir == 0) return;

    double duty = std::min(std::fabs(speed) * 100.0, MAX_DUTY);
    lgTxPwm(h, (dir > 0) ? m.in1 : m.in2, MOTOR_HZ, duty, 0, 0);
}

static void setServo(int h, int idx, bool on) {
    if (on != g_servoOn[idx]) {
        lgTxServo(h, SERVO_PINS[idx], on ? SERVO_US : 0, SERVO_HZ, 0, 0);
        g_servoOn[idx] = on;
    }
}

static void stopAll(int h) {
    for (int i = 0; i < 4; i++) {
        lgTxServo(h, SERVO_PINS[i], 0, SERVO_HZ, 0, 0);
        lgTxPwm(h, MOTORS[i].in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, MOTORS[i].in2, MOTOR_HZ, 0.0, 0, 0);
        g_motorDir[i] = 0;
        g_servoOn[i] = false;
    }
}

// ============================================================
//  MAIN
// ============================================================
int main() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    int h = lgGpiochipOpen(GPIO_CHIP);
    if (h < 0) {
        std::fprintf(stderr, "lgGpiochipOpen(%d) failed: %s\n", GPIO_CHIP, lguErrorText(h));
        return 1;
    }

    for (int i = 0; i < 4; i++) {
        int pins[3] = { SERVO_PINS[i], MOTORS[i].in1, MOTORS[i].in2 };
        for (int p : pins) {
            int rc = lgGpioClaimOutput(h, 0, p, 0);
            if (rc < 0) {
                std::fprintf(stderr, "Could not claim GPIO %d: %s\n", p, lguErrorText(rc));
                lgGpiochipClose(h);
                return 1;
            }
        }
    }

    int js = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
    if (js < 0) {
        std::perror("open joystick");
        std::fprintf(stderr, "Is the controller paired?\n");
        lgGpiochipClose(h);
        return 1;
    }

    std::printf("Running. Ctrl+C to stop.\n");
    bool running = true;

    while (running && !stop) {

        js_event e;
        while (read(js, &e, sizeof(e)) == (ssize_t)sizeof(e)) {
            switch (e.type & ~JS_EVENT_INIT) {
                case JS_EVENT_AXIS:
                    if (e.number < axes.size()) axes[e.number] = e.value;
                    break;
                case JS_EVENT_BUTTON:
                    if (e.number < buttons.size()) buttons[e.number] = e.value;
                    break;
            }
        }
        int readErr = errno;

        // Target speed from the stick (fixed magnitude, direction only)
        double target = 0.0;
        if (axes[AXIS] > deadzone)       target =  MAX_DUTY / 100.0;
        else if (axes[AXIS] < -deadzone) target = -MAX_DUTY / 100.0;

        for (int i = 0; i < 4; i++) {
            // Ramp toward the target; reversals pass slowly through zero
            double diff = target - g_speed[i];
            if (diff >  RAMP_PER_TICK) diff =  RAMP_PER_TICK;
            if (diff < -RAMP_PER_TICK) diff = -RAMP_PER_TICK;
            g_speed[i] += diff;

            setMotor(h, i, g_speed[i]);
            setServo(h, i, std::fabs(g_speed[i]) >= 0.05);
        }

        if (readErr != EAGAIN) {
            std::fprintf(stderr, "joystick read failed: errno=%d (%s)\n",
                         readErr, std::strerror(readErr));
            running = false;
        }
        usleep(TICK_US);
    }

    stopAll(h);
    usleep(200000);
    for (int i = 0; i < 4; i++) {
        lgGpioWrite(h, SERVO_PINS[i], 0);
        lgGpioWrite(h, MOTORS[i].in1, 0);
        lgGpioWrite(h, MOTORS[i].in2, 0);
    }
    close(js);
    lgGpiochipClose(h);
    return 0;
}