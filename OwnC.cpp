#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>
#include <lgpio.h>
#include <cstdio>
#include <vector>
#include <csignal>
#include <cerrno>
#include <cstring>

// ============================================================
//  SETTINGS
// ============================================================
static const int    GPIO_CHIP = 0;        // Pi 4 and earlier: 0, Pi 5: 4
static const int    deadzone  = 8000;     // axis range is -32767..32767
static const int    AXIS      = 0;        // which joystick axis drives the robot
static const double DUTY      = 50.0;     // motor duty cycle, 0-100
static const int    MOTOR_HZ  = 1000;     // PWM frequency on motor pins
static const int    SERVO_HZ  = 50;
static const int    SERVO_US  = 1500;     // servo pulse width while moving
static const int    TICK_US   = 20000;    // 20 ms loop, same as the stress test

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

static int g_motorDir[4] = { 0, 0, 0, 0 };   // last direction applied per motor

// ============================================================
//  OUTPUT HELPERS
// ============================================================
// dir: 1 = forward (PWM on in1), -1 = reverse (PWM on in2), 0 = stop
static void setMotor(int h, int idx, int dir, double duty) {
    const Motor& m = MOTORS[idx];

    // Direction changed: shut both pins off before driving the other one
    if (dir != g_motorDir[idx]) {
        lgTxPwm(h, m.in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, m.in2, MOTOR_HZ, 0.0, 0, 0);
        lgGpioWrite(h, m.in1, 0);
        lgGpioWrite(h, m.in2, 0);
        g_motorDir[idx] = dir;
    }

    if (dir == 0) return;   // stopped, both pins low

    lgTxPwm(h, (dir > 0) ? m.in1 : m.in2, MOTOR_HZ, duty, 0, 0);
}

static void setServo(int h, int idx, int dir) {
    // pulse width 0 = stop sending pulses
    lgTxServo(h, SERVO_PINS[idx], dir == 0 ? 0 : SERVO_US, SERVO_HZ, 0, 0);
}

static void stopAll(int h) {
    for (int i = 0; i < 4; i++) {
        setServo(h, i, 0);
        setMotor(h, i, 0, 0.0);
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

    // Claim all pins as outputs, starting low
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
    int  lastdir = 0;   // direction actually applied on the previous tick

    while (running && !stop) {

        // ---- read all pending joystick events ----
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
        int readErr = errno;   // save immediately, before any other calls

        // ---- decide direction ----
        int want = 0;
        if (axes[AXIS] > deadzone)       want = 1;
        else if (axes[AXIS] < -deadzone) want = -1;

        // Never reverse directly: pass through stop for one tick
        int dir = want;
        if (lastdir != 0 && want != 0 && want != lastdir) dir = 0;
        lastdir = dir;

        // ---- apply every tick (same pattern as the stress test) ----
        for (int i = 0; i < 4; i++) {
            setServo(h, i, dir);
            setMotor(h, i, dir, DUTY);
        }

        // ---- exit only if the joystick really failed ----
        if (readErr != EAGAIN) {
            std::fprintf(stderr, "joystick read failed: errno=%d (%s)\n",
                         readErr, std::strerror(readErr));
            running = false;
        }

        usleep(TICK_US);
    }

    // ---- safe shutdown ----
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