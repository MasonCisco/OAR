// xbox_control.cpp
// Control 4 servos + 4 DC motors (2-pin H-bridge drivers) with an Xbox controller on Linux.
// Uses the Linux joystick API (/dev/input/js0) and lgpio.
//
// Build:  g++ xbox_control.cpp -o xbox_control -llgpio -lm
// Run:    ./xbox_control                  (uses /dev/input/js0)
//         ./xbox_control /dev/input/js1   (different device)
//         ./xbox_control --debug          (prints every axis/button number as you press it)
//
// CONTROLS
//   Left stick  Y      : motors 0 and 1  (up = forward, down = reverse)
//   Right stick Y      : motors 2 and 3
//   Left stick  X      : servo 0
//   Right stick X      : servo 1
//   LT / RT            : servo 2  (RT = one way, LT = the other, released = center)
//   D-pad left/right   : servo 3  (hold to move, stays where you leave it)
//   A                  : re-center servo 3
//   B                  : toggle motor kill switch (motors stop / resume)
//   Start              : quit
//
// Servos are limited to 45-135 degrees.
// If your axis/button numbers differ (Bluetooth, xpadneo, etc.), run with --debug
// and edit the AXIS_ and BTN_ constants below.

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

// ============================================================
//  PIN SETUP  (BCM GPIO numbers)
// ============================================================

// Pi 4 and earlier: 0.  Pi 5: 4.
static const int GPIO_CHIP = 0;

static const int SERVO_PINS[4] = {
    22,  // servo 0
    23,  // servo 1
    27,  // servo 2
    17   // servo 3
};

struct Motor {
    int in1;
    int in2;
};

static const Motor MOTORS[4] = {
    //  IN1  IN2
    {   5,   6 },  // motor 0
    {  20,  21 },  // motor 1
    {  26,  16 },  // motor 2
    {  13,  19 }   // motor 3
};

// Flip a motor's direction in software if it spins the wrong way
// (common for the motors on one side of a tank-style robot).
static const bool MOTOR_INVERT[4] = {false, false, false, false};

// ============================================================
//  CONTROLLER MAPPING  (typical xpad / wired Xbox layout)
// ============================================================

static const int AXIS_LX   = 0;
static const int AXIS_LY   = 1;
static const int AXIS_LT   = 2;
static const int AXIS_RX   = 3;
static const int AXIS_RY   = 4;
static const int AXIS_RT   = 5;
static const int AXIS_DPADX = 6;

static const int BTN_A     = 0;
static const int BTN_B     = 1;
static const int BTN_START = 7;

// ============================================================
//  SETTINGS
// ============================================================

static const double SERVO_MIN_DEG = 45.0;
static const double SERVO_MAX_DEG = 135.0;
static const double SERVO_CENTER  = 90.0;

static const int PULSE_AT_0_DEG   = 500;    // microseconds at 0 degrees
static const int PULSE_AT_180_DEG = 2500;   // microseconds at 180 degrees

static const int    SERVO_HZ = 50;
static const int    MOTOR_HZ = 1000;
static const double MAX_DUTY = 100.0;       // motor duty cap (0-100)

static const double STICK_DEADZONE = 0.12;  // ignore small stick drift
static const double DPAD_DEG_PER_SEC = 90.0;
static const double MOTOR_RAMP_PER_TICK = 0.05;  // max speed change per tick (smooths starts/reversals)
static const int    TICK_MS = 10;

// ============================================================

static volatile sig_atomic_t g_running = 1;
static void onSignal(int) { g_running = 0; }

static double norm(int v) {
    return std::clamp(v / 32767.0, -1.0, 1.0);
}

static double applyDeadzone(double v, double dz) {
    if (std::fabs(v) < dz) return 0.0;
    return (v - std::copysign(dz, v)) / (1.0 - dz);
}

static int angleToPulse(double deg) {
    deg = std::clamp(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);  // hard safety clamp
    double pw = PULSE_AT_0_DEG + (deg / 180.0) * (PULSE_AT_180_DEG - PULSE_AT_0_DEG);
    return (int)std::lround(pw);
}

// ---------- motors ----------

static int g_motorDir[4] = {0, 0, 0, 0};  // 0 = coast, 1 = forward, -1 = reverse

static void setMotor(int h, int idx, double speed /* -1..1 */) {
    const Motor& m = MOTORS[idx];
    if (MOTOR_INVERT[idx]) speed = -speed;

    const double deadband = 0.03;
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

static void stopAll(int h) {
    for (int i = 0; i < 4; i++) {
        lgTxServo(h, SERVO_PINS[i], 0, SERVO_HZ, 0, 0);  // stop servo pulses
        lgTxPwm(h, MOTORS[i].in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, MOTORS[i].in2, MOTOR_HZ, 0.0, 0, 0);
        g_motorDir[i] = 0;
    }
}

int main(int argc, char** argv) {
    const char* devPath = "/dev/input/js0";
    bool debug = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--debug") == 0) debug = true;
        else devPath = argv[i];
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // ---- open controller ----
    int fd = open(devPath, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::fprintf(stderr, "Could not open %s: %s\n", devPath, std::strerror(errno));
        std::fprintf(stderr, "Check it exists with:  ls /dev/input/js*\n");
        return 1;
    }
    char name[128] = "Unknown";
    ioctl(fd, JSIOCGNAME(sizeof(name)), name);
    std::printf("Controller: %s (%s)\n", name, devPath);

    // ---- open GPIO ----
    int h = lgGpiochipOpen(GPIO_CHIP);
    if (h < 0) {
        std::fprintf(stderr, "lgGpiochipOpen(%d) failed: %s\n", GPIO_CHIP, lguErrorText(h));
        close(fd);
        return 1;
    }
    for (int i = 0; i < 4; i++) {
        int pins[3] = {SERVO_PINS[i], MOTORS[i].in1, MOTORS[i].in2};
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

    // ---- state ----
    double axes[16] = {0};
    double servoAngle[4] = {SERVO_CENTER, SERVO_CENTER, SERVO_CENTER, SERVO_CENTER};
    int    lastPulse[4]  = {-1, -1, -1, -1};
    double motorSpeed[4] = {0, 0, 0, 0};   // current (ramped) speed
    bool   motorsEnabled = true;
    const double dt = TICK_MS / 1000.0;

    std::printf("Running. Start = quit, B = motor kill switch. Ctrl+C also stops safely.\n");

    while (g_running) {
        // ---- read all pending controller events ----
        js_event e;
        ssize_t n;
        while ((n = read(fd, &e, sizeof(e))) == (ssize_t)sizeof(e)) {
            bool isInit = e.type & JS_EVENT_INIT;
            e.type &= ~JS_EVENT_INIT;

            if (e.type == JS_EVENT_AXIS && e.number < 16) {
                axes[e.number] = norm(e.value);
                if (debug && !isInit) std::printf("axis %d = %d\n", e.number, e.value);
            } else if (e.type == JS_EVENT_BUTTON) {
                if (debug && !isInit) std::printf("button %d = %d\n", e.number, e.value);
                if (e.value == 1 && !isInit) {
                    if (e.number == BTN_A) servoAngle[3] = SERVO_CENTER;
                    if (e.number == BTN_B) {
                        motorsEnabled = !motorsEnabled;
                        std::printf("Motors %s\n", motorsEnabled ? "ENABLED" : "KILLED");
                    }
                    if (e.number == BTN_START) g_running = 0;
                }
            }
        }
        if (n < 0 && errno != EAGAIN) {
            std::fprintf(stderr, "\nController disconnected (%s). Stopping.\n", std::strerror(errno));
            break;
        }

        // ---- servos ----
        double lx = applyDeadzone(axes[AXIS_LX], STICK_DEADZONE);
        double rx = applyDeadzone(axes[AXIS_RX], STICK_DEADZONE);

        // Triggers rest at -1 and go to +1 when fully pressed -> convert to 0..1
        double lt = (axes[AXIS_LT] + 1.0) / 2.0;
        double rt = (axes[AXIS_RT] + 1.0) / 2.0;
        double trig = rt - lt;  // -1..1

        double half = (SERVO_MAX_DEG - SERVO_MIN_DEG) / 2.0;
        servoAngle[0] = SERVO_CENTER + lx * half;
        servoAngle[1] = SERVO_CENTER + rx * half;
        servoAngle[2] = SERVO_CENTER + trig * half;
        servoAngle[3] += axes[AXIS_DPADX] * DPAD_DEG_PER_SEC * dt;
        servoAngle[3] = std::clamp(servoAngle[3], SERVO_MIN_DEG, SERVO_MAX_DEG);

        for (int i = 0; i < 4; i++) {
            int pw = angleToPulse(servoAngle[i]);
            if (pw != lastPulse[i]) {
                lgTxServo(h, SERVO_PINS[i], pw, SERVO_HZ, 0, 0);
                lastPulse[i] = pw;
            }
        }

        // ---- motors (stick up is negative, so invert) ----
        double left  = -applyDeadzone(axes[AXIS_LY], STICK_DEADZONE);
        double right = -applyDeadzone(axes[AXIS_RY], STICK_DEADZONE);
        double target[4] = {left, left, right, right};

        for (int i = 0; i < 4; i++) {
            double t = motorsEnabled ? target[i] : 0.0;
            double diff = t - motorSpeed[i];
            diff = std::clamp(diff, -MOTOR_RAMP_PER_TICK, MOTOR_RAMP_PER_TICK);
            motorSpeed[i] += diff;
            setMotor(h, i, motorSpeed[i]);
        }

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