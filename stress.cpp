// stress_test.cpp
// Stress test for 4 servos + 4 DC motors (2-pin H-bridge drivers, no enable pin) using lgpio.
//
// Each motor uses two pins, IN1 and IN2:
//   forward : PWM on IN1, IN2 low
//   reverse : PWM on IN2, IN1 low
//   coast   : both low
//
// Build:  g++ stress_test.cpp -o stress_test -llgpio -lm
// Run:    ./stress_test            (Ctrl+C stops everything safely)
//
// Edit the PIN SETUP section below with your BCM GPIO numbers.

#include <lgpio.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <thread>

// ============================================================
//  PIN SETUP  (BCM GPIO numbers) - EDIT THESE
// ============================================================

// Pi 4 and earlier: 0.  Pi 5: 4.
static const int GPIO_CHIP = 0;

// Servo signal pins
static const int SERVO_PINS[4] = {
    22,  // servo 0
    23,  // servo 1
    27,  // servo 2
    17   // servo 3
};

// Each motor takes two pins into its H-bridge driver
struct Motor {
    int in1;
    int in2;
};

static const Motor MOTORS[4] = {
    //  IN1  IN2
    {   5,   6 },  // motor 0
    {   20,   21 },  // motor 1
    {  26,  16 },  // motor 2
    {  13,  19 }   // motor 3
};

// ============================================================
//  TEST SETTINGS
// ============================================================

// Servo limits (degrees). Kept to roughly 45-135 as requested.
static const double SERVO_MIN_DEG = 45.0;
static const double SERVO_MAX_DEG = 135.0;

// Pulse width range for 0-180 degrees (microseconds).
// Most hobby servos use 500-2500. 45 deg -> 1000us, 135 deg -> 2000us.
static const int PULSE_AT_0_DEG   = 500;
static const int PULSE_AT_180_DEG = 2500;

static const int    SERVO_HZ  = 50;
static const int    MOTOR_HZ  = 1000;   // PWM frequency on the motor input pins
static const double MAX_DUTY  = 100.0;  // cap motor duty cycle (0-100). Lower this to be gentle.

static const double TEST_SECONDS  = 120.0;  // total test time (0 = run until Ctrl+C)
static const double PHASE_SECONDS = 15.0;   // how long each speed phase lasts
static const int    TICK_MS       = 20;     // update rate

// Motion period (seconds for one full back-and-forth) for each phase.
// Shorter = faster = harder on the hardware. Phases repeat in order.
static const double SERVO_PERIODS[] = {4.0, 2.0, 1.0, 0.6};
static const double MOTOR_PERIODS[] = {6.0, 4.0, 2.5, 1.5};
static const int    NUM_PHASES = sizeof(SERVO_PERIODS) / sizeof(SERVO_PERIODS[0]);

// ============================================================

static volatile sig_atomic_t g_running = 1;
static void onSignal(int) { g_running = 0; }

static int angleToPulse(double deg) {
    deg = std::clamp(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);  // hard safety clamp
    double pw = PULSE_AT_0_DEG + (deg / 180.0) * (PULSE_AT_180_DEG - PULSE_AT_0_DEG);
    return (int)std::lround(pw);
}

static void setServo(int h, int pin, double deg) {
    lgTxServo(h, pin, angleToPulse(deg), SERVO_HZ, 0, 0);
}

// Last direction per motor: 0 = coast, 1 = forward, -1 = reverse
static int g_motorDir[4] = {0, 0, 0, 0};

static void setMotor(int h, int idx, double speed /* -1.0 .. 1.0 */) {
    const Motor& m = MOTORS[idx];
    const double deadband = 0.05;

    int dir = 0;
    if (speed >= deadband)       dir = 1;
    else if (speed <= -deadband) dir = -1;

    // Direction changed: shut both pins off before driving the other one
    if (dir != g_motorDir[idx]) {
        lgTxPwm(h, m.in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, m.in2, MOTOR_HZ, 0.0, 0, 0);
        lgGpioWrite(h, m.in1, 0);
        lgGpioWrite(h, m.in2, 0);
        g_motorDir[idx] = dir;
    }

    if (dir == 0) return;  // coasting, both low

    double duty = std::min(std::fabs(speed) * 100.0, MAX_DUTY);
    int activePin = (dir > 0) ? m.in1 : m.in2;
    lgTxPwm(h, activePin, MOTOR_HZ, duty, 0, 0);
}

static void stopAll(int h) {
    for (int i = 0; i < 4; i++) {
        lgTxServo(h, SERVO_PINS[i], 0, SERVO_HZ, 0, 0);  // 0 = stop sending pulses
        lgTxPwm(h, MOTORS[i].in1, MOTOR_HZ, 0.0, 0, 0);
        lgTxPwm(h, MOTORS[i].in2, MOTOR_HZ, 0.0, 0, 0);
        g_motorDir[i] = 0;
    }
}

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
        int pins[3] = {SERVO_PINS[i], MOTORS[i].in1, MOTORS[i].in2};
        for (int p : pins) {
            int rc = lgGpioClaimOutput(h, 0, p, 0);
            if (rc < 0) {
                std::fprintf(stderr, "Could not claim GPIO %d: %s\n", p, lguErrorText(rc));
                lgGpiochipClose(h);
                return 1;
            }
        }
    }

    std::printf("Stress test running. Ctrl+C to stop.\n");
    std::printf("Servos limited to %.0f-%.0f degrees.\n", SERVO_MIN_DEG, SERVO_MAX_DEG);

    // Start servos at center
    for (int i = 0; i < 4; i++) setServo(h, SERVO_PINS[i], 90.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    int lastPhase = -1;
    const double TWO_PI = 2.0 * M_PI;

    // Accumulated phase (radians) so changing the period doesn't cause jumps
    double servoPhase = 0.0, motorPhase = 0.0;
    double lastT = 0.0;

    while (g_running) {
        double t = std::chrono::duration<double>(clock::now() - start).count();
        if (TEST_SECONDS > 0 && t >= TEST_SECONDS) break;

        int phase = (int)(t / PHASE_SECONDS) % NUM_PHASES;
        if (phase != lastPhase) {
            std::printf("[%6.1fs] Phase %d: servo period %.1fs, motor period %.1fs\n",
                        t, phase + 1, SERVO_PERIODS[phase], MOTOR_PERIODS[phase]);
            lastPhase = phase;
        }

        double dt = t - lastT;
        lastT = t;
        servoPhase += TWO_PI * dt / SERVO_PERIODS[phase];
        motorPhase += TWO_PI * dt / MOTOR_PERIODS[phase];

        // Servos: sweep between min and max, each offset by a quarter cycle
        for (int i = 0; i < 4; i++) {
            double s = 0.5 * (1.0 + std::sin(servoPhase + i * (M_PI / 2.0)));  // 0..1
            double deg = SERVO_MIN_DEG + s * (SERVO_MAX_DEG - SERVO_MIN_DEG);
            setServo(h, SERVO_PINS[i], deg);
        }

        // Motors: sine speed -1..1 (ramps up, ramps down through zero, reverses).
        // Direction only flips near zero speed, which is easy on the H-bridge.
        // Motors are offset from each other so they don't all pull peak current at once.
        for (int i = 0; i < 4; i++) {
            double speed = std::sin(motorPhase + i * (M_PI / 2.0));
            setMotor(h, i, speed);
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
    std::printf("Done.\n");
    return 0;
}