// joystick_lgpio.cpp
// Reads a Bluetooth Xbox controller (xpadneo driver) through the Linux joystick
// API (/dev/input/jsX), prints every axis/button change, and uses lgpio to
// drive an LED on a GPIO pin while button 0 (A) is held.
//
// Build:  g++ -O2 -o joystick_lgpio joystick_lgpio.cpp -llgpio
// Run:    ./joystick_lgpio [/dev/input/js0] [gpio_pin] [gpiochip]
//
// Install deps:  sudo apt install liblgpio-dev
// Pi 5 note: the header GPIOs are on gpiochip4 (or gpiochip0 on newer kernels);
//            check with `gpioinfo` or `ls /dev/gpiochip*`.

#include <fcntl.h>
#include <linux/joystick.h>
#include <lgpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static volatile sig_atomic_t running = 1;
static void onSignal(int) { running = 0; }

// Typical joydev axis order for xpadneo (verify with `jstest /dev/input/js0`)
static const char *axisName(int a)
{
    switch (a) {
        case 0: return "LeftX";
        case 1: return "LeftY";
        case 2: return "RightX";
        case 3: return "RightY";
        case 4: return "RT";
        case 5: return "LT";
        case 6: return "DpadX";
        case 7: return "DpadY";
        default: return "Axis";
    }
}

int main(int argc, char **argv)
{
    const char *dev  = (argc > 1) ? argv[1] : "/dev/input/js0";
    int ledPin       = (argc > 2) ? std::atoi(argv[2]) : 17;
    int chip         = (argc > 3) ? std::atoi(argv[3]) : 0;

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // ---- Joystick ----
    int js = open(dev, O_RDONLY | O_NONBLOCK);
    if (js < 0) {
        std::perror("open joystick");
        std::fprintf(stderr, "Is the controller paired/connected? Try: ls /dev/input/js*\n");
        return 1;
    }

    char name[128] = "Unknown";
    unsigned char numAxes = 0, numButtons = 0;
    ioctl(js, JSIOCGNAME(sizeof(name)), name);
    ioctl(js, JSIOCGAXES, &numAxes);
    ioctl(js, JSIOCGBUTTONS, &numButtons);
    std::printf("Controller: %s (%d axes, %d buttons)\n", name, numAxes, numButtons);

    std::vector<int> axes(numAxes, 0);
    std::vector<int> buttons(numButtons, 0);

    // ---- lgpio ----
    int h = lgGpiochipOpen(chip);
    if (h < 0) {
        std::fprintf(stderr, "lgGpiochipOpen(%d) failed: %s\n", chip, lguErrorText(h));
        close(js);
        return 1;
    }
    int rc = lgGpioClaimOutput(h, 0, ledPin, 0);
    if (rc < 0) {
        std::fprintf(stderr, "lgGpioClaimOutput(%d) failed: %s\n", ledPin, lguErrorText(rc));
        lgGpiochipClose(h);
        close(js);
        return 1;
    }
    std::printf("LED on GPIO%d (chip %d) follows button 0. Ctrl+C to quit.\n", ledPin, chip);

    // ---- Main loop ----
    struct pollfd pfd = { js, POLLIN, 0 };
    while (running) {
        int p = poll(&pfd, 1, 200);
        if (p < 0) {
            if (errno == EINTR) continue;
            std::perror("poll");
            break;
        }
        if (p == 0) continue;  // timeout, just re-check `running`

        if (pfd.revents & (POLLERR | POLLHUP)) {
            std::fprintf(stderr, "Controller disconnected.\n");
            break;
        }

        struct js_event e;
        while (read(js, &e, sizeof(e)) == (ssize_t)sizeof(e)) {
            bool init = e.type & JS_EVENT_INIT;
            switch (e.type & ~JS_EVENT_INIT) {
                case JS_EVENT_AXIS:
                    if (e.number < axes.size()) axes[e.number] = e.value;
                    if (!init)
                        std::printf("%-7s (axis %u): %6d  (%+.2f)\n",
                                    axisName(e.number), e.number, e.value, e.value / 32767.0);
                    break;

                case JS_EVENT_BUTTON:
                    if (e.number < buttons.size()) buttons[e.number] = e.value;
                    if (!init)
                        std::printf("Button %u: %s\n", e.number, e.value ? "PRESSED" : "released");
                    if (e.number == 0)
                        lgGpioWrite(h, ledPin, e.value ? 1 : 0);
                    break;
            }
        }
    }

    // ---- Cleanup ----
    lgGpioWrite(h, ledPin, 0);
    lgGpioFree(h, ledPin);
    lgGpiochipClose(h);
    close(js);
    std::puts("\nBye.");
    return 0;
}