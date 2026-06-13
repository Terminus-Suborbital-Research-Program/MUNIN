#include <filesystem>
#include <gpiod.hpp>
#include <chrono>

namespace constants
{
    const std::filesystem::path GPIO_CONTROLLER_PATH = "/dev/gpiochip0";

    const char* SOCKET_PATH = "/tmp/munin.sock";

    const unsigned int MICROSTEPS_PER_REV = 200;
    const unsigned int STEP_PIN = 0;
    const unsigned int DIR_PIN = 1;
    const std::chrono::microseconds INIT_PWM_DELAY = std::chrono::microseconds(50);
}