#include <filesystem>
#include <gpiod.hpp>
#include <chrono>

namespace constants
{
    const std::filesystem::path GPIO_CONTROLLER_PATH = "/dev/gpiochip0";

    const char* SOCKET_PATH = "/tmp/munin.sock";

    const unsigned int MICROSTEPS_PER_REV = 200;

    const unsigned int AZIMUTH_STEP_PIN = 11; //GPIO 17
    const unsigned int AZIMUTH_DIR_PIN = 16; //GPIO 23

    const unsigned int ELEVATION_STEP_PIN = 12; //GPIO 18
    const unsigned int ELEVATION_DIR_PIN = 15; //GPIO 22

    const std::chrono::microseconds INIT_PWM_DELAY = std::chrono::microseconds(50);
}