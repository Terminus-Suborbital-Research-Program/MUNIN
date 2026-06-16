#include <filesystem>
#include <gpiod.hpp>
#include <chrono>

namespace constants
{
    const std::filesystem::path GPIO_CONTROLLER_PATH = "/dev/gpiochip0";

    const char* SOCKET_PATH = "/tmp/munin.sock";

    const unsigned int MICROSTEPS_PER_REV = 200;

    const unsigned int ELEVATION_STEP_PIN = 27;
    const unsigned int ELEVATION_DIR_PIN = 23; 

    const unsigned int AZIMUTH_STEP_PIN = 17;
    const unsigned int AZIMUTH_DIR_PIN = 22; 

    const std::chrono::microseconds INIT_PWM_DELAY = std::chrono::microseconds(1000);
}
