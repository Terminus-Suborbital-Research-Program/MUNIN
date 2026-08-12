#pragma once

#include <filesystem>
#include <gpiod.hpp>
#include <chrono>

namespace constants
{
    inline const char* SOCKET_PATH = "/tmp/munin.sock";

    const std::filesystem::path GPIO_CONTROLLER_PATH = "/dev/gpiochip0";

    const unsigned int MICROSTEPS_PER_REV = 200;

    const unsigned int ELEVATION_STEP_PIN = 13;
    const unsigned int ELEVATION_DIR_PIN = 16; 

    const unsigned int AZIMUTH_STEP_PIN = 11;
    const unsigned int AZIMUTH_DIR_PIN = 15; 

    const std::chrono::microseconds INIT_PWM_DELAY = std::chrono::microseconds(1000);
}
