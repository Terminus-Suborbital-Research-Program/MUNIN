#include <iostream>

#include "Motor.hpp"
#include "constants.hpp"

int main()
{
    Motor motor(constants::MICROSTEPS_PER_REV, constants::STEP_PIN, constants::DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);
    motor.~Motor();
}