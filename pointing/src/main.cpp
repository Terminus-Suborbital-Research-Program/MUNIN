#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>

#include "Motor.hpp"
#include "constants.hpp"

struct MoveData
{
    int azimuth, elevation;

    void readData(std::string input_data);
};

int main()
{

    Motor motor(constants::MICROSTEPS_PER_REV, constants::STEP_PIN, constants::DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);
    
    while (motor.getSteps() != 2000 || !motor.atSetpoint())
    {
        std::cout << motor.getSteps() << "\n\r";

        motor.drive();
    }

    motor.~Motor();
}

void MoveData::readData(std::string input_data)
{
    std::stringstream data(input_data);

    data >> this->azimuth >> this->elevation;

}

void moveToAngle(Motor motor, float degrees)
{
    if (motor.getSetpointType() != Motor::SetpointType::kSTEP)
    {
        motor.setSetpointType(Motor::SetpointType::kSTEP);
    }
    
    motor.setStepSetpoint( ((575*int(degrees * 128)) - motor.getSteps()) / 575);
}

std::string readSocket(int socket_file_descriptor)
{
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
}