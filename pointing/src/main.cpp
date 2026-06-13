#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "Motor.hpp"
#include "constants.hpp"
#include "pointing.hpp"

struct MoveData
{
    float azimuth, elevation;

    void readData(std::string input_data);
};

int main()
{
    MoveData data;

    Motor motor(constants::MICROSTEPS_PER_REV, constants::STEP_PIN, constants::DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);

    SocketListener listener(constants::SOCKET_PATH);

    listener.attemptConnection();

    while (true)
    {
        data.readData(listener.fetchData());

        moveToAngle(motor, data.azimuth);
    }
}

void MoveData::readData(std::string input_data)
{
    std::stringstream data(input_data);

    data >> this->azimuth >> this->elevation;

}

void moveToAngle(Motor &motor, float degrees)
{
    if (motor.getSetpointType() != Motor::SetpointType::kSTEP)
    {
        motor.setSetpointType(Motor::SetpointType::kSTEP);
    }
    
    motor.setStepSetpoint( ((574*int(degrees * 128)) - motor.getSteps()) / 575);
}