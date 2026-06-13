#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "Motor.hpp"
#include "constants.hpp"

const char* SOCKET_PATH;

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
    //unlink(SOCKET_PATH);
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
    
    motor.setStepSetpoint( ((574*int(degrees * 128)) - motor.getSteps()) / 575);
}

void setupSocket(const char* socket_path)
{
    unlink(socket_path);

    int sock_fd = socket(AF_UNIX, SOCK_STREAM, 0);

    int sender_fd;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path)-1);

    if (bind(sender_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        std::cout << "\nERROR: FAILED TO BIND TO SOCKET AT PATH " << socket_path << "\n\r";
    }

    if (listen(sender_fd, 5) < 0)
    {
        std::cout << "\nERROR: FAILED TO LISTEN AT SOCKET FD " << sender_fd << "\n\r";
    }
}

std::string readSocket(char buffer[], int sender_fd)
{
    int client_fd = accept(sender_fd, nullptr, nullptr);

    if (client_fd < 0)
    {
        std::cout << "\nERROR: FAILED TO ACCEPT CONNECT FROM CLIENT SOCKET FD" << client_fd << "\n\r";
    }

    if (recv(client_fd, buffer, sizeof(buffer)-1, 0))
    {
        std::cout << "\nERROR: FAILED TO READ DATA FROM CLIENT SOCKET FD" << client_fd << " TO BUFFER\n\r";
    }

    return std::string(buffer);
}