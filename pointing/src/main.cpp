#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <math.h>

#include "Motor.hpp"
#include "constants.hpp"
#include "pointing.hpp"

struct MoveData
{
    float azimuth, elevation;

    void readData(std::string input_data);
};

float stepsToDegrees(int steps);
int degreesToSteps(float degrees);

void setAngleSetpoint(Motor &motor, float degrees);
void calibrateAzimuth(Motor& azimuth_motor, MoveData data, float mag_declination_east_degrees);
void calibrateElevation(Motor& elevation_motor, MoveData data);

int main()
{
    //MoveData data;

    Motor azimuth_motor(constants::MICROSTEPS_PER_REV, constants::AZIMUTH_STEP_PIN, constants::AZIMUTH_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);
    Motor elevation_motor(constants::MICROSTEPS_PER_REV, constants::ELEVATION_STEP_PIN, constants::ELEVATION_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);

//    SocketListener listener(constants::SOCKET_PATH);

//    listener.attemptConnection();

    //data.readData(listener.fetchData());
    //calibrateAzimuth(azimuth_motor, data);
    //calibrateElevation(elevation_motor, data);

    // while (true)
    // {
    //     //data.readData(listener.fetchData());

    //     //moveToAngle(azimuth_motor, data.azimuth);
    //     //moveToAngle(elevation_motor, data.elevation);
    // }

    azimuth_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
    azimuth_motor.setSetpointType(Motor::SetpointType::kSTEP);
    azimuth_motor.usePID(true);

    elevation_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
    elevation_motor.setSetpointType(Motor::SetpointType::kSTEP);
    elevation_motor.usePID(true);

    //azimuth_motor.setStepSetpoint(2000);
    //elevation_motor.setStepSetpoint(2000);

    setAngleSetpoint(azimuth_motor, 45);
    setAngleSetpoint(elevation_motor, 45);
    while (!azimuth_motor.atSetpoint())
    {

        elevation_motor.drive();
        azimuth_motor.drive();
    }
}

void MoveData::readData(std::string input_data)
{
    std::stringstream data(input_data);

    data >> this->azimuth >> this->elevation;
}

float stepsToDegrees(int steps)
{
    return float(steps) / 128.0;
}

int degreesToSteps(float degrees)
{
    int steps = int(degrees * 128.0);
    steps -= steps/575;

    return steps;
}

void setAngleSetpoint(Motor &motor, float degrees)
{
    if (motor.getSetpointType() != Motor::SetpointType::kSTEP)
    {
        motor.setSetpointType(Motor::SetpointType::kSTEP);
    }

    int steps = degreesToSteps(degrees);

    float current_deg = stepsToDegrees(motor.getSteps());

    float forward_turn_error = 360.0 + degrees - current_deg;
    float backward_turn_error = current_deg - degrees;

    if (forward_turn_error < backward_turn_error)
    {
        int new_steps = degreesToSteps(360.0 + degrees);

        motor.setStepSetpoint(new_steps);
        motor.setSteps(degreesToSteps(current_deg - degrees));
    }
    else
    {
        motor.setStepSetpoint(steps);
    }

    //Error will still accumulate for many, small increments.
    //Backsteps are only calculated for the given degrees.
}

//void calibrateAzimuth(Motor& azimuth_motor, MoveData data, float mag_declination_east_degrees)
//{
//    float mag_north_deg_2d = 180.0 * atan2(data.magnetometer_y, data.magnetometer_x) / 3.14159265359;
//    float true_north = mag_north_deg_2d + mag_declination_east_degrees;

//    Motor::SetpointType og_type = azimuth_motor.getSetpointType();

//    azimuth_motor.setSetpointType(Motor::SetpointType::kSTEP);
//    azimuth_motor.setStepSetpoint(degreesToSteps(true_north));
    
//    while (!azimuth_motor.atSetpoint())
//    {
//        azimuth_motor.drive();
//    }

//    azimuth_motor.setSteps();    
//    azimuth_motor.setSetpointType(og_type);

//}

//void calibrateElevation(Motor& elevation_motor, MoveData data)
//{
//    Motor::SetpointType og_type = elevation_motor.getSetpointType();

//    elevation_motor.setSetpointType(Motor::SetpointType::kSTEP);
//    elevation_motor.setStepSetpoint(degreesToSteps(180.0 * data.gyro_y / 3.14159265359));
    
//    while (!elevation_motor.atSetpoint())
//    {
//        elevation_motor.drive();
//    }

//    elevation_motor.setSteps();    
//    elevation_motor.setSetpointType(og_type);

//}
