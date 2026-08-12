#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <math.h>


#include "constants.hpp"
#include "pointing.hpp"

#include "../sensor/BMM350_SensorAPI/bmm350.h"
#include "../sensor/BMI3XY_SensorAPI/bmi323.h"
#include "../sensor/BMM350_SensorAPI/examples/common/common.h"
#undef _COMMON_H
#include "../sensor/BMI3XY_SensorAPI/bmi323_examples/common/common.h"

void init(bmm350_dev &mag, bmi3_dev &imu, Motor &azimuth_motor, Motor &elevation_motor);

float stepsToDegrees(int steps);
int degreesToSteps(float degrees);

void setAngleSetpoint(Motor &motor, float degrees);
void calibrateAzimuth(Motor& azimuth_motor, MoveData data, float mag_declination_east_degrees);
void calibrateElevation(Motor& elevation_motor, MoveData data);

int main()
{
    // bmi3_sensor_data imu_data[2] = { 0 };
    // imu_data[0].type = BMI323_ACCEL;
    // imu_data[1].type = BMI323_GYRO;

    // bmm350_mag_temp_data mag_data[3] = { 0 };

    // bmm350_dev mag = { 0 };
    // bmi3_dev imu = { 0 };

    // MoveData data;
    //To get data sensor data to sensor_data: bmi323_get_sensor_data(sensor_data, 3, &dev);

    Motor azimuth_motor(constants::MICROSTEPS_PER_REV, constants::AZIMUTH_STEP_PIN, constants::AZIMUTH_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);
    Motor elevation_motor(constants::MICROSTEPS_PER_REV, constants::ELEVATION_STEP_PIN, constants::ELEVATION_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);

    //init(mag, imu, azimuth_motor, elevation_motor);

    azimuth_motor.setPID(0.5, 0.0, 0.5, constants::INIT_PWM_DELAY);
    azimuth_motor.setSetpointType(Motor::SetpointType::kSTEP);
    azimuth_motor.usePID(true);

    elevation_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
    elevation_motor.setSetpointType(Motor::SetpointType::kSTEP);
    elevation_motor.usePID(true);

    
    azimuth_motor.setStepSetpoint(400);

    while (!azimuth_motor.atSetpoint())
    {
        azimuth_motor.drive();
        std::cout << "Steps: " << azimuth_motor.getSteps() << "\n\r";
    }

//    SocketListener listener(constants::SOCKET_PATH);

//    listener.attemptConnection();

    //data.readData(listener.fetchData());
    //calibrateAzimuth(azimuth_motor, data);
    //calibrateElevation(elevation_motor, data);

    //azimuth_motor.setStepSetpoint(2000);
    //elevation_motor.setStepSetpoint(2000);

    //setAngleSetpoint(azimuth_motor, 45);

    // while (!azimuth_motor.atSetpoint())
    // {

    //     //elevation_motor.drive();
    //     azimuth_motor.drive();
    // }
}