#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <math.h>


#include "pointing.hpp"

int main()
{
    bmi3_sensor_data imu_data[2] = { 0 };
    imu_data[0].type = BMI323_ACCEL;
    imu_data[1].type = BMI323_GYRO;

    bmm350_mag_temp_data mag_data[3] = { 0 };

    bmm350_dev mag = { 0 };
    bmi3_dev imu = { 0 };

    MoveData data;
    //To get data sensor data to sensor_data: bmi323_get_sensor_data(sensor_data, 3, &dev);

    Motor azimuth_motor(constants::MICROSTEPS_PER_REV, constants::AZIMUTH_STEP_PIN, constants::AZIMUTH_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);
    Motor elevation_motor(constants::MICROSTEPS_PER_REV, constants::ELEVATION_STEP_PIN, constants::ELEVATION_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);

    init(mag, imu, azimuth_motor, elevation_motor);

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