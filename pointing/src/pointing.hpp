#pragma once

#include <iostream>
#include <string>
#include <cstring>
#include <exception>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "Motor.hpp"
#include "constants.hpp"

extern "C"
{
#include "../sensor/BMM350_SensorAPI/bmm350.h"
#include "../sensor/BMI3XY_SensorAPI/bmi323.h"
#include "../sensor/BMM350_SensorAPI/examples/common/common.h"
#undef _COMMON_H
#include "../sensor/BMI3XY_SensorAPI/bmi323_examples/common/common.h"
}

class SocketListener {
public:
    const char* sock_path = "";
    int server_endpoint = -1;
    int client_endpoint = -1;
    char data[256];
    bool peerConnected = false;

    SocketListener() = delete;
    SocketListener(std::string socket_path);

    ~SocketListener();

    /**
     * @brief
     * @note This method is blocking due to the call to `accept`
     */
    void attemptConnection();

    std::string fetchData();
};

struct MoveData
{
    float azimuth, elevation;

    void readData(std::string input_data);
};

void init(bmm350_dev &mag, bmi3_dev &imu, Motor &azimuth_motor, Motor &elevation_motor);

float stepsToDegrees(int steps);
int degreesToSteps(float degrees);

void setAngleSetpoint(Motor &motor, float degrees);
void calibrateAzimuth(Motor& azimuth_motor, MoveData data, float mag_declination_east_degrees);
void calibrateElevation(Motor& elevation_motor, MoveData data);

void readMagData(bmm350_dev &mag, bmm350_mag_temp_data &mag_data);
void readIMUData(bmi3_dev &imu, bmi3_sensor_data &imu_data);