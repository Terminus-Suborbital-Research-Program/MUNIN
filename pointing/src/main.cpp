#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <math.h>

#include "Motor.hpp"
#include "constants.hpp"
#include "pointing.hpp"

#include "bmm350.h"
#include "bmi323.h"
#include "sensor/BMM350_SensorAPI/examples/common/common.h"
#undef _COMMON_H
#include "sensor/BMI3XY_SensorAPI/bmi323_examples/common/common.h"

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

int main()
{
    /* Create an instance of sensor data structure. */
    struct bmi3_sensor_data sensor_data[2] = { 0 };

    /* Select accel and gyro sensor. */
    sensor_data[0].type = BMI323_ACCEL;
    sensor_data[1].type = BMI323_GYRO;

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

void init(bmm350_dev &mag, bmi3_dev &imu, Motor &azimuth_motor, Motor &elevation_motor)
{
    int8_t rslt;

    // IMU (BMI323) INIT /////////////////////////////

    rslt = bmi3_interface_init(&imu, BMI3_I2C_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);
    if (rslt != BMI3_OK) return;

    rslt = bmi323_init(&imu);
    bmi3_error_codes_print_result("bmi323_init", rslt);
    if (rslt != BMI323_OK) return;

    struct bmi3_sens_config config[2];
    struct bmi3_map_int map_int = { 0 };

    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_GYRO;

    rslt = bmi323_get_sensor_config(config, 2, &imu);
    bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);
    if (rslt != BMI323_OK) return;

    map_int.acc_drdy_int = BMI3_INT1;
    map_int.gyr_drdy_int = BMI3_INT1;
    map_int.temp_drdy_int = BMI3_INT1;

    rslt = bmi323_map_interrupt(map_int, &imu);
    bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);
    if (rslt != BMI323_OK) return;

    config[0].cfg.acc.odr      = BMI3_ACC_ODR_50HZ;
    config[0].cfg.acc.range    = BMI3_ACC_RANGE_2G;
    config[0].cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER;
    config[0].cfg.acc.avg_num  = BMI3_ACC_AVG64;
    config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    config[1].cfg.gyr.odr      = BMI3_GYR_ODR_50HZ;
    config[1].cfg.gyr.range    = BMI3_GYR_RANGE_2000DPS;
    config[1].cfg.gyr.bwp      = BMI3_GYR_BW_ODR_HALF;
    config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
    config[1].cfg.gyr.avg_num  = BMI3_GYR_AVG1;

    rslt = bmi323_set_sensor_config(config, 2, &imu);
    bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
    if (rslt != BMI323_OK) return;

    // MAGNETOMETER (BMM350) INIT ////////////////////

    rslt = bmm350_interface_init(&mag);
    bmm350_error_codes_print_result("bmm350_interface_init", rslt);
    if (rslt != BMM350_OK) return;

    rslt = bmm350_init(&mag);
    bmm350_error_codes_print_result("bmm350_init", rslt);
    if (rslt != BMM350_OK) return;

    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &mag);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
    if (rslt != BMM350_OK) return;

    // MOTOR INIT ////////////////////

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
