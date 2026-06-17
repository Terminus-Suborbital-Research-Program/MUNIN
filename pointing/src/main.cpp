#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <math.h>

#include "Motor.hpp"
#include "constants.hpp"
#include "pointing.hpp"
#include "BMM350/bmm350.h"
#include "BMM350/examples/common/common.h"
#include "BMI323/bmi323.h"
#include "BMI323/bmi323_examples/common/common.h"

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
    bmi3_dev imu;

    MoveData data;
    //To get data sensor data to sensor_data: bmi323_get_sensor_data(sensor_data, 3, &dev);

    Motor azimuth_motor(constants::MICROSTEPS_PER_REV, constants::AZIMUTH_STEP_PIN, constants::AZIMUTH_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);
    Motor elevation_motor(constants::MICROSTEPS_PER_REV, constants::ELEVATION_STEP_PIN, constants::ELEVATION_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);

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
    // IMU INIT /////////////////////////////



    // MAGNETOMETER INIT //////////////////// 

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print accel data. */
    uint16_t limit = 100;

    /* Initialize the interrupt status of accel. */
    uint16_t int_status = 0;

    /* Variable to store temperature */
    float temperature_value;

    uint8_t indx = 0;
    float acc_x = 0, acc_y = 0, acc_z = 0;
    float gyr_x = 0, gyr_y = 0, gyr_z = 0;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_I2C_INTF);

    /* Initialize bmi323. */
    rslt = bmi323_init(&dev);

     /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config[2];

    /* Structure to map interrupt */
    struct bmi3_map_int map_int = { 0 };

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, &dev);

    if (rslt == BMI323_OK)
    {
        map_int.acc_drdy_int = BMI3_INT1;
        map_int.gyr_drdy_int = BMI3_INT1;
        map_int.temp_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, &dev);

        if (rslt == BMI323_OK)
        {
            /* NOTE: The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for accel. */
            config[0].cfg.acc.odr = BMI3_ACC_ODR_50HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config[0].cfg.acc.range = BMI3_ACC_RANGE_2G;

            /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
            config[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

            /* Set number of average samples for accel. */
            config[0].cfg.acc.avg_num = BMI3_ACC_AVG64;

            /* Enable the accel mode where averaging of samples
             * will be done based on above set bandwidth and ODR.
             * Note : By default accel is disabled. The accel will get enable by selecting the mode.
             */
            config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_50HZ;

            /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
            config[1].cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;

            /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
             *  Value   Name      Description
             *    0   odr_half   BW = gyr_odr/2
             *    1  odr_quarter BW = gyr_odr/4
             */
            config[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

            /* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
            config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

            /* Value    Name    Description
             *  0b000     avg_1   No averaging; pass sample without filtering
             *  0b001     avg_2   Averaging of 2 samples
             *  0b010     avg_4   Averaging of 4 samples
             *  0b011     avg_8   Averaging of 8 samples
             *  0b100     avg_16  Averaging of 16 samples
             *  0b101     avg_32  Averaging of 32 samples
             *  0b110     avg_64  Averaging of 64 samples
             */
            config[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;

            /* Set the accel and gyro configurations. */
            rslt = bmi323_set_sensor_config(config, 2, &dev);
        }
    }

    // MOTOR INIT ////////////////////

    azimuth_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
    azimuth_motor.setSetpointType(Motor::SetpointType::kSTEP);
    azimuth_motor.usePID(true);

    elevation_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
    elevation_motor.setSetpointType(Motor::SetpointType::kSTEP);
    elevation_motor.usePID(true);
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

// void calibrateAzimuth(Motor& azimuth_motor, MoveData data, float mag_declination_east_degrees)
// {
//     float mag_north_deg_2d = 180.0 * atan2(data.magnetometer_y, data.magnetometer_x) / 3.14159265359;
//     float true_north = mag_north_deg_2d + mag_declination_east_degrees;

//     Motor::SetpointType og_type = azimuth_motor.getSetpointType();

//     azimuth_motor.setSetpointType(Motor::SetpointType::kSTEP);
//     azimuth_motor.setStepSetpoint(degreesToSteps(true_north));
    
//     while (!azimuth_motor.atSetpoint())
//     {
//         azimuth_motor.drive();
//     }

//     azimuth_motor.setSteps();    
//     azimuth_motor.setSetpointType(og_type);

// }

// void calibrateElevation(Motor& elevation_motor, MoveData data)
// {
//     Motor::SetpointType og_type = elevation_motor.getSetpointType();

//     elevation_motor.setSetpointType(Motor::SetpointType::kSTEP);
//     elevation_motor.setStepSetpoint(degreesToSteps(180.0 * data.gyro_y / 3.14159265359));
    
//     while (!elevation_motor.atSetpoint())
//     {
//         elevation_motor.drive();
//     }

//     elevation_motor.setSteps();    
//     elevation_motor.setSetpointType(og_type);

// }
