#include "pointing.hpp"

SocketListener::SocketListener(std::string socket_path) : sock_path(socket_path.c_str()) {
        
    unlink(this->sock_path);

    this->server_endpoint = socket(AF_UNIX, SOCK_STREAM, 0);

    if (this->server_endpoint == -1) {
        // TODO: Output the path of the UNIX socket as well
        std::runtime_error("Failed to get an endpoint for the UNIX socket");
    }

    unlink(this->sock_path);
    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, this->sock_path, sizeof(addr.sun_path) - 1);

    if (bind(this->server_endpoint, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
           close(this->server_endpoint);
        std::runtime_error("Failed to bind the socket to the endpoint");
    }

    if (listen(this->server_endpoint, 5) == -1) {
        std::cerr << "Listen failed\n";
        close(this->server_endpoint);
        std::runtime_error("Failed to listen to the socket. Try running with sudo.");
    }
};

SocketListener::~SocketListener() {
    close(this->server_endpoint);
    close(this->client_endpoint);
    unlink(this->sock_path);
}

void SocketListener::attemptConnection() {
    this->client_endpoint = accept(this->server_endpoint, nullptr, nullptr);
    if (this->client_endpoint == -1) {
        std::runtime_error("Failed to accept socket connection");
    } else {
        this->peerConnected = true;
    }
}

std::string SocketListener::fetchData() {
    std::memset(data, 0, sizeof(data));
    //char old_data[256];
    //memcpy(old_data, this->data,256);
    ssize_t bytes_received = recv(this->client_endpoint, data, sizeof(data) - 1, 0);
    if (!this->peerConnected) {
        this->attemptConnection();
    } 

    if (bytes_received == 0) {
        //memcpy(this->data, old_data, 256);
        std::cout << "Peer disconnected from the socket\n";
        this->peerConnected = false;
        return "";
    } else if (bytes_received < 0){
        std::cout << "Failed the read from socket"<< std::strerror(errno) << errno<<"\n";
        return "";
    } else {
        //std::cout << this->data << "\n";   
        return this->data;
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

void init(bmm350_dev &mag, bmi3_dev &imu, Motor &azimuth_motor, Motor &elevation_motor)
{
    int8_t rslt;

    // IMU (BMI323) INIT /////////////////////////////
    // Taken from examples/accel_gyro_temp/accel_gyro_temp.c

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
    // Taken from examples/bmm350_polling/bmm350_polling.c

    uint8_t int_ctrl = 0;

    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

    /* Update device structure */
    rslt = bmm350_interface_init(&mag);
    bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

    /* Initialize BMM350 */
    rslt = bmm350_init(&mag);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    /* Check PMU busy */
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &mag);
    bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

    /* Configure interrupt settings */
    rslt = bmm350_configure_interrupt(BMM350_PULSED,
                                      BMM350_ACTIVE_HIGH,
                                      BMM350_INTR_PUSH_PULL,
                                      BMM350_UNMAP_FROM_PIN,
                                      &mag);
    bmm350_error_codes_print_result("bmm350_configure_interrupt", rslt);

    /* Enable data ready interrupt */
    rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &mag);
    bmm350_error_codes_print_result("bmm350_enable_interrupt", rslt);

    /* Get interrupt settings */
    rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, &mag);
    bmm350_error_codes_print_result("bmm350_get_regs", rslt);

    /* Set ODR and performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, &mag);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

    /* Enable all axis */
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &mag);
    bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

    if (rslt == BMM350_OK)
    {
        rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &mag);
        bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
    }

    // MOTOR INIT ////////////////////

    azimuth_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
    azimuth_motor.setSetpointType(Motor::SetpointType::kSTEP);
    azimuth_motor.usePID(true);

    elevation_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
    elevation_motor.setSetpointType(Motor::SetpointType::kSTEP);
    elevation_motor.usePID(true);
}

void readMagData(bmm350_dev &mag, bmm350_mag_temp_data &mag_data)
{
    int8_t rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &mag);
    bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data_fixed", rslt);
}

void readIMUData(bmi3_dev &imu, bmi3_sensor_data &imu_data)
{
    int8_t rslt = bmi323_get_sensor_data(&imu_data, 3, &imu);
    bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);
}