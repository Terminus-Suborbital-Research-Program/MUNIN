#include <gpiod.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    try
    {
        // Open GPIO chip
        gpiod::chip chip("/dev/gpiochip0");

        // Configure GPIO18 as an output
        gpiod::line_settings settings;
        settings.set_direction(gpiod::line::direction::OUTPUT);
        settings.set_output_value(gpiod::line::value::INACTIVE);

        gpiod::line_config config;
        config.add_line_settings({gpiod::line::offset(18)}, settings);

        // Request ownership of GPIO18
        auto request = chip.prepare_request()
                           .set_consumer("gpio18-test")
                           .set_line_config(config)
                           .do_request();

        std::cout << "Toggling GPIO18...\n";

        while (true)
        {
            request.set_value(18, gpiod::line::value::ACTIVE);
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << "\n" << request.get_value(gpiod::line::offset(18));

            std::cout << "LOW\n";
            request.set_value(18, gpiod::line::value::INACTIVE);
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << "\n" << request.get_value(gpiod::line::offset(18)) << "\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << '\n';
    }

    return 0;
}

// #include <iostream>
// #include <sstream>
// #include <sys/socket.h>
// #include <sys/un.h>
// #include <unistd.h>

// #include "Motor.hpp"
// #include "constants.hpp"
// #include "pointing.hpp"

// struct MoveData
// {
//     float azimuth, elevation;
//     float accelerometer_x, accelerometer_y, accelerometer_z;
//     float magnetometer_x, magnetometer_y, magnetometer_z;
//     float gyro_x, gyro_y, gyro_z;

//     void readData(std::string input_data);
// };

// int main()
// {
//     //MoveData data;

//     Motor azimuth_motor(constants::MICROSTEPS_PER_REV, constants::AZIMUTH_STEP_PIN, constants::AZIMUTH_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);
//     //Motor elevation_motor(constants::MICROSTEPS_PER_REV, constants::ELEVATION_STEP_PIN, constants::ELEVATION_DIR_PIN, constants::INIT_PWM_DELAY, constants::GPIO_CONTROLLER_PATH);

//     //SocketListener listener(constants::SOCKET_PATH);


//     //listener.attemptConnection();

//     // while (true)
//     // {
//     //     //data.readData(listener.fetchData());

//     //     //moveToAngle(azimuth_motor, 20);
//     //     //moveToAngle(elevation_motor, data.elevation);
//     // }

//     azimuth_motor.setPID(1, 0, 0, 2 * constants::INIT_PWM_DELAY);
//     azimuth_motor.setSetpointType(Motor::SetpointType::kSTEP);
//     azimuth_motor.usePID(true);

//     azimuth_motor.setStepSetpoint(2000);

//     while (!azimuth_motor.atSetpoint())
//     {
//         azimuth_motor.drive();
//     }

//     //delete constants::SOCKET_PATH;
// }

// void MoveData::readData(std::string input_data)
// {
//     std::stringstream data(input_data);

//     data >> this->azimuth >> this->elevation;
//     data >> this->accelerometer_x >> this->accelerometer_y >> this->accelerometer_z;
//     data >> this->magnetometer_x >> this->magnetometer_y >> this->magnetometer_z;
//     data >> this->gyro_x >> this->gyro_y >> this->gyro_z;

// }

// void moveToAngle(Motor &motor, float degrees)
// {
//     if (motor.getSetpointType() != Motor::SetpointType::kSTEP)
//     {
//         motor.setSetpointType(Motor::SetpointType::kSTEP);
//     }

//     int steps = int(degrees * 128.0);

//     steps -= steps/575;

//     float current_deg = float(motor.getSteps()) / 128.0;

//     float forward_turn_error = 360.0 + degrees - current_deg;
//     float backward_turn_error = current_deg - degrees;

//     if (forward_turn_error < backward_turn_error)
//     {
//         int new_steps = int((360.0 + degrees) * 128.0);
//         new_steps -= new_steps/575;

//         motor.setStepSetpoint(new_steps);
//         motor.setSteps(int((current_deg - degrees) * 128.0));
//     }
//     else
//     {
//         motor.setStepSetpoint(steps);
//     }

//     //Error will still accumulate for many, small increments.
//     //Backsteps are only calculate for the given degrees.
// }