#include "Motor.hpp"

using namespace std::chrono_literals;

Motor::Motor(const unsigned int step_resolution, const std::vector<unsigned int> gpio_pins, const std::chrono::microseconds PWM, 
    std::filesystem::path gpio_chip_path)
    : m_chip(gpio_chip_path)
    , m_driving(false)
    , m_using_pid(false)
    , m_resolution(step_resolution)
    , m_microstep_point(0)
    , m_revs(0)
    , m_microsteps(0)
    , m_step_setpoint(0)
    , m_setpoint_type(Motor::SetpointType::kNONE)
    , m_timer_setpoint(0us)
    , m_offsets()
    , m_clk(PWM)
    , m_pid(0.0, 0.0, 0.0, m_clk, m_microsteps, 0us)
    , m_drive_thread()
{ 
    for (unsigned int pin : gpio_pins) //Initialize gpio pins
    {
        m_offsets.push_back(gpiod::line::offset(pin));
    }

    const unsigned int max = gpio_pins.size();

    std::vector<gpiod::line::value_mappings> sequence;
    for (int i = 0; i < (max*2); i++) //Fill step sequence
    {
        for (int j = 0; j < max; j++)
        {
            sequence[i].push_back({m_offsets[j], gpiod::line::value::INACTIVE});
        }
    }

    //std::vector<std::vector<unsigned int>> sequence(max*2, std::vector<unsigned int>(max, 0));

    // for (int i = 0; i < (max*2); i++) //Generate step sequence
    // {
    //     int primary = i / 2;
    //     sequence[i][primary] = 1;

    //     if (i % 2 == 1)
    //         sequence[i][(primary + 1) % max] = 1;
    // }

    for (int i = 0; i < (max*2); i++) //Generate step sequence
    {
        int primary = i / 2;
        sequence[i][primary] = {sequence[i][primary].first, gpiod::line::value::ACTIVE};

        if (i % 2 == 1)
            sequence[i][(primary + 1) % max] = {sequence[i][primary].first, gpiod::line::value::ACTIVE};
    }
    
    m_step_sequence = std::move(sequence); //Set step sequence

    gpiod::line_settings settings;
    settings.set_direction(gpiod::line::direction::OUTPUT); //Set pins for taking requests
    settings.set_output_value(gpiod::line::value::INACTIVE); //By default, be inactive

    gpiod::line_config config;
    config.add_line_settings(m_offsets, settings); 
    //configure the associated request with the magnet GPIO pin and the preconfigured settings

    gpiod::request_builder builder = m_chip.prepare_request(); //Create a request builder from the GPIO chip
    builder.set_line_config(config); //Set the request builder's config to be the previously created config

    m_mag_requests.push_back(builder.do_request()); //Create request

    // for (unsigned int pin : gpio_pins) //Initialize gpio pins
    // {
    //     m_offsets.push_back(gpiod::line::offset(pin));

    //     gpiod::line_config config;
    //     config.add_line_settings(pin, settings); //configure the associated request with the magnet GPIO pin and the preconfigured settings

    //     gpiod::request_builder builder = m_chip.prepare_request(); //Create a request builder from the GPIO chip
    //     builder.set_line_config(config); //Set the request builder's config to be the previously created config

    //     m_mag_requests.push_back(builder.do_request()); //Create request
    //}
}

Motor::~Motor()
{
    for (int i = 0; i < m_mag_requests.size(); i++)
    {
        m_mag_requests[i].release();
    }
}

Motor::PID::PID(double P, double I, double D, MotorClock& clock, int& steps, std::chrono::microseconds output_max)
: P_constant(P)
, I_constant(I)
, D_constant(D)
, last_time()
, now_time()
, time_sp()
, time_sp_error()
, last_time_sp_error()
, step_sp(0)
, last_step(0)
, step_sp_error(0)
, last_step_sp_error(0)
, steps(steps)
, time_difference()
, i_sum()
, sp_type(SetpointType::kNONE)
, clk(clock)
{
    now_time = last_time = clk.getClock().now();
}

std::chrono::microseconds Motor::PID::outputP()
{
    switch (sp_type)
    {
        case SetpointType::kTIMER:
            return std::chrono::microseconds(int(P_constant * double(time_sp_error.count())));
        case SetpointType::kSTEP:
            return std::chrono::microseconds(int(P_constant * step_sp_error));
        case SetpointType::kNONE:
            return std::chrono::microseconds(0);
    }
}

std::chrono::microseconds Motor::PID::outputI()
{
    std::chrono::microseconds add;

    switch (sp_type)
    {
        case SetpointType::kTIMER:
            add = std::chrono::microseconds(int( I_constant * double(now_time.time_since_epoch().count()) 
            * (time_sp_error.count()) ));
            break;
        case SetpointType::kSTEP:
            add = std::chrono::microseconds(int( I_constant * double(now_time.time_since_epoch().count()) 
            * (double(step_sp_error)) ));
            break;
        case SetpointType::kNONE:
            return std::chrono::microseconds(0);
    }

    i_sum += add;
        
    return i_sum;
}

std::chrono::microseconds Motor::PID::outputD()
{
    switch (sp_type)
    {
        case SetpointType::kTIMER:
            return std::chrono::microseconds(int( D_constant * double(time_sp_error.count())
                    / double((time_difference).count())));
        case SetpointType::kSTEP:
            return std::chrono::microseconds(int( D_constant * (double(step_sp_error))
                    / double((time_difference).count())));
        case SetpointType::kNONE:
            return std::chrono::microseconds(0);
    }
}

std::chrono::microseconds Motor::PID::calculate()
{
    now_time = clk.getClock().now();

    time_sp_error = std::chrono::microseconds((time_sp - last_time).count());
    time_difference = std::chrono::microseconds((now_time - last_time).count());
    step_sp_error = step_sp - steps;

    std::chrono::microseconds output = outputP() + outputI() + outputD();

    last_time = now_time;
    last_step = steps;

    last_step_sp_error = step_sp_error;
    last_time_sp_error = time_sp_error;

    return output;
}


bool Motor::atSetpoint()
{
    switch (m_setpoint_type)
    {
        case SetpointType::kTIMER:
            return m_clk.pastTimer();

        case SetpointType::kSTEP:
            return (m_microsteps >= m_step_setpoint);

        case SetpointType::kNONE:
            return true;
    }
}

void Motor::usePID(bool state)
{
    m_using_pid = state;
}

void Motor::drive_thread_func()
{
    unsigned int i = 0;

    while (true)
    {
        if (m_driving && !atSetpoint())
        {
            i += m_clk.pastDelay(); //Increments iterator only when past the delay
            i %= m_step_sequence.size() - 1; //Keep iterator in bounds for looping
    
            m_mag_requests[0].set_values(m_step_sequence[i]); //Apply step configuration
            
            if (m_using_pid)
            {
                m_clk.setDelay(m_pid.calculate());
            }
        }
    }
}

void Motor::drive()
{
    m_driving = true;

    m_drive_thread.join();
    m_drive_thread = std::thread(drive_thread_func);
    m_drive_thread.detach();
}

void Motor::stop()
{
    m_driving = false;
}

void Motor::setSetpointType(SetpointType type)
{
    m_setpoint_type = type;
    m_pid.sp_type = type;
}

void Motor::setStepSetpoint(int steps, bool set_type = false)
{
    m_step_setpoint = steps;

    if (set_type)
    {
        setSetpointType(Motor::SetpointType::kSTEP);
    }
}
void Motor::setRevSetpoint(int revs, bool set_type = false)
{
    setStepSetpoint(revs * m_resolution, set_type);
}

void Motor::setTimerSetpoint(std::chrono::microseconds time_delay, bool set_type = false)
{
    m_clk.setTimer(time_delay);

    if (set_type)
    {
        setSetpointType(Motor::SetpointType::kTIMER);
    }
}

/*
Unnecessary when using PID
*/
void Motor::setPWM(std::chrono::microseconds PWM)
{
    m_clk.setDelay(PWM);
}

void Motor::setPID(double P, double I, double D, std::chrono::microseconds output_max)
{
    m_pid.P_constant = P;
    m_pid.I_constant = I;
    m_pid.D_constant = D;
    m_pid.output_max = output_max;
}