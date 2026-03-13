#include <iostream>
#include <gpiod.hpp>
#include <vector>
#include <chrono>
#include <filesystem>
#include <thread>
#include <algorithm>
#include <atomic>

#include "MotorClock.hpp"

class Motor
{
    public:

    enum class SetpointType
    {
        kSTEP,
        kTIMER,
        kNONE
    };

    private:

    struct PID
    {
        double P_constant, I_constant, D_constant;

        std::chrono::time_point<std::chrono::steady_clock> time_sp, last_time, now_time;
        std::chrono::microseconds time_sp_error, last_time_sp_error, time_difference, output_max;

        int step_sp, last_step, last_step_sp_error, step_sp_error;
        std::atomic<int>& steps;

        std::chrono::microseconds i_sum;

        SetpointType sp_type;

        MotorClock& clk;

        PID(double P, double I, double D, MotorClock& clock, std::atomic<int>& steps, std::chrono::microseconds output_max);

        std::chrono::microseconds outputP();
        std::chrono::microseconds outputI();
        std::chrono::microseconds outputD();

        std::chrono::microseconds calculate();

    };

    gpiod::chip m_chip;

    MotorClock m_clk;

    std::atomic<bool> m_driving;
    bool m_using_pid;

    const unsigned int m_resolution;

    std::atomic<int> m_microsteps;
    
    int m_microstep_point, m_revs, m_step_setpoint;

    //std::vector<std::vector<unsigned int>> m_step_sequence;
    std::vector<gpiod::line::value_mappings> m_step_sequence;

    //std::vector<gpiod::line::offset> m_offsets;
    gpiod::line::offsets m_offsets;

    std::vector<gpiod::line_request> m_mag_requests;

    std::chrono::duration<std::chrono::microseconds> m_timer_setpoint;

    SetpointType m_setpoint_type;
    PID m_pid;

    std::thread m_drive_thread;

    void drive_thread_func();

    public:

    /*
    @param step_resolution The number of microsteps that create a full revolution. Turn degrees / 360
    @param gpio_pins a vector of the gpio_pins that connect to the motor magnets. First -> Top. Next -> Right. Next -> Bottom. Last -> Left.
    @param PWM The delay between each microstep. Guides how fast the motor will move.
    @param gpio_chip_path The file directory path of the GPIO chip used to control the gpio pins. Can take a string.
    */
    Motor(const unsigned int step_resolution, const std::vector<unsigned int> gpio_pins, const std::chrono::microseconds PWM,
        std::filesystem::path gpio_chip_path);

    /*
    EXTREMELY IMPORTANT TO CALL THE DECONSTRUCTOR WHEN FINISHED. If not, ownereship of the gpio pins will remain transferred and resources will remain allocated.
    */
    ~Motor();

    /*
    Restarts the thread used for driving.
    */
    void drive();

    /*
    Stops the drive loop*/
    void stop();

    void setSetpointType(SetpointType type);
    void setStepSetpoint(int steps, bool set_type = false);
    void setRevSetpoint(int revs, bool set_type = false);
    void setTimerSetpoint(std::chrono::microseconds time_delay, bool set_type = false);
    void setPWM(std::chrono::microseconds PWM);
    void setPID(double P, double I, double D,  std::chrono::microseconds output_max);

    void usePID(bool state);

    bool atSetpoint();

    MotorClock getCurrentClock();

};