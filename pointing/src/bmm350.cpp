#include "bmm350.hpp"

/*
WARNING: VIBE-CODED
*/

BMM350 *BMM350::instance_ = nullptr;

BMM350::BMM350()
{
    instance_ = this;

    dev_.read = readCallback;
    dev_.write = writeCallback;
    dev_.delay_us = delayCallback;

    //dev_.intf = BMM350_I2C_INTF;

    // I2C address pointer
    static uint8_t address = 0x14;
    dev_.intf_ptr = &address;
}

bool BMM350::init()
{
    return bmm350_init(&dev_) == BMM350_OK;
}

bool BMM350::setNormalMode()
{
    return bmm350_set_powermode(
               BMM350_NORMAL_MODE,
               &dev_)
           == BMM350_OK;
}

bool BMM350::setODR(
    enum bmm350_data_rates rate,
    enum bmm350_performance_parameters avg)
{
    return bmm350_set_odr_performance(
               rate,
               avg,
               &dev_)
           == BMM350_OK;
}

BMM350::Sample BMM350::read()
{
    bmm350_mag_temp_data data{};

    bmm350_get_compensated_mag_xyz_temp_data(
        &data,
        &dev_);

    return {
        data.x,
        data.y,
        data.z,
        data.temperature};
}