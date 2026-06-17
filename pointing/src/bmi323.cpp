#include "bmi323.hpp"

class LinuxSPIBus : public SPIBus
{
public:
    void read(
        uint8_t reg,
        uint8_t *data,
        uint32_t len) override;

    void write(
        uint8_t reg,
        const uint8_t *data,
        uint32_t len) override;
};

BMI323::BMI323(
    SPIBus &spi,
    DelayProvider &delay)
{
    callback_data_.spi = &spi;
    callback_data_.delay = &delay;

    dev_.read = read;
    dev_.write = write;
    dev_.delay_us = delayUs;
    dev_.intf = BMI3_SPI_INTF;
    dev_.intf_ptr = &callback_data_;
}

bool BMI323::init()
{
    if (bmi323_init(&dev_) != BMI323_OK)
    {
        return false;
    }

    bmi3_sens_config cfg[2];

    cfg[0].type = BMI3_ACCEL;
    cfg[0].cfg.acc.odr = 100;
    cfg[0].cfg.acc.range = BMI3_ACC_RANGE_2G;
    cfg[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
    cfg[0].cfg.acc.avg_num = BMI3_ACC_AVG1;
    cfg[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

    cfg[1].type = BMI3_GYRO;
    cfg[1].cfg.gyr.odr = 100;
    cfg[1].cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;
    cfg[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
    cfg[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;
    cfg[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_QUARTER;

    return bmi323_set_sensor_config(
        cfg,
        2,
        &dev_) == BMI323_OK;
}

BMI323::Vector3 BMI323::getAccel()
{
    bmi3_sensor_data data;
    data.type = BMI3_ACCEL;

    bmi323_get_sensor_data(&data, 1, &dev_);

    return {
        static_cast<float>(data.sens_data.acc.x),
        static_cast<float>(data.sens_data.acc.y),
        static_cast<float>(data.sens_data.acc.z)
    };
}

BMI323::Vector3 BMI323::getGyro()
{
    bmi3_sensor_data data;
    data.type = BMI3_GYRO;

    bmi323_get_sensor_data(&data, 1, &dev_);

    return {
        static_cast<float>(data.sens_data.gyr.x),
        static_cast<float>(data.sens_data.gyr.y),
        static_cast<float>(data.sens_data.gyr.z)
    };
}

int8_t BMI323::read(
    uint8_t reg,
    uint8_t *data,
    uint32_t len,
    void *intf_ptr)
{
    auto *cb =
        static_cast<CallbackData *>(intf_ptr);

    cb->spi->read(
        reg,
        data,
        len);

    return BMI323_OK;
}

int8_t BMI323::write(
    uint8_t reg,
    const uint8_t *data,
    uint32_t len,
    void *intf_ptr)
{
    auto *cb =
        static_cast<CallbackData *>(intf_ptr);

    cb->spi->write(
        reg,
        data,
        len);

    return BMI323_OK;
}

void BMI323::delayUs(
    uint32_t us,
    void *intf_ptr)
{
    auto *cb =
        static_cast<CallbackData *>(intf_ptr);

    cb->delay->delayUs(us);
}