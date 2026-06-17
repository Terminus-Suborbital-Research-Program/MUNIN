#pragma once

/*
WARNING: VIBE-CODED
*/

extern "C"
{
#include "BMM350/bmm350.h"
}

#include <cstdint>

class BMM350
{
public:
    struct Sample
    {
        float x;
        float y;
        float z;
        float temperature;
    };

    BMM350();

    bool init();

    bool setNormalMode();

    bool setODR(enum bmm350_data_rates rate,
                enum bmm350_performance_parameters avg);

    Sample read();

private:
    bmm350_dev dev_;

    // Hardware callbacks
    static BMM350 *instance_;

    static BMM350_INTF_RET_TYPE readCallback(
        uint8_t reg_addr,
        uint8_t *reg_data,
        uint32_t len,
        void *intf_ptr);

    static BMM350_INTF_RET_TYPE writeCallback(
        uint8_t reg_addr,
        const uint8_t *reg_data,
        uint32_t len,
        void *intf_ptr);

    static void delayCallback(uint32_t period_us, void *intf_ptr);

    // Replace these with your platform code
    int readRegister(uint8_t reg, uint8_t *buffer, uint32_t len);
    int writeRegister(uint8_t reg, const uint8_t *buffer, uint32_t len);
    void delayMicroseconds(uint32_t us);
};