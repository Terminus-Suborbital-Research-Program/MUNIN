/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file  common.c
*
 */

#define _POSIX_C_SOURCE 199309L

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>

#include "../../bmm350.h"
#include "common.h"


/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function
 */
BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{   
    const struct bmm350_fd *bmm = (const struct bmm350_fd *)intf_ptr;
    int file = bmm->fd;
    __s32 res;

    res = i2c_smbus_read_i2c_block_data(file, reg_addr, (unsigned char)length, reg_data);

    if (res < 0 || (uint32_t)res != length) return BMM350_E_COM_FAIL;

    return BMM350_INTF_RET_SUCCESS;
}

/*!
 * I2C write function
 */
BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    const struct bmm350_fd *bmm = (const struct bmm350_fd *)intf_ptr;
    int file = bmm->fd;
    __s32 res;

    res = i2c_smbus_write_i2c_block_data(file, reg_addr, (unsigned char)length, reg_data);

    if (res != 0) return BMM350_E_COM_FAIL;

    return BMM350_INTF_RET_SUCCESS;
}

/*!
 * Delay function (microseconds)
 */
void bmm350_delay(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    struct timespec ts;
    ts.tv_sec = (time_t)(period / 1000000U);
    ts.tv_nsec = (long)((period % 1000000U) * 1000U);
    (void)nanosleep(&ts, NULL);
}

/*!
 *  @brief Prints the execution status of the APIs.
 *  @param[out] api_name    : API which triggered the error.
 *  @param[in] rslt     : Error trriggered.
 */
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMM350_OK:
            break;

        case BMM350_E_NULL_PTR:
            printf("%s Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMM350_E_COM_FAIL:
            printf("%s Error [%d] : Communication fail\r\n", api_name, rslt);
            break;
        case BMM350_E_DEV_NOT_FOUND:
            printf("%s Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMM350_E_INVALID_CONFIG:
            printf("%s Error [%d] : Invalid configuration\r\n", api_name, rslt);
            break;
        case BMM350_E_BAD_PAD_DRIVE:
            printf("%s Error [%d] : Bad pad drive\r\n", api_name, rslt);
            break;
        case BMM350_E_RESET_UNFINISHED:
            printf("%s Error [%d] : Reset unfinished\r\n", api_name, rslt);
            break;
        case BMM350_E_INVALID_INPUT:
            printf("%s Error [%d] : Invalid input\r\n", api_name, rslt);
            break;
        case BMM350_E_SELF_TEST_INVALID_AXIS:
            printf("%s Error [%d] : Self-test invalid axis selection\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_BOOT:
            printf("%s Error [%d] : OTP boot\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_PAGE_RD:
            printf("%s Error [%d] : OTP page read\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_PAGE_PRG:
            printf("%s Error [%d] : OTP page prog\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_SIGN:
            printf("%s Error [%d] : OTP sign\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_INV_CMD:
            printf("%s Error [%d] : OTP invalid command\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_UNDEFINED:
            printf("%s Error [%d] : OTP undefined\r\n", api_name, rslt);
            break;
        case BMM350_E_ALL_AXIS_DISABLED:
            printf("%s Error [%d] : All axis are disabled\r\n", api_name, rslt);
            break;
        case BMM350_E_PMU_CMD_VALUE:
            printf("%s Error [%d] : Unexpected PMU CMD value\r\n", api_name, rslt);
            break;
        default:
            printf("%s Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

/*!
 *  @brief Function to select the interface.
 *  @param[in, out] dev     : Structure instance of bmm350_dev.
 *
 * @return Result of API execution status
 * @retval = 0 -> Success
 * @retval < 0 -> Error
 */
int8_t bmm350_interface_init(struct bmm350_dev *dev)
{
    char filename[20];

    if (dev == NULL) {
        fprintf(stderr, "BMM350: bmm350_interface_init: NULL device pointer\n");
        return BMM350_E_NULL_PTR;
    }

    snprintf(filename, sizeof(filename), "/dev/i2c-%d", IMU_BUS);
    const int file = open(filename, O_RDWR);
    if (file < 0) {
        fprintf(stderr, "BMM350: failed to open I2C bus %s: %s\n",
                filename, strerror(errno));
        return BMM350_E_DEV_NOT_FOUND;
    }

    dev_addr = BMM350_I2C_ADSEL_SET_LOW;

    if (ioctl(file, I2C_SLAVE, dev_addr) < 0) {
        fprintf(stderr, "BMM350: no sensor at I2C address 0x%02X on %s: %s\n",
                dev_addr, filename, strerror(errno));
        close(file);
        if (errno == ENXIO) {
            return BMM350_E_DEV_NOT_FOUND;
        }
        return BMM350_E_COM_FAIL;
    }

    static struct bmm350_fd interface = {
        .fd = -1,
        .addr = BMM350_I2C_ADSEL_SET_LOW,
    };
    interface.fd = file;
    dev->intf_ptr = &interface;
    dev->read = bmm350_i2c_read;
    dev->write = bmm350_i2c_write;
    dev->delay_us = bmm350_delay;

    return BMM350_OK;
}

void bmm350_interface_deinit(struct bmm350_dev *dev) {
    if (dev == NULL) {
        fprintf(stderr, "BMM350: bmm350_interface_deinit: NULL device pointer\n");
        return;
    }
    if (dev->intf_ptr == NULL) {
        fprintf(stderr, "BMM350: bmm350_interface_deinit: NULL intf_ptr (init never called or failed)\n");
        return;
    }
    const struct bmm350_fd *bmm = (const struct bmm350_fd *)dev->intf_ptr;
    if (bmm->fd >= 0) {
        close(bmm->fd);
    }
    dev->intf_ptr = NULL;
}

/*!
 *  @brief Function to deinitialize the interface.
 *
 * @return Result of API execution status
 * @retval = 0 -> Success
 * @retval < 0 -> Error
 */
void bmm350_coines_deinit(void)
{
    (void)fflush(stdout);
}

/**
 * @brief Prints a fixed-point 48.16 value as a decimal number with 5 decimal places.
 *
 * @param[in] raw  The 64-bit signed integer representing a 48.16 fixed-point value.
 */
void print_A48_16(int64_t raw)
{
    int64_t integer = (raw >> 16); /* signed integer part */
    uint32_t frac_raw = (uint32_t)((raw < 0 ? -raw : raw) & 0xFFFF);

    /* scale fraction to 5 decimal places */
    uint32_t frac_dec = (uint32_t)((uint64_t)(frac_raw * 100000ULL) >> 16);

#ifdef PC

    if (raw >= 0)
    {
        printf("%d.%05u", (int32_t)integer, frac_dec);
    }
    else
    {
        /* handle negative: print integer and fraction as negative */
        if (frac_dec == 0)
        {
            printf("%d", (int32_t)integer + 1);
        }
        else
        {
            if (integer == 0)
            {
                printf("-");
            }

            printf("%d.%05u", (int32_t)integer + 1, frac_dec);
        }
    }

#else
    if (raw >= 0)
    {
        printf("%ld.%05lu ", (int32_t)integer, frac_dec);
    }
    else
    {
        /* handle negative: print integer and fraction as negative */
        if (frac_dec == 0)
        {
            printf("%ld ", (int32_t)integer + 1);
        }
        else
        {
            if (integer == 0)
            {
                printf("-");
            }

            printf("%ld.%05lu ", (int32_t)integer + 1, frac_dec);
        }
    }

#endif

}