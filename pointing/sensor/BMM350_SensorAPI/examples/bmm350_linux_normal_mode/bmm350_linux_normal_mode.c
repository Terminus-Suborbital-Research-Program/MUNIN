/**
* Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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
* INCIDENTAL, SPECIAL, EXEMPLARY, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* @file  bmm350_linux_normal_mode.c
*
* @brief Linux-native example: read magnetometer data in normal mode via /dev/i2c-N.
*        No COINES dependency. Uses the Linux I2C userspace interface wired up
*        in examples/common/common.c.
*
*/

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#include "bmm350.h"
#include "common.h"

static volatile sig_atomic_t g_keep_running = 1;

static void handle_sigint(int signo) {
    (void)signo;
    g_keep_running = 0;
}

static uint64_t monotonic_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

int main(void) {
    int8_t rslt;
    struct bmm350_dev dev = { 0 };
    struct bmm350_raw_mag_data raw_data;
    uint8_t int_status = 0;
    uint64_t t_start_ms;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_sigint;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    rslt = bmm350_interface_init(&dev);
    bmm350_error_codes_print_result("bmm350_interface_init", rslt);
    if (rslt != BMM350_OK) {
        return EXIT_FAILURE;
    }

    rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);
    if (rslt != BMM350_OK) {
        bmm350_interface_deinit(&dev);
        return EXIT_FAILURE;
    }

    printf("BMM350 chip ID: 0x%X\n", dev.chip_id);

    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_4, &dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);
    if (rslt != BMM350_OK) {
        bmm350_interface_deinit(&dev);
        return EXIT_FAILURE;
    }

    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    bmm350_error_codes_print_result("bmm350_enable_axes", rslt);
    if (rslt != BMM350_OK) {
        bmm350_interface_deinit(&dev);
        return EXIT_FAILURE;
    }

    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
    if (rslt != BMM350_OK) {
        bmm350_interface_deinit(&dev);
        return EXIT_FAILURE;
    }

    printf("\nUncompensated magnetometer data (Ctrl-C to stop)\n");
    printf("t_ms, mag_x_raw, mag_y_raw, mag_z_raw\n");

    t_start_ms = monotonic_ms();

    while (g_keep_running) {
        int_status = 0;

        rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, &dev);
        bmm350_error_codes_print_result("bmm350_get_regs", rslt);
        if (rslt != BMM350_OK) {
            break;
        }

        if (int_status & BMM350_DRDY_DATA_REG_MSK) {
            rslt = bmm350_read_uncomp_mag_temp_data(&raw_data, &dev);
            bmm350_error_codes_print_result("bmm350_read_uncomp_mag_temp_data", rslt);
            if (rslt != BMM350_OK) {
                break;
            }

            printf("%llu, %ld, %ld, %ld\n",
                   (unsigned long long)(monotonic_ms() - t_start_ms),
                   (long)raw_data.raw_xdata,
                   (long)raw_data.raw_ydata,
                   (long)raw_data.raw_zdata);
        }
    }

    rslt = bmm350_set_powermode(BMM350_SUSPEND_MODE, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode(SUSPEND)", rslt);

    bmm350_interface_deinit(&dev);

    return EXIT_SUCCESS;
}
