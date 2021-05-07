/*

MIT License

Copyright (c) 2019-2021 Mika Tuupola

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

-cut-

This file is part of hardware agnostic I2C driver for AXP192:
https://github.com/tuupola/axp192

SPDX-License-Identifier: MIT

*/

#include <stdint.h>

#include "axp192_config.h"
#include "axp192.h"

static axp192_err_t read_coloumb_counter(const axp192_t *axp, float *buffer);
static axp192_err_t read_battery_power(const axp192_t *axp, float *buffer);

static const axp192_init_command_t init_commands[] = {
    {AXP192_LDO23_VOLTAGE, {CONFIG_AXP192_LDO23_VOLTAGE}, 1},
    {AXP192_DCDC13_LDO23_CONTROL, {CONFIG_AXP192_DCDC13_LDO23_CONTROL}, 1},
    {AXP192_GPIO0_LDOIO0_VOLTAGE, {CONFIG_AXP192_GPIO0_LDOIO0_VOLTAGE}, 1},
    {AXP192_GPIO0_CONTROL, {CONFIG_AXP192_GPIO0_CONTROL}, 1},
    {AXP192_EXTEN_DCDC2_CONTROL, {CONFIG_AXP192_EXTEN_DCDC2_CONTROL}, 1},
    {AXP192_ADC_ENABLE_1, {CONFIG_AXP192_ADC_ENABLE_1}, 1},
    {AXP192_CHARGE_CONTROL_1, {CONFIG_AXP192_CHARGE_CONTROL_1}, 1},
    /* End of commands. */
    {0, {0}, 0xff},
};

axp192_err_t axp192_init(const axp192_t *axp)
{
    uint8_t cmd = 0;
    axp192_err_t status;

    /* Send all the commands. */
    while (init_commands[cmd].count != 0xff) {
        status = axp->write(
            axp->handle,
            AXP192_ADDRESS,
            init_commands[cmd].command,
            init_commands[cmd].data,
            init_commands[cmd].count & 0x1f
        );
        if (AXP192_OK != status) {
            return status;
        }
        cmd++;
    }

    return AXP192_OK;
}

axp192_err_t axp192_read(const axp192_t *axp, uint8_t reg, float *buffer)
{
    uint8_t tmp[4];
    float sensitivity = 1.0;
    float offset = 0.0;
    axp192_err_t status;

    switch (reg) {
    case AXP192_ACIN_VOLTAGE:
    case AXP192_VBUS_VOLTAGE:
        /* 1.7mV per LSB */
        sensitivity = 1.7 / 1000;
        break;
    case AXP192_ACIN_CURRENT:
        /* 0.375mA per LSB */
        sensitivity = 0.625 / 1000;
        break;
    case AXP192_VBUS_CURRENT:
        /* 0.375mA per LSB */
        sensitivity = 0.375 / 1000;
        break;
    case AXP192_TEMP:
        /* 0.1C per LSB, 0x00 = -144.7C */
        sensitivity = 0.1;
        offset = -144.7;
        break;
    case AXP192_TS_INPUT:
        /* 0.8mV per LSB */
        sensitivity = 0.8 / 1000;
        break;
    case AXP192_BATTERY_POWER:
        /* 1.1mV * 0.5mA per LSB */
        return read_battery_power(axp, buffer);
        break;
    case AXP192_BATTERY_VOLTAGE:
        /* 1.1mV per LSB */
        sensitivity = 1.1 / 1000;
        break;
    case AXP192_CHARGE_CURRENT:
    case AXP192_DISCHARGE_CURRENT:
        /* 0.5mV per LSB */
        sensitivity = 0.5 / 1000;
        break;
    case AXP192_APS_VOLTAGE:
        /* 1.4mV per LSB */
        sensitivity = 1.4 / 1000;
        break;
    case AXP192_COULOMB_COUNTER:
        /* This is currently untested. */
        return read_coloumb_counter(axp, buffer);
        break;
    }

    status = axp->read(axp->handle, AXP192_ADDRESS, reg, tmp, 2);
    if (AXP192_OK != status) {
        return status;
    }
    *buffer = (((tmp[0] << 4) + tmp[1]) * sensitivity) + offset;

    return AXP192_OK;
}

axp192_err_t axp192_ioctl(const axp192_t *axp, uint16_t command, uint8_t *buffer)
{
    uint8_t reg = command >> 8;
    uint8_t tmp;

    switch (command) {
    case AXP192_READ_POWER_STATUS:
    case AXP192_READ_CHARGE_STATUS:
        return axp->read(axp->handle, AXP192_ADDRESS, reg, buffer, 1);
        break;
    case AXP192_COULOMB_COUNTER_ENABLE:
        tmp = 0b10000000;
        return axp->write(axp->handle, AXP192_ADDRESS, reg, &tmp, 1);
        break;
    case AXP192_COULOMB_COUNTER_DISABLE:
        tmp = 0b00000000;
        return axp->write(axp->handle, AXP192_ADDRESS, reg, &tmp, 1);
        break;
    case AXP192_COULOMB_COUNTER_SUSPEND:
        tmp = 0b11000000;
        return axp->write(axp->handle, AXP192_ADDRESS, reg, &tmp, 1);
        break;
    case AXP192_COULOMB_COUNTER_CLEAR:
        tmp = 0b10100000;
        return axp->write(axp->handle, AXP192_ADDRESS, reg, &tmp, 1);
        break;
    }

    return AXP192_ERROR_NOTTY;
}

static axp192_err_t read_coloumb_counter(const axp192_t *axp, float *buffer)
{
    uint8_t tmp[4];
    int32_t coin, coout;
    axp192_err_t status;

    status = axp->read(axp->handle, AXP192_ADDRESS, AXP192_CHARGE_COULOMB, tmp, sizeof(coin));
    if (AXP192_OK != status) {
        return status;
    }
    coin = (tmp[0] << 24) + (tmp[1] << 16) + (tmp[2] << 8) + tmp[3];

    status = axp->read(axp->handle, AXP192_ADDRESS, AXP192_DISCHARGE_COULOMB, tmp, sizeof(coout));
    if (AXP192_OK != status) {
        return status;
    }
    coout = (tmp[0] << 24) + (tmp[1] << 16) + (tmp[2] << 8) + tmp[3];

    /* CmAh = 65536 * 0.5mA *（coin - cout) / 3600 / ADC sample rate */
    *buffer = 32768 * (coin - coout) / 3600 / 25;

    return AXP192_OK;
}

static axp192_err_t read_battery_power(const axp192_t *axp, float *buffer)
{
    uint8_t tmp[4];
    float sensitivity;
    axp192_err_t status;

    /* 1.1mV * 0.5mA per LSB */
    sensitivity = 1.1 * 0.5 / 1000;
    status = axp->read(axp->handle, AXP192_ADDRESS, AXP192_BATTERY_POWER, tmp, 3);
    if (AXP192_OK != status) {
        return status;
    }
    *buffer = (((tmp[0] << 16) + (tmp[1] << 8) + tmp[2]) * sensitivity);
    return AXP192_OK;
}