/*

SPDX-License-Identifier: MIT

MIT License

Copyright (c) 2019-2020 Mika Tuupola

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

*/

#include <stdint.h>

#include "axp192.h"
#include "axp192_config.h"

static i2c_read_fn i2c_read;
static i2c_write_fn i2c_write;
static const uint8_t DELAY_BIT = 1 << 7;

static const axp192_init_command_t init_commands[] = {
    {AXP192_LDO23_VOLTAGE, {CONFIG_AXP192_LDO23_VOLTAGE}, 1},
    {AXP192_DCDC13_LDO23_CONTROL, {CONFIG_AXP192_DCDC13_LDO23_CONTROL}, 1},
    {AXP192_EXTEN_DCDC2_CONTROL, {CONFIG_AXP192_EXTEN_DCDC2_CONTROL}, 1},
    {AXP192_ADC_ENABLE_1, {CONFIG_AXP192_ADC_ENABLE_1}, 1},
    {AXP192_CHARGE_CONTROL_1, {CONFIG_AXP192_CHARGE_CONTROL_1}, 1},
    /* End of commands . */
    {0, {0}, 0xff},
};

void axp192_init(i2c_read_fn i2c_read_ptr, i2c_write_fn i2c_write_ptr)
{
    uint8_t cmd = 0;

    /* Assign pointers to hal functions. */
    i2c_read = i2c_read_ptr;
    i2c_write = i2c_write_ptr;

    /* Send all the commands. */
    while (init_commands[cmd].count != 0xff) {
        i2c_write(
            AXP192_ADDRESS,
            init_commands[cmd].command,
            init_commands[cmd].data,
            init_commands[cmd].count & 0x1f
        );
        if (init_commands[cmd].count & DELAY_BIT) {
            // TODO: delay
            //vTaskDelay(200 / portTICK_RATE_MS);
        }
        cmd++;
    }
}

