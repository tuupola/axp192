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
#include <esp_log.h>

#include "axp192.h"

i2c_read_t i2c_read;
i2c_write_t i2c_write;

static inline void _i2c_master_write(uint8_t reg, uint8_t data)
{
    i2c_write(AXP192_ADDRESS, reg, &data, 1);
}

void axp192_init(i2c_read_t i2c_read_ptr, i2c_write_t i2c_write_ptr)
{
    i2c_read = i2c_read_ptr;
    i2c_write = i2c_write_ptr;

    _i2c_master_write(AXP192_EXTEN_DCDC2, 0xff);
    _i2c_master_write(AXP192_LDO23_VOLTAGE, 0xff);
    _i2c_master_write(AXP192_ADC_ENA1, 0xff);
    _i2c_master_write(AXP192_CHARGE_CTRL1, 0xc0);
    _i2c_master_write(AXP192_DCDC13_LDO23, 0x4d);
    _i2c_master_write(AXP192_PEK, 0x5c);
    _i2c_master_write(AXP192_GPIO0, 0x02);
    _i2c_master_write(AXP192_VOFF, 0x04);

}

