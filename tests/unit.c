/*

MIT License

Copyright (c) 2026 Mika Tuupola

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

SPDX-License-Identifier: MIT

*/

#include "greatest.h"
#include "axp192.h"
#include "mock_i2c.h"

TEST
should_pass(void)
{
    PASS();
}

TEST
should_init(void)
{
    axp192_t axp;
    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    ASSERT(AXP192_OK == axp192_init(&axp));
    PASS();
}

TEST
should_read_battery_voltage(void)
{
    axp192_t axp;
    float voltage;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    /* Set mock ADC value: 0xEEA = 3818 -> 4.1998V */
    extern uint8_t memory[];
    memory[0x78] = 0xEE;
    memory[0x79] = 0x0A;

    ASSERT(AXP192_OK == axp192_read(&axp, AXP192_BATTERY_VOLTAGE, &voltage));
    ASSERT(voltage > 4.19 && voltage < 4.21);

    PASS();
}

TEST
should_fail_write_battery_voltage(void)
{
    axp192_t axp;
    uint8_t buffer = 0x00;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    /* ADC registers are read-only. */
    ASSERT(AXP192_ERROR_ENOTSUP == axp192_write(&axp, AXP192_BATTERY_VOLTAGE, &buffer));

    PASS();
}

TEST
should_write_data_buffer(void)
{
    axp192_t axp;
    uint8_t buffer = 0xDE;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    extern uint8_t memory[];

    ASSERT(AXP192_OK == axp192_write(&axp, AXP192_DATA_BUFFER0, &buffer));
    ASSERT(0xDE == memory[AXP192_DATA_BUFFER0]);

    PASS();
}

TEST
should_set_dcdc1_voltage(void)
{
    axp192_t axp;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    extern uint8_t memory[];

    ASSERT(AXP192_OK == axp192_ioctl(&axp, AXP192_DCDC1_SET_VOLTAGE, 3300));
    ASSERT(0x68 == memory[AXP192_DCDC1_VOLTAGE]);

    PASS();
}

TEST
should_fail_invalid_dcdc1_voltage(void)
{
    axp192_t axp;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    /* Below minimum */
    ASSERT(AXP192_ERROR_EINVAL == axp192_ioctl(&axp, AXP192_DCDC1_SET_VOLTAGE, 699));
    /* Above maximum */
    ASSERT(AXP192_ERROR_EINVAL == axp192_ioctl(&axp, AXP192_DCDC1_SET_VOLTAGE, 3501));

    PASS();
}

TEST
should_fail_invalid_dcdc2_voltage(void)
{
    axp192_t axp;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    /* Below minimum */
    ASSERT(AXP192_ERROR_EINVAL == axp192_ioctl(&axp, AXP192_DCDC2_SET_VOLTAGE, 699));
    /* Above maximum */
    ASSERT(AXP192_ERROR_EINVAL == axp192_ioctl(&axp, AXP192_DCDC2_SET_VOLTAGE, 2276));

    PASS();
}

TEST
should_fail_invalid_ioctl_command(void)
{
    axp192_t axp;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    ASSERT(AXP192_ERROR_NOTTY == axp192_ioctl(&axp, 0xFFFF));

    PASS();
}

TEST
should_enable_dcdc1(void)
{
    axp192_t axp;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    extern uint8_t memory[];
    memory[AXP192_DCDC13_LDO23_CONTROL] = 0x00;

    ASSERT(AXP192_OK == axp192_ioctl(&axp, AXP192_DCDC1_ENABLE));
    ASSERT(0x01 == (memory[AXP192_DCDC13_LDO23_CONTROL] & 0x01));

    PASS();
}

TEST
should_disable_dcdc1(void)
{
    axp192_t axp;

    axp.read = &mock_i2c_read;
    axp.write = &mock_i2c_write;

    extern uint8_t memory[];
    memory[AXP192_DCDC13_LDO23_CONTROL] = 0xFF;

    ASSERT(AXP192_OK == axp192_ioctl(&axp, AXP192_DCDC1_DISABLE));
    ASSERT(0x00 == (memory[AXP192_DCDC13_LDO23_CONTROL] & 0x01));

    PASS();
}

GREATEST_MAIN_DEFS();

int
main(int argc, char **argv)
{
    GREATEST_MAIN_BEGIN();

    RUN_TEST(should_pass);
    RUN_TEST(should_init);
    RUN_TEST(should_read_battery_voltage);
    RUN_TEST(should_fail_write_battery_voltage);
    RUN_TEST(should_write_data_buffer);
    RUN_TEST(should_set_dcdc1_voltage);
    RUN_TEST(should_fail_invalid_dcdc1_voltage);
    RUN_TEST(should_fail_invalid_ioctl_command);
    RUN_TEST(should_fail_invalid_dcdc2_voltage);
    RUN_TEST(should_enable_dcdc1);
    RUN_TEST(should_disable_dcdc1);

    GREATEST_MAIN_END();
}
