# Platform agnostic I2C driver for AXP192 PMU

To use this library you must to provide functions for both reading and writing the I2C bus. Function definitions must be the following.

```c
int32_t i2c_read(void *handle, uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size);
int32_t i2c_write(void *handle, uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t size);
```

Where `address` is the I2C address, `reg` is the register to read or write, `buffer` holds the data to write or read into and `size` is the amount of data to read or write. The `handle` parameter is an optional customizable argument. You can use it if your I2C implementation requires any additional information such has number of the hardware I2C driver. For an example HAL implementation see [ESP I2C helper](https://github.com/tuupola/esp_i2c_helper). For working example see [M5StickC kitchen sink](https://github.com/tuupola/esp_m5stick).


## Usage

```
$ make menuconfig
```

```c
#include "axp192.h"
#include "user_i2c.h"

float vacin, iacin, vvbus, ivbus, temp, pbat, vbat, icharge, idischarge, vaps, cbat;
uint8_t power, charge;
axp192_t axp;

/* Add pointers to HAL functions. */
axp.read = &user_i2c_read;
axp.write = &user_i2c_write;

/* You could set the handle here. It can be pointer to anything. */
axp.handle = NULL;

axp192_init(&axp);

axp192_read(&axp, AXP192_ACIN_VOLTAGE, &vacin);
axp192_read(&axp, AXP192_ACIN_CURRENT, &iacin);
axp192_read(&axp, AXP192_VBUS_VOLTAGE, &vvbus);
axp192_read(&axp, AXP192_VBUS_CURRENT, &ivbus);
axp192_read(&axp, AXP192_TEMP, &temp);
axp192_read(&axp, AXP192_TS_INPUT, &vts);
axp192_read(&axp, AXP192_BATTERY_POWER, &pbat);
axp192_read(&axp, AXP192_BATTERY_VOLTAGE, &vbat);
axp192_read(&axp, AXP192_CHARGE_CURRENT, &icharge);
axp192_read(&axp, AXP192_DISCHARGE_CURRENT, &idischarge);
axp192_read(&axp, AXP192_APS_VOLTAGE, &vaps);

printf(
    "vacin: %.2fV iacin: %.2fA vvbus: %.2fV ivbus: %.2fA vts: %.2fV temp: %.0fC "
    "pbat: %.2fmW vbat: %.2fV icharge: %.2fA idischarge: %.2fA, vaps: %.2fV",
    vacin, iacin, vvbus, ivbus, vts, temp, pbat, vbat, icharge, idischarge, vaps
);

axp192_ioctl(&axp, AXP192_READ_POWER_STATUS, &power);
axp192_ioctl(&axp, AXP192_READ_CHARGE_STATUS, &charge);

printf("power: 0x%02x charge: 0x%02x", power, charge);

axp192_ioctl(&axp, AXP192_COULOMB_COUNTER_ENABLE, NULL);
axp192_read(&axp, AXP192_COULOMB_COUNTER, &cbat);

printf("cbat: %.2fmAh", cbat);
```


&nbsp;

![](readme_assets/danger.png)

If you invoke `axp192_init()`, you will write the values set in menuconfig to the AXP192. And whether your write things this way or some other way: be aware that any values you write to the Power Management IC are stored in **non-volatile** memory. That means that when you tell the AXP192 to turn off the power to the CPU in a device, your device may very well be essentially dead until you find a way to turn the power back on by talking to the AXP192 without using the CPU. In a case we fortunately only heard about  - *cough* - this has involved using another device and some wires. Twice.

You do not need to use `axp192_init()` first, you can also read and write without using the menuconfig settings, using the functions provided. Have a look at [m5core2_axp192](https://github.com/ropg/m5core2_axp192) which provides device setup and some utility functions for the M5Core2 device using this driver for the communication with the AXP192.
