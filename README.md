# Hardware agnostic I2C driver for AXP192 PMU

To use this library you must to provide HAL functions for both reading and writing the I2C bus. Function definitions must be the following.

```c
int32_t i2c_read(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size);
int32_t i2c_write(uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t size);
```

Where `address` is the I2C address, `reg` is the register to read or write, `buffer` holds the data to write or read into and `size` is the amount of data to read or write. For example HAL implementation see [ESP I2C master HAL](https://github.com/tuupola/esp_i2c_hal). For working example see [M5StickC kitchen sink](https://github.com/tuupola/esp_m5stick).

## Usage

```
$ make menuconfig
```

```c
#include "axp192.h"
#include "your-i2c-hal.h"

float vacin, iacin, vvbus, ivbus, temp, pbat, vbat, icharge, idischarge, vaps, cbat;
uint8_t power, charge;
axp192_t axp;

/* Add pointers to HAL functions. */
axp.read = &i2c_read;
axp.write = &i2c_write;

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