# Platform agnostic I2C driver for AXP192 PMU

To use this library you must to provide functions for both reading and writing the I2C bus. Function definitions must be the following.

```c
int32_t i2c_read(void *handle, uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size);
int32_t i2c_write(void *handle, uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t size);
```

Where `address` is the I2C address, `reg` is the register to read or write, `buffer` holds the data to write or read into and `size` is the amount of data to read or write. The `handle` parameter is an optional customizable argument. You can use it if your I2C implementation requires any additional information such has number of the hardware I2C driver. For example HAL implementation see [ESP I2C helper](https://github.com/tuupola/esp_i2c_helper). For working example see [M5StickC kitchen sink](https://github.com/tuupola/esp_m5stick).

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

/* All ADC registers will be read as floats. */
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

/* All non ADC registers will be read as a raw bytes. */
/* See axp192.h or datasheet for all possible registers. */
axp192_read(&axp, AXP192_POWER_STATUS, &power);
axp192_read(&axp, AXP192_CHARGE_STATUS, &charge);

printf("power: 0x%02x charge: 0x%02x", power, charge);

axp192_ioctl(&axp, AXP192_COULOMB_COUNTER_ENABLE);
axp192_read(&axp, AXP192_COULOMB_COUNTER, &cbat);

printf("cbat: %.2fmAh", cbat);

/* Shortcuts for common tasks which otherwise would require */
/* multiple steps or function calls. */
axp192_ioctl(&axp, AXP192_LDO2_ENABLE);
axp192_ioctl(&axp, AXP192_LDO2_DISABLE);

axp192_ioctl(&axp, AXP192_LDO3_ENABLE);
axp192_ioctl(&axp, AXP192_LDO3_DISABLE);

axp192_ioctl(&axp, AXP192_DCDC3_ENABLE);
axp192_ioctl(&axp, AXP192_DCDC3_DISABLE);

axp192_ioctl(&axp, AXP192_DCDC1_SET_VOLTAGE, 3300);
axp192_ioctl(&axp, AXP192_DCDC2_SET_VOLTAGE, 2275);
axp192_ioctl(&axp, AXP192_DCDC3_SET_VOLTAGE, 3300);
axp192_ioctl(&axp, AXP192_LDOIO0_SET_VOLTAGE, 3300);
axp192_ioctl(&axp, AXP192_LDO2_SET_VOLTAGE, 3300);
axp192_ioctl(&axp, AXP192_LDO3_SET_VOLTAGE, 3300);

axp192_ioctl(&axp, AXP192_GPIO0_SET_LEVEL, AXP192_HIGH);
axp192_ioctl(&axp, AXP192_GPIO0_SET_LEVEL, AXP192_LOW);

axp192_ioctl(&axp, AXP192_GPIO1_SET_LEVEL, AXP192_HIGH);
axp192_ioctl(&axp, AXP192_GPIO1_SET_LEVEL, AXP192_LOW);

axp192_ioctl(&axp, AXP192_GPIO2_SET_LEVEL, AXP192_HIGH);
axp192_ioctl(&axp, AXP192_GPIO2_SET_LEVEL, AXP192_LOW);

axp192_ioctl(&axp, AXP192_GPIO4_SET_LEVEL, AXP192_HIGH);
axp192_ioctl(&axp, AXP192_GPIO4_SET_LEVEL, AXP192_LOW);
```

### Notes

| Output | Type | Example usage | Voltage   | Amperage |
|--------|------|---------------|-----------|----------|
| DCDC1  | Buck | 3.3V IO       | 0v7-3v5   | 1200mA   |
| DCDC2  | Buck | 1.25Vcore     | 0v7-2v275 | 1600mA   |
| DCDC3  | Buck | 2.5Vddr       | 0v7-3v5   | 700mA    |
| LDO1   | LDO  | RTC           | 3v3       | 30mA     |
| LDO2   | LDO  | Analog/FM     | 1v8-3v3   | 200mA    |
| LDO3   | LDO  | 1.8V HDMI     | 1v8-3v3   | 200mA    |
| LDOIO0 | LDO  | Vmic          | 1v8-3v3   | 50mA     |

LDO1 cannot be configured and it is always on.

| Output | M5StickC      | M5Core2   |
|--------|---------------|-----------|
| DCDC1  | VESP_3V3      | MCU_VDD   |
| DCDC2  |               |           |
| DCDC3  |               | LCD_BL    |
| LDO1   | RTC_VCC       | RTC_VDD   |
| LDO2   | LCD_BL_VCC    | PERI_VDD  |
| LDO3   | LCD_LOGIC_VCC | VIB_MOTOR |
| LDOIO0 | MIC_VCC       | BUS_PW_EN |
| IO1    |               | SYS_LED   |
| IO2    |               | SPK_EN    |
| IO3    |               |           |
| IO4    |               | LCD_RST   |
