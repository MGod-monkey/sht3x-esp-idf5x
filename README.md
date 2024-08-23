# Driver for **SHT3x** digital **temperature and humidity sensor**


### This repository was forked and modified from [https://github.com/gschorcht/sht3x-esp-idf](https://github.com/gschorcht/sht3x-esp-idf), you can go to the original repository for the description

Here are some new features:
- Adapting to ESP-IDF5.x
- Add a scanner, you can detect whether sht3x is normal by scanning I2C devices

---

### Description

If the error "Could not initialize SHT3x sensor" appears, you can use the following program to scan the i2c bus for the 0x44/0x45 address to determine whether the SHT3x has been successfully soldered.

```c
#include "driver/i2c.h"
#include <stdio.h>

#define I2C_MASTER_SCL_IO           GPIO_NUM_5      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_4      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000                       /*!< I2C master timeout in milliseconds */

void i2c_scanner()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

    printf("I2C scanner. Scanning...\n");

    for (int address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            printf("I2C device found at address 0x%02x\n", address);
        }
    }

    printf("Scan completed.\n");
}

void app_main(void)
{
    i2c_scanner();
}
```

