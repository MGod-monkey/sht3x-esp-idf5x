/**
 * Simple example with SHT3x sensor. 
 *
 * It shows different user task implementations in *single shot mode* and
 * *periodic mode*. In *single shot* mode either low level or high level
 * functions are used.
 * 
 * Constants SINGLE_SHOT_LOW_LEVEL and SINGLE_SHOT_HIGH_LEVEL controls which
 * task implementation is used.
 *
 * Harware configuration:
 *
 *    +-----------------+     +----------+
 *    | ESP8266 / ESP32 |     | SHT3x    |
 *    |                 |     |          |
 *    |   GPIO 5 (SCL) ------> SCL      |
 *    |   GPIO 4 (SDA) <-----> SDA      |
 *    +-----------------+     +----------+
 */

/* -- use following constants to define the example mode ----------- */

#define SINGLE_SHOT_LOW_LEVEL
// #define SINGLE_SHOT_HIGH_LEVEL

/* -- includes ----------------------------------------------------- */

#include "sht3x.h"
#include "driver/i2c.h"

/* -- platform dependent definitions ------------------------------- */
#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

#else  // ESP8266 (esp-open-rtos)

// user task stack depth for ESP8266
#define TASK_STACK_DEPTH 256

#endif  // ESP_PLATFORM

/* -- user tasks --------------------------------------------------- */
static sht3x_sensor_t* sensor;    // sensor device data structure

#if defined(SINGLE_SHOT_HIGH_LEVEL)
/*
 * User task that triggers a measurement every 5 seconds. Due to power
 * efficiency reasons it uses *single shot* mode. In this example it uses the
 * high level function *sht3x_measure* to perform one measurement in each cycle.
 */
void user_task (void *pvParameters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    while (1) 
    {
        // perform one measurement and do something with the results
        if (sht3x_measure (sensor, &temperature, &humidity))
            printf("%.3f SHT3x Sensor: %.2f °C, %.2f %%\n", 
        (double)sdk_system_get_time()*1e-3, temperature, humidity);

        // wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, 5000 / portTICK_PERIOD_MS);
    }
}

#elif defined(SINGLE_SHOT_LOW_LEVEL)
/*
 * User task that triggers a measurement every 5 seconds. Due to power
 * efficiency reasons it uses *single shot* mode. In this example it starts the
 * measurement, waits for the results and fetches the results using separate
 * functions
 */
void user_task (void *pvParameters)
{
    float temperature;
    float humidity;

    TickType_t last_wakeup = xTaskGetTickCount();

    // get the measurement duration for high repeatability;
    uint8_t duration = sht3x_get_measurement_duration(sht3x_high);
    
    while (1) 
    {
        // Trigger one measurement in single shot mode with high repeatability.
        sht3x_start_measurement (sensor, sht3x_single_shot, sht3x_high);
        
        // Wait until measurement is ready (constant time of at least 30 ms
        // or the duration returned from *sht3x_get_measurement_duration*).
        vTaskDelay (duration);
        
        // retrieve the values and do something with them
        if (sht3x_get_results (sensor, &temperature, &humidity))
            printf("%.3f SHT3x Sensor: %.2f °C, %.2f %%\n", 
        (double)sdk_system_get_time()*1e-3, temperature, humidity);

        // wait until 5 seconds are over
        vTaskDelayUntil(&last_wakeup, 5000 / portTICK_PERIOD_MS);
    }
}

#else  // PERIODIC MODE
/*
 * User task that fetches latest measurement results of sensor every 2
 * seconds. It starts the SHT3x in periodic mode with 1 measurements per
 * second (*sht3x_periodic_1mps*).
 */
void user_task (void *pvParameters)
{
    float temperature;
    float humidity;

    // Start periodic measurements with 1 measurement per second.
    sht3x_start_measurement (sensor, sht3x_periodic_1mps, sht3x_high);

    // Wait until first measurement is ready (constant time of at least 30 ms
    // or the duration returned from *sht3x_get_measurement_duration*).
    vTaskDelay (sht3x_get_measurement_duration(sht3x_high));

    TickType_t last_wakeup = xTaskGetTickCount();
    
    while (1) 
    {
        // Get the values and do something with them.
        if (sht3x_get_results (sensor, &temperature, &humidity))
            printf("%.3f SHT3x Sensor: %.2f °C, %.2f %%\n", 
        (double)sdk_system_get_time()*1e-3, temperature, humidity);
        // Wait until 2 seconds (cycle time) are over.
        vTaskDelayUntil(&last_wakeup, 2000 / portTICK_PERIOD_MS);
    }
}
#endif

#define I2C_MASTER_SCL_IO           GPIO_NUM_5      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_4      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000                       /*!< I2C master timeout in milliseconds */

/* -- main program ------------------------------------------------- */

void app_main(void)
{
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);
    
    // Init I2C bus interfaces at which SHT3x sensors are connected
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
        
    // Create the sensors, multiple sensors are possible.
    if ((sensor = sht3x_init_sensor(I2C_MASTER_NUM, SHT3x_ADDR_1)))
    {
        // Create a user task that uses the sensors.
        xTaskCreate(user_task, "user_task", TASK_STACK_DEPTH, NULL, 2, 0);
    }
    else
        printf("Could not initialize SHT3x sensor\n");
}

/* 
 * If the error "Could not initialize SHT3x sensor" appears, 
 * you can use the following program to scan the i2c bus for 
 * the 0x44/0x45 address to determine whether the SHT3x has 
 * been successfully soldered.
*/

// #include "driver/i2c.h"
// #include <stdio.h>

// #define I2C_MASTER_SCL_IO           GPIO_NUM_5      /*!< GPIO number used for I2C master clock */
// #define I2C_MASTER_SDA_IO           GPIO_NUM_4      /*!< GPIO number used for I2C master data  */
// #define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
// #define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_TIMEOUT_MS       1000                       /*!< I2C master timeout in milliseconds */

// void i2c_scanner()
// {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };

//     i2c_param_config(I2C_MASTER_NUM, &conf);
//     i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

//     printf("I2C scanner. Scanning...\n");

//     for (int address = 1; address < 127; address++) {
//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
//         i2c_master_stop(cmd);
//         esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
//         i2c_cmd_link_delete(cmd);

//         if (ret == ESP_OK) {
//             printf("I2C device found at address 0x%02x\n", address);
//         }
//     }

//     printf("Scan completed.\n");
// }

// void app_main(void)
// {
//     i2c_scanner();
// }