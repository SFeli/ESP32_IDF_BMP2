/* 
    IDF für BME2 - Treiber von Bosch
    mit I2C - Standard-Bibliothek
    Für ESP32 aber auch ESP S3 mit idf.py ... im Terminal
*/

#include "stdio.h"
#include "stdlib.h"
#include <stdint.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "bmp2.h"
#include "bmp2_defs.h"

// ursprünglich aus Hauptprogramm
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define RTOS_DELAY_1SEC          ( 1 * 1000 / portTICK_PERIOD_MS)

#define BMP280_SENSOR_ADDR          0x76        /*!< Primary address of the BMP280 ensor */
#define BOSCH__REGISTER_CHIPID      0xD0        /*!< Register bei den Boschsensoren zur ID */
#define BOSCH_RESET_REGISTER        0xE0        /*!< Register addresses of the power managment register */
#define BOSCH_RESET_VALUE           0xB6

void delay_ms(uint32_t period_ms);

int8_t rslt;
struct bmp2_dev bmp;
struct bmp2_config conf;
static const char *TAG = "ESP32-BMP2";

/**
 *  @brief Function for reading the sensor's registers through I2C bus.
 *  @param[in]  reg_addr   : Register address.
 *  @param[out] reg_data   : Pointer to the data buffer to store the read data.
 *  @param[in]  length     : No of bytes to read.
 *  @param[in]  intf_ptr   : Interface pointer
 *  @return Status of execution
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 */
static BMP2_INTF_RET_TYPE bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    esp_err_t sfret;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    sfret = i2c_master_write_read_device(I2C_MASTER_NUM, *(int *)intf_ptr, &reg_addr, 1, reg_data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (sfret != ESP_OK)
        ESP_LOGE(TAG, "bmp2_i2c_read failed!");
    return 0;
}

/**
 *  @brief Function for writing the sensor's registers through I2C bus.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *  @param[in] intf_ptr : Interface pointer
 *  @return Status of execution
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 */
static BMP2_INTF_RET_TYPE bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t  len, void *intf_ptr)
{
    esp_err_t sfret;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uint8_t prim_adr =  *(int *)intf_ptr;
    uint8_t write_buf[2] = {reg_addr, (uint8_t)*reg_data};
    sfret = i2c_master_write_to_device(I2C_MASTER_NUM, prim_adr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (sfret != ESP_OK)
        ESP_LOGE(TAG, "i2c_master_write_to_device failed!");
    return sfret;
}

void delay_ms(uint32_t period_ms)
{
    vTaskDelay(period_ms / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_cmd_handle_t cmd;
	ESP_LOGI(TAG, "I2C_MASTER_NUM   :  %d", I2C_MASTER_NUM);
	ESP_LOGI(TAG, "I2C_MODE_MASTER  :  %d", I2C_MODE_MASTER);
	ESP_LOGI(TAG, "I2C_MASTER_SDA_IO:  %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "I2C_MASTER_SCL_IO:  %d", I2C_MASTER_SCL_IO);
	ESP_LOGI(TAG, "I2C_master_port  :  %d", i2c_master_port);
   	ESP_LOGI(TAG, "I2C_MASTER_FREQ_HZ  %d", I2C_MASTER_FREQ_HZ);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);

//*************// Verify that the I2C slave is working properly
    esp_err_t f_retval;                 /* changed */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP2_I2C_ADDR_PRIM << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    f_retval = i2c_master_cmd_begin(I2C_NUM_0, cmd, RTOS_DELAY_1SEC);
    if (f_retval != ESP_OK) {
        ESP_LOGI(TAG, "I2C slave NOT working or wrong I2C slave address - error (%i)", f_retval);
    }
    i2c_cmd_link_delete(cmd);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello from ESP32!");
    struct bmp2_data comp_data;
    
    if (i2c_master_init() == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C Initialization Success!");
        static uint8_t dev_addr;
        int8_t rslt = BMP2_OK;
        dev_addr = BMP2_I2C_ADDR_PRIM;
        bmp.read = bmp2_i2c_read;
        bmp.write = bmp2_i2c_write;
        bmp.intf = BMP2_I2C_INTF;        /* Holds the I2C device addr */
        bmp.intf_ptr = &dev_addr;
        ESP_LOGI(TAG, "Print &dev_addr %d", dev_addr);
        ESP_LOGI(TAG, "Print &dev_addr %d", *(int *)bmp.intf_ptr);
        bmp.delay_us = delay_ms;         /* Configure delay in microseconds */
        //bmp.power_mode = 
        rslt = bmp2_init(&bmp);
        ESP_LOGI(TAG, "BMP2 Initialization Success! RC=%d", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
        rslt = bmp2_get_config(&conf, &bmp);
        ESP_LOGI(TAG, "BMP2 get Config RC=%d", rslt);

        /* Configuring the over-sampling mode, filter coefficient and output data rate */
        conf.filter = BMP2_FILTER_OFF;

        /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
        conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

        /* Setting the output data rate */
        conf.odr = BMP2_ODR_250_MS;

        rslt = bmp2_set_config(&conf, &bmp);
        ESP_LOGI(TAG, "BMP2 set Config RC=%d", rslt);

        /* Set normal power mode */
        rslt = bmp2_set_power_mode(BMP2_POWERMODE_NORMAL, &conf, &bmp);
        ESP_LOGI(TAG, "BMP2 set power  RC=%d", rslt);

        while (1)
        {
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            rslt = bmp2_get_sensor_data(&comp_data, &bmp);
            ESP_LOGI(TAG, "Data-> Temp: %2.2f °C // Pres: %4.1f hPa", comp_data.temperature, comp_data.pressure/100);
        }
    }
}