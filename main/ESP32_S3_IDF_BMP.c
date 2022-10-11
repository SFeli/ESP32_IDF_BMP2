/* 
    IDF für BME2 - Treiber von Bosch
    mit I2C - Standard-Bibliothek ab Version 4.4.
    Für ESP32 aber auch ESP S3 mit idf.py ... im Terminal
*/

#include "stdio.h"
#include "stdlib.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "bmp2.h"
#include "bmp2_defs.h"

// ursprünglich aus Hauptprogramm
#define I2C_MASTER_SCL_IO           18        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           19        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0         /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define RTOS_DELAY_1SEC          ( 1 * 1000 / portTICK_PERIOD_MS)

#define BMP280_SENSOR_ADDR          0x76        /*!< Primary address of the BMP280 ensor */
#define BOSCH__REGISTER_CHIPID      0xD0        /*!< Register bei den Boschsensoren zur ID */
#define BOSCH_RESET_REGISTER        0xE0        /*!< Register addresses of the power managment register */
#define BOSCH_RESET_VALUE           0xB6

/* #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE  */
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_VERBOSE

// void delay_ms(uint32_t period_ms);

int8_t rslt;
struct bmp2_dev bmp;
struct bmp2_config conf;
static const char *TAG = "ESP32-BMP2-IDF  ";

/**
 *  @brief Function for reading the sensor's registers through I2C bus.
 *  @param[in]  reg_addr   : Register address.
 *  @param[out] reg_data   : Pointer to the data buffer to store the read data.
 *  @param[in]  len        : No of bytes to read.
 *  @param[in]  intf_ptr   : Pointer Interface pointer
 *  @return Status of execution 0 .. success 1 Failure
 */
static BMP2_INTF_RET_TYPE bmp2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    char* TAGR = "ESP32-BMP2-Read ";
    switch (reg_addr)
    {
    case 0xd0:
        ESP_LOGD(TAGR, "Register to read  %x -> Chip-ID", reg_addr);
        break;
    case 0xf4:
        ESP_LOGD(TAGR, "Register to read  %x -> controls", reg_addr);
        break;
    case 0xf7:
        ESP_LOGD(TAGR, "Register to read  %x -> burst-read of values", reg_addr);
        break;
    case 0x88:
        ESP_LOGD(TAGR, "Register to read  %x -> trimming parameters", reg_addr);
        break;
    default:
        ESP_LOGD(TAGR, "Register to read  %x -> not known", reg_addr);
        break;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    rslt = i2c_master_write_read_device(I2C_MASTER_NUM, *(int *)intf_ptr, &reg_addr, 1, reg_data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "bmp2_i2c_read failed!");
    }
    return rslt;
}

/**
 *  @brief Function for writing the sensor's registers through I2C bus.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] len      : No of bytes to write.
 *  @param[in] intf_ptr : Pointer Interface pointer
 *  @return Status of execution
 *  @retval BMP2_INTF_RET_SUCCESS -> Success.
 *  @retval != BMP2_INTF_RET_SUCCESS -> Failure.
 */
static BMP2_INTF_RET_TYPE bmp2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t  len, void *intf_ptr)
{
    char* TAGW = "ESP32-BMP2-Write";
    switch (reg_addr)
    {
    case 0xe0:
        ESP_LOGD(TAGW, "Register to write %x -> reset", reg_addr);
        break;
    case 0xf4:
        ESP_LOGD(TAGW, "Register to write %x -> controls", reg_addr);
        break;
    default:
        ESP_LOGD(TAGW, "Register to write %x -> not known", reg_addr);
        break;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);      // 100 / 10
    uint8_t prim_adr =  *(int *)intf_ptr;
    uint8_t write_buf[2] = {reg_addr, (uint8_t)*reg_data};
    rslt = i2c_master_write_to_device(I2C_MASTER_NUM, prim_adr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_write_to_device failed!");
    }
    return rslt;
}

/*!
 * Delay function 
 */
void delay_ms(uint32_t period_ms)
{
    //vTaskDelay(period_ms / portTICK_PERIOD_MS);
    printf("Function delay 1\n");
    vTaskDelay(200);
    printf("Function delay 2\n");
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_cmd_handle_t cmd;
	ESP_LOGI(TAG, "I2C_MASTER_PORT  :  %d", I2C_NUM_0);
	ESP_LOGI(TAG, "I2C_MASTER_MODE  :  %d (1..master or 0..slave)", I2C_MODE_MASTER);
	ESP_LOGI(TAG, "I2C_MASTER_SDA_IO:  %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "I2C_MASTER_SCL_IO:  %d", I2C_MASTER_SCL_IO);
   	ESP_LOGI(TAG, "I2C_MASTER_FREQ_HZ  %d", I2C_MASTER_FREQ_HZ);

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    rslt = i2c_param_config(I2C_NUM_0, &i2c_conf);
    if (rslt != ESP_OK)
        ESP_LOGE(TAG, "Error in i2c-configuration RC=%d", rslt);
    rslt = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (rslt != ESP_OK)
        ESP_LOGE(TAG, "Error in i2c-driver RC=%d", rslt);
    else
        ESP_LOGI(TAG, "I2C - initialisation RC=%d", rslt);
//*************// Verify if I2C slave is working properly
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP2_I2C_ADDR_PRIM << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    rslt = i2c_master_cmd_begin(I2C_NUM_0, cmd, RTOS_DELAY_1SEC);
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "I2C slave NOT working or wrong I2C slave address - RC=%d", rslt);
    }
    i2c_cmd_link_delete(cmd);
    return rslt;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello from ESP32!");
    esp_log_level_set("*", ESP_LOG_INFO);
    struct bmp2_data comp_data;
    
    if (i2c_master_init() == ESP_OK)
    {
        // Link I2C - Library to BMP2 - calls
        static uint8_t dev_addr;
        //int8_t rslt = BMP2_OK;
        dev_addr  = BMP2_I2C_ADDR_PRIM;
        bmp.read  = bmp2_i2c_read;
        bmp.write = bmp2_i2c_write;
        bmp.intf = BMP2_I2C_INTF;        /* Holds the I2C device addr */
        bmp.intf_ptr = &dev_addr;
        ESP_LOGD(TAG, "Print &dev_addr   %x", dev_addr);
        ESP_LOGD(TAG, "Print bmp.inf_ptr %x", *(int *)bmp.intf_ptr);
        bmp.delay_us = delay_ms;         /* Configure delay in microseconds */

        rslt = bmp2_init(&bmp);
        ESP_LOGI(TAG, "BMP2 initialization RC=%d", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
        rslt = bmp2_get_config(&conf, &bmp);
        ESP_LOGI(TAG, "BMP2 get config RC=%d", rslt);

        /* Configuring the over-sampling mode, filter coefficient and output data rate */
        conf.filter = BMP2_FILTER_OFF;

        /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
        conf.os_mode = BMP2_OS_MODE_HIGH_RESOLUTION;

        /* Setting the output data rate */
        conf.odr = BMP2_ODR_250_MS;

        rslt = bmp2_set_config(&conf, &bmp);
        ESP_LOGI(TAG, "BMP2 set config RC=%d", rslt);

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
