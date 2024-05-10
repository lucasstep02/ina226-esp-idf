
#include "esp_log.h"
#include "driver/i2c.h"

const char *TAG = "INA226-test";


#include "include/INA226.h"


static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = CONFIG_I2C_MASTER_NUM;

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
}



void app_main(void)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    INA226 ina;

    INA226_configure(&ina,
                     INA226_AVERAGES_16,
                     INA226_BUS_CONV_TIME_1100US,
                     INA226_SHUNT_CONV_TIME_1100US,
                     INA226_MODE_SHUNT_BUS_CONT);
    INA226_calibrate(&ina, 0.002f, 10);

    while(1) {
        ESP_LOGI(TAG, "Bus voltage: %f\tShunt current: %f", INA226_readBusVoltage(&ina), INA226_readShuntCurrent(&ina));
//        ESP_LOGI(TAG, "Bus power: %f", INA226_readBusPower(&ina));
//        ESP_LOGI(TAG, "Shunt voltage: %f", INA226_readShuntVoltage(&ina));
//        ESP_LOGI(TAG, "Shunt current: %f", INA226_readShuntCurrent(&ina));
//        ESP_LOGI(TAG, "\n");
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    vTaskDelay(2000 / portTICK_RATE_MS);

    ESP_ERROR_CHECK(i2c_driver_delete(CONFIG_I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
}
