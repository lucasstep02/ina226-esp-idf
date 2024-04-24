//
// Created by bohm on 4/23/24.
//


#include "include/INA226.h"

extern char *TAG;

static esp_err_t INA226_register_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(CONFIG_I2C_MASTER_NUM, INA226_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

static esp_err_t INA226_register_write_2_bytes(uint8_t reg_addr, uint16_t value) {
    int ret;
    uint8_t write_buf[3] = {reg_addr, value >> 8, value & 0xFF};

    ret = i2c_master_write_to_device(CONFIG_I2C_MASTER_NUM, INA226_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

static esp_err_t INA226_register_write_byte(uint8_t reg_addr, uint8_t value) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, value};

    ret = i2c_master_write_to_device(CONFIG_I2C_MASTER_NUM, INA226_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

void INA226_configure(INA226 *ina, ina226_averages_t avg, ina226_busConvTime_t busConvTime, ina226_shuntConvTime_t shuntConvTime, ina226_mode_t mode) {
    uint16_t config = 0;// 16679;

    config |= (avg << 9 | busConvTime << 6 | shuntConvTime << 3 | mode);

    ina->vBusMax = 36;
    ina->vShuntMax = 0.08192f;

    //ESP_ERROR_CHECK(INA226_register_write_byte(INA226_REG_CONFIG, config));
    ESP_ERROR_CHECK(INA226_register_write_2_bytes(INA226_REG_CONFIG, config));

    ESP_LOGI(TAG, "Configured with config %d", config);

    uint8_t data[2];
    INA226_register_read(INA226_REG_CONFIG, data, 2);
    int16_t cfg = (data[0] << 8 | data[1]);
    ESP_LOGI(TAG, "Checking config value %d", cfg);
}

void INA226_calibrate(INA226 *ina, float rShuntValue, float iMaxExpected) {
    ESP_LOGI(TAG, "Going to calibrate...");
    ina->rShunt = rShuntValue;

    /* */
    float iMaxPossible = ina->vShuntMax / ina->rShunt;

    float minimumLSB = iMaxExpected / 32767;

    ina->currentLSB = (uint16_t)(minimumLSB * 100000000);
    ina->currentLSB /= 100000000;
    ina->currentLSB /= 0.0001;
    ina->currentLSB = ceil(ina->currentLSB);
    ina->currentLSB *= 0.0001;

    ina->powerLSB = ina->currentLSB * 25;

    uint16_t calibrationValue = (uint16_t)((0.00512) / (ina->currentLSB * ina->rShunt));
    /* */

    /* * /
    ina->currentLSB = iMaxExpected / 32768.0f;
    uint16_t calibrationValue = (uint16_t) (0.00512 / (ina->currentLSB * rShuntValue));
    / * */

    INA226_register_write_2_bytes(INA226_REG_CALIBRATION, calibrationValue);
    ESP_LOGI(TAG, "currentLSB is %f, calibration value is %d", ina->currentLSB, calibrationValue);
    ESP_LOGI(TAG, "Calibrated...");
}

float INA226_readBusVoltage(INA226 *ina){
    uint8_t data[2];
    INA226_register_read(INA226_REG_BUSVOLTAGE, data, 2);
    int16_t voltage = (data[0] << 8 | data[1]);

    return (float) voltage * 0.00125f;
}

float INA226_readBusPower(INA226 *ina) {
    uint8_t data[2];
    INA226_register_read(INA226_REG_POWER, data, 2);
    int16_t power = (data[0] << 8 | data[1]);
    return ((float) power * ina->powerLSB);
}

float INA226_readShuntCurrent(INA226 *ina) {
    uint8_t data[2];
    INA226_register_read(INA226_REG_CURRENT, data, 2);
    int current = (data[0] << 8 | data[1]);
    // ESP_LOGI(TAG, "Raw current value read: %d", current);
    // For some reason it needs to be divided by 2...
    return ((float) current * ina->currentLSB / 2.0f);
}

float INA226_readShuntVoltage(INA226 *ina) {
    uint8_t data[2];
    INA226_register_read(INA226_REG_SHUNTVOLTAGE, data, 2);
    int16_t voltage = (data[0] << 8 | data[1]);

    return (float) voltage * 2.5e-6f; // fixed to 2.5 uV
}
