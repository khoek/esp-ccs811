#include <driver/i2c.h>
#include <esp_log.h>
#include <libi2c.h>

#include "esp-ccs811.h"

static const char *TAG = "ccs811";

const uint8_t CCS811_SW_RESET_COOKIE[4] = {0x11, 0xE5, 0x72, 0x8A};

esp_err_t ccs811_init(i2c_port_t port, uint8_t addr, ccs811_handle_t *out_dev) {
    ccs811_handle_t dev;
    i2c_7bit_init(port, addr, &dev);

    // As per spec max power-on start up time is 20ms.
    vTaskDelay(1 + (20 / portTICK_PERIOD_MS));

    uint8_t reg_hw_id;
    esp_err_t ret = i2c_7bit_reg_read(dev, CCS811_REG_HW_ID, 1, &reg_hw_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed, are I2C pin numbers/address correct?");
        goto ccs811_init_fail;
    }

    if (reg_hw_id != CCS811_HW_ID_DRIVER_SUPPORTED) {
        ESP_LOGE(TAG, "unsupported HW ID (0x%02X), have you specified the address of another device?", reg_hw_id);
        goto ccs811_init_fail;
    }

    uint8_t reg_hw_version;
    ccs811_reg_read(dev, CCS811_REG_HW_VERSION, 1, &reg_hw_version);

    if ((reg_hw_version & MASK_CCS811_HW_VERSION_MAJOR) != CCS811_HW_VERSION_DRIVER_SUPPORTED) {
        ESP_LOGE(TAG, "unsupported HW version (0x%02X), have you specified the address of another device?", reg_hw_version);
        goto ccs811_init_fail;
    }

    uint8_t reg_fw_boot_version[2];
    ccs811_reg_read(dev, CCS811_REG_FW_BOOT_VERSION, 2, reg_fw_boot_version);

    uint8_t reg_fw_app_version[2];
    ccs811_reg_read(dev, CCS811_REG_FW_APP_VERSION, 2, reg_fw_app_version);

    ESP_LOGI(TAG, "hw(rev=%d), fw(boot=%d.%d.%d,app=%d.%d.%d)",
             reg_hw_version & MASK_CCS811_HW_VERSION_REVISION,
             (reg_fw_boot_version[0] & 0xF0) >> 4,
             (reg_fw_boot_version[0] & 0x0F) >> 0,
             reg_fw_boot_version[1],
             (reg_fw_app_version[0] & 0xF0) >> 4,
             (reg_fw_app_version[0] & 0x0F) >> 0,
             reg_fw_app_version[1]);

    ccs811_reset(dev);

    *out_dev = dev;
    return ESP_OK;

ccs811_init_fail:
    i2c_7bit_destroy(dev);
    return ret;
}

void ccs811_destroy(ccs811_handle_t dev) {
    i2c_7bit_destroy(dev);
}

void ccs811_reset(ccs811_handle_t dev) {
    ccs811_reg_write(dev, CCS811_REG_SW_RESET, 4, CCS811_SW_RESET_COOKIE);

    // Delay for 2ms as per spec.
    vTaskDelay(1 + (2 / portTICK_PERIOD_MS));

    uint8_t reg_error_id;
    ccs811_reg_read(dev, CCS811_REG_ERROR_ID, 1, &reg_error_id);

    if (reg_error_id) {
        ESP_LOGE(TAG, "error id register not cleared after reset! (0x%02X)", reg_error_id);
        abort();
    }
}

void ccs811_reg_read(ccs811_handle_t dev, ccs811_reg_t reg, size_t count, uint8_t *data) {
    ESP_ERROR_CHECK(i2c_7bit_reg_read(dev, reg, count, data));
}

void ccs811_reg_write(ccs811_handle_t dev, ccs811_reg_t reg, size_t count, const uint8_t *data) {
    ESP_ERROR_CHECK(i2c_7bit_reg_write(dev, reg, count, data));
}

void ccs811_start_app(ccs811_handle_t dev) {
    uint8_t reg_status;
    ccs811_reg_read(dev, CCS811_REG_STATUS, 1, &reg_status);

    if (!(reg_status & CCS811_STATUS_APP_VALID)) {
        ESP_LOGE(TAG, "stored app not valid!");
        abort();
    }

    if (reg_status & CCS811_STATUS_FW_MODE) {
        ESP_LOGE(TAG, "firmware is already in application mode, reset the device first!");
        abort();
    }

    if (reg_status & CCS811_STATUS_ERROR) {
        uint8_t reg_error_id;
        ccs811_reg_read(dev, CCS811_REG_ERROR_ID, 1, &reg_error_id);

        ESP_LOGE(TAG, "error reported! (0x%02X)", reg_error_id);
        abort();
    }

    ccs811_reg_write(dev, CCS811_REG_BOOTLOADER_APP_START, 0, NULL);

    // Delay for 1ms as per spec.
    vTaskDelay(1 + (100 / portTICK_PERIOD_MS));

    ccs811_reg_read(dev, CCS811_REG_STATUS, 1, &reg_status);

    if (!(reg_status & CCS811_STATUS_FW_MODE)) {
        ESP_LOGE(TAG, "firmware did not enter application mode!");
        abort();
    }
}

void ccs811_read_alg_result_data(ccs811_handle_t dev, uint16_t *out_ppm_eco2, uint16_t *out_ppb_tvoc, uint8_t *out_status, uint8_t *out_error_id, uint16_t *out_raw_data) {
    uint8_t reg_alg_result_data[8];
    ccs811_reg_read(dev, CCS811_REG_ALG_RESULT_DATA, 8, reg_alg_result_data);

    uint16_t ppm_eco2 = (((uint16_t) reg_alg_result_data[0]) << 8) | (((uint16_t) reg_alg_result_data[1]) << 0);
    uint16_t ppb_tvoc = (((uint16_t) reg_alg_result_data[2]) << 8) | (((uint16_t) reg_alg_result_data[3]) << 0);
    uint8_t status = reg_alg_result_data[4];
    uint8_t error_id = reg_alg_result_data[5];
    uint16_t raw_data = (((uint16_t) reg_alg_result_data[6]) << 8) | (((uint16_t) reg_alg_result_data[7]) << 0);

    if (out_ppm_eco2) {
        *out_ppm_eco2 = ppm_eco2;
    }

    if (out_ppb_tvoc) {
        *out_ppb_tvoc = ppb_tvoc;
    }

    if (out_status) {
        *out_status = status;
    }

    if (out_error_id) {
        *out_error_id = error_id;
    }

    if (out_raw_data) {
        *out_raw_data = raw_data;
    }
}