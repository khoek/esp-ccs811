#pragma once

#include <driver/i2c.h>
#include <libi2c.h>

// The device HW_ID/VERSION of the CCS811 supported by this driver.
#define CCS811_HW_ID_DRIVER_SUPPORTED 0x81
#define CCS811_HW_VERSION_DRIVER_SUPPORTED 0x10

typedef enum ccs811_reg {
    CCS811_REG_STATUS = 0x00,
    CCS811_REG_MEAS_MODE = 0x01,
    CCS811_REG_ALG_RESULT_DATA = 0x02,
    CCS811_REG_RAW_DATA = 0x03,

    CCS811_REG_ENV_DATA = 0x05,

    CCS811_REG_THRESHOLDS = 0x10,
    CCS811_REG_BASELINE = 0x11,

    CCS811_REG_HW_ID = 0x20,
    CCS811_REG_HW_VERSION = 0x21,

    CCS811_REG_FW_BOOT_VERSION = 0x23,
    CCS811_REG_FW_APP_VERSION = 0x24,

    CCS811_REG_INTERNAL_STATE = 0xA0,

    CCS811_REG_ERROR_ID = 0xE0,

    CCS811_REG_BOOTLOADER_APP_ERASE = 0xF1,
    CCS811_REG_BOOTLOADER_APP_DATA = 0xF2,
    CCS811_REG_BOOTLOADER_APP_VERIFY = 0xF3,
    CCS811_REG_BOOTLOADER_APP_START = 0xF4,

    CCS811_REG_SW_RESET = 0xFF,
} ccs811_reg_t;

#define CCS811_STATUS_ERROR (1ULL << 0)
#define CCS811_STATUS_DATA_READY (1ULL << 3)
#define CCS811_STATUS_APP_VALID (1ULL << 4)
#define CCS811_STATUS_APP_VERIFY (1ULL << 5)
#define CCS811_STATUS_APP_ERASE (1ULL << 6)
#define CCS811_STATUS_FW_MODE (1ULL << 7)

#define CCS811_MEAS_MODE_DRIVE_MODE_0 (0b000ULL << 4)
#define CCS811_MEAS_MODE_DRIVE_MODE_1 (0b001ULL << 4)
#define CCS811_MEAS_MODE_DRIVE_MODE_2 (0b010ULL << 4)
#define CCS811_MEAS_MODE_DRIVE_MODE_3 (0b011ULL << 4)
#define CCS811_MEAS_MODE_DRIVE_MODE_4 (0b100ULL << 4)

#define MASK_CCS811_HW_VERSION_REVISION 0x0F
#define MASK_CCS811_HW_VERSION_MAJOR 0xF0

extern const uint8_t CCS811_SW_RESET_COOKIE[4];

typedef i2c_7bit_handle_t ccs811_handle_t;

// Register the CCS811 on the given I2C bus.
__result_use_check esp_err_t ccs811_init(i2c_port_t port, uint8_t addr,
                                         ccs811_handle_t *out_dev);

// Release the given handle.
void ccs811_destroy(ccs811_handle_t dev);

// Reset the device and read/set calibration data from internal memory.
__result_use_check esp_err_t ccs811_reset(ccs811_handle_t dev);

// Read a register over I2C.
__result_use_check esp_err_t ccs811_reg_read(ccs811_handle_t dev,
                                             ccs811_reg_t reg, uint8_t *data,
                                             size_t count);

// Write a register over I2C.
__result_use_check esp_err_t ccs811_reg_write(ccs811_handle_t dev,
                                              ccs811_reg_t reg,
                                              const uint8_t *data,
                                              size_t count);

// While in the bootloader mode (e.g. after a reset), verify the stored app is
// valid and start it running.
__result_use_check esp_err_t ccs811_start_app(ccs811_handle_t dev);

// Read and decode the ALG_RESULT_DATA register.
__result_use_check esp_err_t ccs811_read_alg_result_data(
    ccs811_handle_t dev, uint16_t *out_eco2_ppm, uint16_t *out_etvoc_ppb,
    uint8_t *out_status, uint8_t *out_error_id, uint16_t *out_raw_data);

// Encode a temperature value to be passed to the CCS811.
void ccs811_encode_temp(double temp_c, uint16_t *raw_temp);

// Encode a humidity value to be passed to the CCS811.
void ccs811_encode_hum(double rel_humidity, uint16_t *raw_hum);
