// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "esp_types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_DATA_MAX_WIDTH (1)  /*!< Maximum width of LCD data bus */

/**
 * @brief Structure to store config information of lcd driver
 */
typedef struct {
    bool en;                             /*!< Enable switch */
    uint8_t  width;                      /*!< Data bus width (maximum LCD_DATA_MAX_WIDTH) 1: 1bit, 8: 8bit, 16: 16bit */
    uint32_t fre;                        /*!< CLK clock frequency */
    struct {
        int8_t clk;                      /*!< CLK output pin */
        int8_t data[LCD_DATA_MAX_WIDTH]; /*!< DATA output pin */
    } pin;                               /*!< Pin configuration */
    struct {
        bool clk;                        /*!< CLK output signal inversion */
        bool data[LCD_DATA_MAX_WIDTH];   /*!< DATA output signal inversion */
    } invert;                            /*!< Signal inversion configuration */
    uint32_t max_dma_buffer_size;        /*!< DMA maximum memory usage, memory must be able to accessed by DMA */
    bool swap_data;                      /*!< Two-byte data exchange */
} lcd_config_t;

/**
 * @brief Structure to store config information of lcd_cam driver
 */
typedef struct {
    lcd_config_t lcd; /*!< lcd config */
} lcd_cam_config_t;

/**
 * @brief Structure to store handle information of lcd driver
 */
typedef struct {
    /**
     * @brief Write data
     *
     * @param data Data pointer
     * @param len Write data length, unit: byte
     */
    void (*write_data) (uint8_t *data, size_t len);

    /**
     * @brief Two-byte data exchange
     *
     * @param en enable, 1: enable, 0: disable
     */
    void (*swap_data) (bool en);
} lcd_handle_t;

/**
 * @brief Structure to store handle information of lcd_cam driver
 */
typedef struct {
    lcd_handle_t lcd; /*!< lcd handle */
} lcd_cam_handle_t;

/**
 * @brief Uninitialize the lcd_cam module
 *
 * @param handle Provide handle pointer to release resources
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Uninitialize fail
 */
esp_err_t lcd_cam_deinit(lcd_cam_handle_t *handle);

/**
 * @brief Initialize the lcd_cam module
 *
 * @param handle Return handle pointer after successful initialization - see lcd_cam_handle_t struct
 * @param config Configurations - see lcd_cam_config_t struct
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM No memory to initialize lcd_cam
 *     - ESP_FAIL Initialize fail
 */
esp_err_t lcd_cam_init(lcd_cam_handle_t *handle, lcd_cam_config_t *config);

#ifdef __cplusplus
}
#endif