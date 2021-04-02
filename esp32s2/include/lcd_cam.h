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

#define LCD_DATA_MAX_WIDTH (24)  /*!< Maximum width of LCD data bus */
#define CAM_DATA_MAX_WIDTH (16)  /*!< Maximum width of CAM data bus */

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
 * @brief Structure to store config information of cam driver
 */
typedef struct {
    bool en;                             /*!< Enable switch */
    uint8_t  width;                      /*!< Data bus width (maximum CAM_DATA_MAX_WIDTH) 8: 8bit, 16: 16bit */
    uint32_t fre;                        /*!< XCLK frequency */
    struct {
        int8_t xclk;                     /*!< XCLK output pin */
        int8_t pclk;                     /*!< PCLK input pin */
        int8_t vsync;                    /*!< Vertical sync input pin */
        int8_t href;                     /*!< Horizontal enable input pin */
        int8_t data[CAM_DATA_MAX_WIDTH]; /*!< DATA input pin */
    } pin;
    struct {
        bool xclk;                       /*!< XCLK output signal inversion */
        bool pclk;                       /*!< PCLK input signal inversion */
        bool vsync;                      /*!< Vertical sync input signal inversion */
        bool href;                       /*!< Horizontal enable input signal inversion */
        bool data[CAM_DATA_MAX_WIDTH];   /*!< DATA input signal inversion */
    } invert;                            /*!< Signal inversion configuration */
    union {
        struct {
            uint32_t jpeg:   1;          /*!< JPEG mode */
        };
        uint32_t val;
    } mode;                              /*!< Mode selection */
    uint32_t max_dma_buffer_size;        /*!< DMA maximum memory usage, memory must be able to accessed by DMA */
    uint32_t recv_size;                  /*!< Receive data length */
    uint32_t frame_cnt;                  /*!< Number of buffered frames */
    uint32_t frame_caps;                 /*!< frame memory capabilities, MALLOC_CAP_SPIRAM, MALLOC_CAP_INTERNAL, etc. */
    uint32_t task_stack;                 /*!< Data processing task stack depth */
    uint8_t  task_pri;                   /*!< Data processing task priority */
    bool swap_data;                      /*!< Two-byte data exchange */
} cam_config_t;

/**
 * @brief Structure to store config information of lcd_cam driver
 */
typedef struct {
    lcd_config_t lcd; /*!< lcd config */
    cam_config_t cam; /*!< cam config */
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
 * @brief Structure to store handle information of cam driver
 */
typedef struct {
    /**
     * @brief cam starts working
     */
    void (*start)(void);

    /**
     * @brief cam stopped working
     */
    void (*stop)(void);

    /**
     * @brief Get frame data
     *
     * @param buffer_p Provide frame header pointer
     *
     * @return The actual data length obtained, unit: byte
     */
    size_t (*take)(uint8_t **buffer_p);

    /**
     * @brief Return frame data
     *
     * @param buffer Return frame data header, used to release memory
     */
    void (*give)(uint8_t *buffer);
} cam_handle_t;

/**
 * @brief Structure to store handle information of lcd_cam driver
 */
typedef struct {
    lcd_handle_t lcd; /*!< lcd handle */
    cam_handle_t cam; /*!< cam handle */
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