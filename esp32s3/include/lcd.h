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

#define LCD_DATA_MAX_WIDTH (16)  /*!< Maximum width of LCD data bus */
#define LCD_DATA_MAX_FB    (8) 

typedef struct lcd_handle_struct lcd_handle_t;

/**
 * @brief Structure to store config information of lcd driver
 */
typedef struct {
    union {
        struct {
            uint32_t width:      5; /*!< Data bus width (maximum LCD_DATA_MAX_WIDTH) 1: 1bit, 8: 8bit, 16: 16bit */
            uint32_t fre:       27;        /*!< CLK clock frequency */
        };
        uint32_t val;
    } bus;
    union {
        struct {
            uint32_t x:      16; /*!< Data bus width (maximum LCD_DATA_MAX_WIDTH) 1: 1bit, 8: 8bit, 16: 16bit */
            uint32_t y:      16; /*!< CLK clock frequency */
        };
        uint32_t val;
    } res;
    union {
        struct {
            uint32_t hsync:  16; /*!< DMA maximum memory usage, memory must be able to accessed by DMA */
            uint32_t vsync:  16; /*!< Two-byte data exchange */
        };
        uint32_t val;
    } video_timing_sync;
    union {
        struct {
            uint32_t hbp:  16; /*!< DMA maximum memory usage, memory must be able to accessed by DMA */
            uint32_t hfp:  16; /*!< Two-byte data exchange */
        };
        uint32_t val;
    } video_timing_h;
    union {
        struct {
            uint32_t vbp:  16; /*!< DMA maximum memory usage, memory must be able to accessed by DMA */
            uint32_t vfp:  16; /*!< Two-byte data exchange */
        };
        uint32_t val;
    } video_timing_v;
    union {
        struct {
            uint32_t video_mode_en:         1;
            uint32_t frame_buffer_num:      4;
            uint32_t max_dma_buffer_size:  20; /*!< DMA maximum memory usage, memory must be able to accessed by DMA */
            uint32_t swap_data:             1; /*!< Two-byte data exchange */
            uint32_t pix_bytes:             3;
            uint32_t reserved29:            3; /*reserved*/
        };
        uint32_t val;
    } ctr;
    union {
        struct {
            uint32_t clk:        16; /*!< CLK output pin */
            uint32_t clk_inv:     1; /*!< CLK output signal inversion */
            uint32_t reserved17: 15; /*reserved*/
        };
        uint32_t val;
    } pin_clk;
    union {
        struct {
            uint32_t data:        16; /*!< DATA output pin */
            uint32_t data_inv:     1; /*!< DATA output signal inversion */
            uint32_t reserved17:  15; /*reserved*/
        };
        uint32_t val;
    } pin_data[LCD_DATA_MAX_WIDTH];
    union {
        struct {
            uint32_t addr:        32; /*!< DATA output pin */
        };
        uint32_t val;
    } frame_buffer[LCD_DATA_MAX_FB];
} lcd_ctrl_t;

/**
 * @brief Structure to store handle information of lcd driver
 */
typedef struct {
    struct {
        struct {
            esp_err_t (*run)(lcd_handle_t *handle);
            esp_err_t (*stop)(lcd_handle_t *handle);
        } ctr;

        struct {
            void (*write) (int pos);
        } fb;

        struct {
            void (*write) (uint8_t *data, size_t len);
        } data;
    } s; // slave

    struct {
        struct {
            void (*start)(void *arg);
            void (*end)(void *arg);
        } fb;

        struct {
            void (*start)(void *arg);
            void (*end)(void *arg);
        } data;
    } m; // master
} lcd_port_t; 

/**
 * @brief Structure to store handle information of lcd_fb driver
 */
struct lcd_handle_struct {
    lcd_ctrl_t *ctrl; // 寄存器
    lcd_port_t *port; // 驱动器
    void *obj;
}; /*!< lcd handle */

esp_err_t lcd_create(lcd_handle_t *handle);

esp_err_t lcd_remove(lcd_handle_t *handle);

#ifdef __cplusplus
}
#endif