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

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp32c3/rom/lldesc.h"
#include "esp32c3/rom/gpio.h"
#include "driver/gpio.h"
#include "gpio_sig_map.h"
#include "spi_struct.h"
#include "gdma_struct.h"
#include "interrupt_core0_reg.h"
#include "system_reg.h"
#include "lcd_cam.h"

static const char *TAG = "lcd_cam";

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4000)
#define LCD_CAM_DMA_MAX_NUM               (3) // Maximum number of DMA channels
#define DMA_CH0_INTR_SOURCE               (((INTERRUPT_CORE0_DMA_CH0_INT_MAP_REG - DR_REG_INTERRUPT_CORE0_BASE) / 4))

typedef struct {
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t dma_half_node_cnt;
    lldesc_t *dma;
    uint8_t  *dma_buffer;
    QueueHandle_t event_queue;
    uint8_t  width;
    bool swap_data;
} lcd_obj_t;

typedef struct {
    bool lcd_en;
    lcd_obj_t lcd;
    uint8_t dma_num;
    intr_handle_t dma_intr_handle;
} lcd_cam_obj_t;

static lcd_cam_obj_t *lcd_cam_obj = NULL;

static void IRAM_ATTR dma_isr(void *arg)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(GDMA.intr[lcd_cam_obj->dma_num].st) status = GDMA.intr[lcd_cam_obj->dma_num].st;
    if (status.val == 0) {
        return;
    }
    GDMA.intr[lcd_cam_obj->dma_num].clr.val = status.val;
    if (status.out_eof) {
        xQueueSendFromISR(lcd_cam_obj->lcd.event_queue, &status.val, &HPTaskAwoken);
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void lcd_dma_set_int(void)
{
    // Generate a data DMA linked list
    for (int x = 0; x < lcd_cam_obj->lcd.dma_node_cnt; x++) {
        lcd_cam_obj->lcd.dma[x].size = lcd_cam_obj->lcd.dma_node_buffer_size;
        lcd_cam_obj->lcd.dma[x].length = lcd_cam_obj->lcd.dma_node_buffer_size;
        lcd_cam_obj->lcd.dma[x].buf = (lcd_cam_obj->lcd.dma_buffer + lcd_cam_obj->lcd.dma_node_buffer_size * x);
        lcd_cam_obj->lcd.dma[x].eof = !((x + 1) % lcd_cam_obj->lcd.dma_half_node_cnt);
        lcd_cam_obj->lcd.dma[x].empty = &lcd_cam_obj->lcd.dma[(x + 1) % lcd_cam_obj->lcd.dma_node_cnt];
    }
    lcd_cam_obj->lcd.dma[lcd_cam_obj->lcd.dma_half_node_cnt - 1].empty = NULL;
    lcd_cam_obj->lcd.dma[lcd_cam_obj->lcd.dma_node_cnt - 1].empty = NULL; 
}

static void lcd_dma_set_left(int pos, size_t len)
{
    int end_pos = 0, size = 0;
    // Processing data length is an integer multiple of lcd_cam_obj->lcd.dma_node_buffer_size
    if (len % lcd_cam_obj->lcd.dma_node_buffer_size) {
        end_pos = (pos % 2) * lcd_cam_obj->lcd.dma_half_node_cnt + len / lcd_cam_obj->lcd.dma_node_buffer_size;
        size = len % lcd_cam_obj->lcd.dma_node_buffer_size;
    } else {
        end_pos = (pos % 2) * lcd_cam_obj->lcd.dma_half_node_cnt + len / lcd_cam_obj->lcd.dma_node_buffer_size - 1;
        size = lcd_cam_obj->lcd.dma_node_buffer_size;
    }
    // Process the tail node to make it a DMA tail
    lcd_cam_obj->lcd.dma[end_pos].size = size;
    lcd_cam_obj->lcd.dma[end_pos].length = size;
    lcd_cam_obj->lcd.dma[end_pos].eof = 1;
    lcd_cam_obj->lcd.dma[end_pos].empty = NULL;
}

static void spi_start(uint32_t addr, size_t len)
{
    while (GPSPI2.cmd.usr);
    GPSPI2.ms_dlen.ms_data_bitlen = len * 8 - 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.out_conf0.out_rst = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.out_conf0.out_rst = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.out_link.addr = addr;
    GDMA.channel[lcd_cam_obj->dma_num].out.out_link.start = 1;
    GPSPI2.cmd.update = 1;
    while (GPSPI2.cmd.update);
    GPSPI2.cmd.usr = 1;
}

static void spi_write_data(uint8_t *data, size_t len)
{
    int event  = 0;
    int x = 0, y = 0, left = 0, cnt = 0;
    if (len <= 0) {
        ESP_LOGE(TAG, "wrong len!");
        return;
    }
    lcd_dma_set_int();
    cnt = len / lcd_cam_obj->lcd.dma_half_buffer_size;
    // Start signal
    xQueueSend(lcd_cam_obj->lcd.event_queue, &event, 0);
    // Process a complete piece of data, ping-pong operation
    for (x = 0; x < cnt; x++) {
        uint8_t *out = lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf;
        uint8_t *in  = data;
        if (lcd_cam_obj->lcd.swap_data) {
            memcpy(out, in, lcd_cam_obj->lcd.dma_half_buffer_size);
        } else {
            for (y = 0; y < lcd_cam_obj->lcd.dma_half_buffer_size; y+=2) {
                out[y+1] = in[y+0];
                out[y+0] = in[y+1];
            }
        }
        data += lcd_cam_obj->lcd.dma_half_buffer_size;
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        spi_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, lcd_cam_obj->lcd.dma_half_buffer_size);
    }
    left = len % lcd_cam_obj->lcd.dma_half_buffer_size;
    // Process remaining incomplete segment data
    if (left) {
        uint8_t *out = lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf;
        uint8_t *in  = data;
        cnt = left - left % 2;
        if (cnt) {
            if (lcd_cam_obj->lcd.swap_data) {
                memcpy(out, in, cnt);
            } else {
                for (y = 0; y < cnt; y+=2) {
                    out[y+1] = in[y+0];
                    out[y+0] = in[y+1];
                }
            }
        }

        if (left % 2) {
            out[cnt] = in[cnt];
        }
        lcd_dma_set_left(x, left);
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        spi_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, left);
    }
    xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
}

static void lcd_swap_data(bool en) 
{
    lcd_cam_obj->lcd.swap_data = en;
}

static esp_err_t lcd_cam_config(lcd_cam_config_t *config)
{
    GDMA.channel[lcd_cam_obj->dma_num].out.out_conf0.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].in.in_conf0.mem_trans_en = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.out_conf1.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.out_conf1.out_check_owner = 0;
    GDMA.intr[lcd_cam_obj->dma_num].clr.val = ~0;
    GDMA.intr[lcd_cam_obj->dma_num].ena.val = 0;

    if (lcd_cam_obj->lcd_en) {
        if (config->lcd.width == 1) { // SPI mode
            GPSPI2.clk_gate.clk_en = 1;
            GPSPI2.clk_gate.mst_clk_active = 1;
            GPSPI2.clk_gate.mst_clk_sel = 1;

            int div = 2;
            if (config->lcd.fre >= 80000000) {
                GPSPI2.clock.clk_equ_sysclk = 1;
            } else {
                GPSPI2.clock.clk_equ_sysclk = 0;
                div = 80000000 / config->lcd.fre;
            }
            GPSPI2.slave.clk_mode = 0;
            GPSPI2.clock.clkdiv_pre = 1 - 1;
            GPSPI2.clock.clkcnt_n = div - 1;
            GPSPI2.clock.clkcnt_l = div - 1;
            GPSPI2.clock.clkcnt_h = ((div >> 1) - 1);
            
            GPSPI2.misc.ck_dis = 0;

            GPSPI2.user1.val = 0;
            GPSPI2.slave.val = 0;
            GPSPI2.misc.ck_idle_edge = 0;
            GPSPI2.user.ck_out_edge = 0;
            GPSPI2.ctrl.wr_bit_order = 0;
            GPSPI2.ctrl.rd_bit_order = 0;
            GPSPI2.user.val = 0;
            GPSPI2.user.cs_setup = 1;
            GPSPI2.user.cs_hold = 1;
            GPSPI2.user.usr_mosi = 1;
            GPSPI2.user.usr_mosi_highpart = 0;
            GPSPI2.dma_conf.val = 0;
            GPSPI2.dma_conf.dma_tx_ena = 1;
            GPSPI2.dma_conf.tx_seg_trans_clr_en = 1;
            GPSPI2.dma_conf.dma_seg_trans_en = 0;
            
            GPSPI2.dma_conf.dma_afifo_rst = 1;
            GPSPI2.dma_conf.dma_afifo_rst = 0;
            GPSPI2.dma_conf.buf_afifo_rst = 1;
            GPSPI2.dma_conf.buf_afifo_rst = 0;

            GPSPI2.cmd.update = 1;
            while (GPSPI2.cmd.update);
            GPSPI2.cmd.usr = 0;
        }

        GDMA.channel[lcd_cam_obj->dma_num].out.out_conf0.out_rst = 1;
        GDMA.channel[lcd_cam_obj->dma_num].out.out_conf0.out_rst = 0;
        GDMA.channel[lcd_cam_obj->dma_num].out.out_conf0.outdscr_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].out.out_conf0.out_data_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].out.out_peri_sel.sel = (config->lcd.width == 1) ? 0 : 0;
        GDMA.channel[lcd_cam_obj->dma_num].out.out_pri.tx_pri = 1;
        GDMA.intr[lcd_cam_obj->dma_num].ena.out_eof = 1;
    }

    return ESP_OK;
}

static esp_err_t lcd_set_pin(lcd_config_t *config)
{
    if (config->width == 1) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.clk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.clk, GPIO_FLOATING);
        gpio_matrix_out(config->pin.clk, FSPICLK_OUT_IDX, config->invert.clk, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[0]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.data[0], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.data[0], GPIO_FLOATING);
        gpio_matrix_out(config->pin.data[0], FSPID_OUT_IDX, config->invert.data[0], false);
    } else {
        ESP_LOGE(TAG, "lcd width wrong!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t lcd_dma_config(lcd_config_t *config) 
{
    int cnt = 0;
    if (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE % 2 != 0) {
         ESP_LOGE(TAG, "need 2-byte aligned data length");
         return ESP_FAIL;
    }
    if (config->width == 1) {
        if (config->max_dma_buffer_size > 65536) {
            config->max_dma_buffer_size = 65536;
        }
    }
    if (config->max_dma_buffer_size >= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * 2) {
        lcd_cam_obj->lcd.dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        for (cnt = 0; cnt < config->max_dma_buffer_size - 8; cnt++) { // Find a buffer size that can divide dma_size
            if ((config->max_dma_buffer_size - cnt) % (lcd_cam_obj->lcd.dma_node_buffer_size * 2) == 0) {
                break;
            }
        }
        lcd_cam_obj->lcd.dma_buffer_size = config->max_dma_buffer_size - cnt;
    } else {
        lcd_cam_obj->lcd.dma_node_buffer_size = config->max_dma_buffer_size / 2;
        lcd_cam_obj->lcd.dma_buffer_size = lcd_cam_obj->lcd.dma_node_buffer_size * 2;
    }
    
    lcd_cam_obj->lcd.dma_half_buffer_size = lcd_cam_obj->lcd.dma_buffer_size / 2;

    lcd_cam_obj->lcd.dma_node_cnt = (lcd_cam_obj->lcd.dma_buffer_size) / lcd_cam_obj->lcd.dma_node_buffer_size; // Number of DMA nodes
    lcd_cam_obj->lcd.dma_half_node_cnt = lcd_cam_obj->lcd.dma_node_cnt / 2;

    ESP_LOGI(TAG, "lcd_buffer_size: %d, lcd_dma_size: %d, lcd_dma_node_cnt: %d\n", lcd_cam_obj->lcd.dma_buffer_size, lcd_cam_obj->lcd.dma_node_buffer_size, lcd_cam_obj->lcd.dma_node_cnt);

    lcd_cam_obj->lcd.dma    = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->lcd.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_cam_obj->lcd.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->lcd.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
    return ESP_OK;
}

esp_err_t lcd_cam_deinit(lcd_cam_handle_t *handle)
{
    if (!lcd_cam_obj) {
        return ESP_FAIL;
    }

    if (lcd_cam_obj->lcd_en) {
        if (lcd_cam_obj->lcd.event_queue) {
            vQueueDelete(lcd_cam_obj->lcd.event_queue);
        }
        if (lcd_cam_obj->lcd.dma) {
            free(lcd_cam_obj->lcd.dma);
        }
        if (lcd_cam_obj->lcd.dma_buffer) {
            free(lcd_cam_obj->lcd.dma_buffer);
        }
    }
     
    if (lcd_cam_obj->dma_intr_handle) {
        esp_intr_free(lcd_cam_obj->dma_intr_handle);
    }   
    free(lcd_cam_obj);
    lcd_cam_obj = NULL;
    return ESP_OK;
}

esp_err_t lcd_cam_init(lcd_cam_handle_t *handle, lcd_cam_config_t *config)
{
    esp_err_t ret = ESP_OK;
    if (handle == NULL || config == NULL) {
        ESP_LOGE(TAG, "arg error\n");
        return ESP_ERR_INVALID_ARG;
    }
    lcd_cam_obj = (lcd_cam_obj_t *)heap_caps_calloc(1, sizeof(lcd_cam_obj_t), MALLOC_CAP_DMA);
    if (lcd_cam_obj == NULL) {
        ESP_LOGE(TAG, "lcd_cam object malloc error\n");
        return ESP_ERR_NO_MEM;
    }

    //Enable LCD_CAM periph

    if (config->lcd.width == 1) {
        if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN) == 0) {
            REG_CLR_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN);
            REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI2_CLK_EN);
            REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI2_RST);
            REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI2_RST);
        }
    }

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    }

    for (int x = 0; x < LCD_CAM_DMA_MAX_NUM; x++) {
        if (GDMA.channel[lcd_cam_obj->dma_num].out.out_link.start == 0x0) {
            lcd_cam_obj->dma_num = x;
            break;
        }
        if (x == LCD_CAM_DMA_MAX_NUM - 1) {
            ESP_LOGE(TAG, "DMA error\n");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }
    }

    lcd_cam_obj->lcd_en = config->lcd.en;
    ret |= lcd_cam_config(config);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd_cam config fail!");
        lcd_cam_deinit(handle);
        return ESP_FAIL;
    }

    if (lcd_cam_obj->lcd_en) {
        ret |= lcd_set_pin(&config->lcd);
        ret |= lcd_dma_config(&config->lcd);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "lcd config fail!");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }

        lcd_cam_obj->lcd.event_queue = xQueueCreate(1, sizeof(int));
        lcd_cam_obj->lcd.width = config->lcd.width;
        lcd_cam_obj->lcd.swap_data = config->lcd.swap_data;
        if (lcd_cam_obj->lcd.event_queue == NULL) {
            ESP_LOGE(TAG, "lcd config fail!");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }

        if (config->lcd.width == 1) {
            handle->lcd.write_data = spi_write_data;
        }
        
        handle->lcd.swap_data = lcd_swap_data;
        ESP_LOGI(TAG, "lcd init ok\n");
    }

    if (lcd_cam_obj->lcd_en) {
        ret |= esp_intr_alloc((DMA_CH0_INTR_SOURCE + lcd_cam_obj->dma_num), ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, dma_isr, NULL, &lcd_cam_obj->dma_intr_handle);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "lcd_cam intr alloc fail!");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}
