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
#include "esp32s3/rom/lldesc.h"
#include "esp32s3/rom/gpio.h"
#include "esp32s3/rom/cache.h"
#include "driver/gpio.h"
#include "spi_struct.h"
#include "gdma_struct.h"
#include "interrupt_core0_reg.h"
#include "system_reg.h"
#include "lcd_cam_struct.h"
#include "lcd.h"

static const char *TAG = "lcd";

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4000)
#define LCD_CAM_DMA_MAX_NUM               (5) // Maximum number of DMA channels
#define LCD_CAM_INTR_SOURCE               (((INTERRUPT_CORE0_LCD_CAM_INT_MAP_REG - DR_REG_INTERRUPT_CORE0_BASE) / 4))
#define DMA_OUT_CH0_INTR_SOURCE           (((INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_REG - DR_REG_INTERRUPT_CORE0_BASE) / 4))

#define LCD_DEFAULT_CTRL() (lcd_ctrl_t) { \
    .bus = {.width = 8, .fre = 10000000}, \
    .res = {.x = 320, .y = 240}, \
    .video_timing_sync = {.hsync = 0, .vsync = 0}, \
    .video_timing_h = {.hbp = 0, .hfp = 0}, \
    .video_timing_v = {.vbp = 0, .vfp = 0}, \
    .ctr = {.video_mode_en = 0, .frame_buffer_num = 0, .max_dma_buffer_size = 32000, .swap_data = 0, .pix_bytes = 2} \
}

typedef struct lcd_obj_struct lcd_obj_t;

struct lcd_obj_struct {
    lcd_handle_t *handle;
    QueueHandle_t event_queue;
    lldesc_t **dma;
    uint8_t dma_num;
    int fb_pos;
    intr_handle_t lcd_intr_handle;
    intr_handle_t dma_out_intr_handle;
    lcd_obj_t *next;
};

lcd_obj_t *lcd_obj = NULL;

static void IRAM_ATTR lcd_isr(void *arg)
{
    typeof(LCD_CAM.lc_dma_int_st) status = LCD_CAM.lc_dma_int_st;
    BaseType_t HPTaskAwoken = pdFALSE;
    if (status.val == 0) {
        return;
    }
    LCD_CAM.lc_dma_int_clr.val = status.val;

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR dma_isr(void *arg)
{
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(GDMA.channel[lcd_obj->dma_num].out.int_st) out_status = GDMA.channel[lcd_obj->dma_num].out.int_st;
    if (out_status.val != 0) {
        GDMA.channel[lcd_obj->dma_num].out.int_clr.val = out_status.val;
        if (out_status.out_eof) {
            xQueueSendFromISR(lcd_obj->event_queue, &out_status.val, &HPTaskAwoken);
        }
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void lcd_start(uint32_t addr, size_t len)
{
    while (LCD_CAM.lcd_user.lcd_start);
    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_user.lcd_reset = 0;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
    while (GDMA.channel[lcd_obj->dma_num].out.link.start);
    GDMA.channel[lcd_obj->dma_num].out.conf0.val = 0;
    GDMA.channel[lcd_obj->dma_num].out.conf1.val = 0;
    GDMA.channel[lcd_obj->dma_num].out.int_clr.val = ~0;
    GDMA.channel[lcd_obj->dma_num].out.int_ena.val = 0;
    GDMA.channel[lcd_obj->dma_num].out.conf0.out_rst = 1;
    GDMA.channel[lcd_obj->dma_num].out.conf0.out_rst = 0;
    GDMA.channel[lcd_obj->dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[lcd_obj->dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[lcd_obj->dma_num].out.conf1.out_ext_mem_bk_size = 2;
    GDMA.sram_size[lcd_obj->dma_num].out.out_size = 14; // fifo 128 Bytes
    GDMA.channel[lcd_obj->dma_num].out.peri_sel.sel = 5;
    GDMA.channel[lcd_obj->dma_num].out.pri.tx_pri = 1;
    GDMA.channel[lcd_obj->dma_num].out.int_ena.out_eof = 1;
    GDMA.channel[lcd_obj->dma_num].out.link.addr = addr;
    GDMA.channel[lcd_obj->dma_num].out.link.start = 1;
    if (len > 16) {
        while (GDMA.channel[lcd_obj->dma_num].out.outfifo_status.outfifo_cnt_l3 < 16);
    } else {
        esp_rom_delay_us(1);
    }
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
}

static void lcd_write_data(uint8_t *data, size_t len)
{
    lcd_ctrl_t *ctrl = lcd_obj->handle->ctrl;
    lcd_port_t *port = lcd_obj->handle->port;
    int event  = 0;
    int x = 0, y = 0, left = 0, cnt = 0;
    if (len <= 0) {
        ESP_LOGE(TAG, "wrong len!");
        return;
    }
    Cache_WriteBack_Addr(data, len);
    if (ctrl->ctr.swap_data) {
        LCD_CAM.lcd_user.lcd_8bits_order = 0;
    }
    if (len % LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE) {
        cnt = len / LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        left = len % LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
    } else {
        cnt = len / LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1;
        left = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
    }
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc((cnt + 1) * sizeof(lldesc_t), MALLOC_CAP_DMA);
    // lldesc_t *dma = lcd_obj->lcd.dma;
    for (x = 0; x < cnt; x++) {
        memset(&dma[x], 0, sizeof(lldesc_t));
        dma[x].size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        dma[x].length = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        dma[x].buf = (data + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
        dma[x].eof = 0;
        dma[x].empty = &dma[(x + 1)];
    }
    if (left) {
        memset(&dma[x], 0, sizeof(lldesc_t));
        dma[x].size = left;
        dma[x].length = left;
        dma[x].buf = (data + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
        dma[x].eof = 1;
        dma[x].empty = NULL;
    }
    xQueueSend(lcd_obj->event_queue, &event, 0);
    xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
    lcd_start(((uint32_t)&dma[0]) & 0xfffff, len);
    xQueueReceive(lcd_obj->event_queue, (void *)&event, portMAX_DELAY);
    free(dma);
    LCD_CAM.lcd_user.lcd_8bits_order = 1;
}

#include "esp_wifi.h"

static void lcd_frame_write(int pos) 
{
    lcd_obj->fb_pos = pos;
}

static void lcd_task(void *arg)
{
    lcd_ctrl_t *ctrl = lcd_obj->handle->ctrl;
    lcd_port_t *port = lcd_obj->handle->port;
    int len = ctrl->res.x * ctrl->res.y * ctrl->ctr.pix_bytes;
    int event  = 0;
    uint32_t ticks_now = 0, ticks_last = 0;
    struct timeval now;  
    while (1) {
        while (lcd_obj->fb_pos == -1) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } 
        gettimeofday(&now, NULL);
        ticks_last = now.tv_sec * 1000 + now.tv_usec / 1000;
        if (port->m.fb.start) {
            port->m.fb.start(NULL);
        }
        LCD_CAM.lcd_user.lcd_8bits_order = 0;
        lcd_start(((uint32_t)lcd_obj->dma[lcd_obj->fb_pos]) & 0xfffff, len);
        if (xQueueReceive(lcd_obj->event_queue, (void *)&event, 40 / portTICK_PERIOD_MS) != pdTRUE) {
            ESP_LOGE(TAG, "error frame\n");
            GDMA.channel[lcd_obj->dma_num].out.conf0.out_rst = 1;
            GDMA.channel[lcd_obj->dma_num].out.conf0.out_rst = 0;
        }
        LCD_CAM.lcd_user.lcd_8bits_order = 1;
        if (port->m.fb.end) {
            port->m.fb.end(NULL);
        }
        gettimeofday(&now, NULL);
        ticks_now = now.tv_sec * 1000 + now.tv_usec / 1000;
        if (ticks_now - ticks_last > 0) {
            printf("fps: %.2f\n", 1000.0 / (int)(ticks_now - ticks_last));
        }
    }
}

static esp_err_t lcd_config(lcd_handle_t *handle)
{
    lcd_ctrl_t *ctrl = handle->ctrl;
    GDMA.channel[lcd_obj->dma_num].out.conf0.val = 0;
    GDMA.channel[lcd_obj->dma_num].out.conf1.val = 0;
    GDMA.channel[lcd_obj->dma_num].out.int_clr.val = ~0;
    GDMA.channel[lcd_obj->dma_num].out.int_ena.val = 0;

    LCD_CAM.lcd_clock.val = 0;
    LCD_CAM.lcd_clock.clk_en = 1;
    LCD_CAM.lcd_clock.lcd_clk_sel = 3;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
    LCD_CAM.lcd_clock.lcd_clkm_div_a = 10;
    LCD_CAM.lcd_clock.lcd_clkm_div_num = 2;
    LCD_CAM.lcd_clock.lcd_clkcnt_n = 80000000 / ctrl->bus.fre - 1; 
    LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
    LCD_CAM.lcd_clock.lcd_ck_idle_edge = 1; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_clock.lcd_ck_out_edge = 0; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_user.val = 0;
    LCD_CAM.lcd_user.lcd_2byte_en = (ctrl->bus.width == 16) ? 1 : 0;
    LCD_CAM.lcd_user.lcd_byte_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order = 0;
    LCD_CAM.lcd_user.lcd_cmd = 0;		// FSM CMD phase
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;	// 2 cycle command
    LCD_CAM.lcd_user.lcd_dout = 1;	// FSM DOUT phase
    LCD_CAM.lcd_user.lcd_dout_cyclelen = 2 - 1;
    LCD_CAM.lcd_user.lcd_8bits_order = 1;
    LCD_CAM.lcd_user.lcd_always_out_en = 1;
    LCD_CAM.lcd_misc.val = 0;
    LCD_CAM.lcd_misc.lcd_afifo_threshold_num = 11;
    LCD_CAM.lcd_misc.lcd_vfk_cyclelen = 3;
    LCD_CAM.lcd_misc.lcd_vbk_cyclelen = 0;
    LCD_CAM.lcd_misc.lcd_cd_idle_edge = 1;	// idle edge of CD is set to 0
    LCD_CAM.lcd_misc.lcd_cd_cmd_set = 1;
    LCD_CAM.lcd_misc.lcd_cd_dummy_set = 0;
    LCD_CAM.lcd_misc.lcd_cd_data_set = 0;	// change when DOUT start
    LCD_CAM.lcd_misc.lcd_bk_en = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
    LCD_CAM.lcd_ctrl.val = 0;
    LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
    LCD_CAM.lcd_cmd_val = 0;	// write command
    LCD_CAM.lcd_user.lcd_update = 1;

    GDMA.channel[lcd_obj->dma_num].out.conf0.out_rst = 1;
    GDMA.channel[lcd_obj->dma_num].out.conf0.out_rst = 0;
    GDMA.channel[lcd_obj->dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[lcd_obj->dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[lcd_obj->dma_num].out.peri_sel.sel = (ctrl->bus.width == 1) ? 1 : 5;
    GDMA.channel[lcd_obj->dma_num].out.pri.tx_pri = 1;
    GDMA.channel[lcd_obj->dma_num].out.int_ena.out_eof = 1;

    return ESP_OK;
}

static esp_err_t lcd_set_pin(lcd_handle_t *handle)
{
    lcd_ctrl_t *ctrl = handle->ctrl;
    if (ctrl->bus.width == 1) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_clk.clk], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_clk.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_clk.clk, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_clk.clk, SPI3_CLK_OUT_IDX, ctrl->pin_clk.clk_inv, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_data[0].data], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_data[0].data, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_data[0].data, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_data[0].data, SPI3_D_OUT_IDX, ctrl->pin_data[0].data_inv, false);
    } else if (ctrl->bus.width <= LCD_DATA_MAX_WIDTH && ctrl->bus.width % 8 == 0){
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_clk.clk], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_clk.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_clk.clk, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_clk.clk, LCD_PCLK_IDX, ctrl->pin_clk.clk_inv, false);

        for(int i = 0; i < ctrl->bus.width; i++) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_data[i].data], PIN_FUNC_GPIO);
            gpio_set_direction(ctrl->pin_data[i].data, GPIO_MODE_OUTPUT);
            gpio_set_pull_mode(ctrl->pin_data[i].data, GPIO_FLOATING);
            gpio_matrix_out(ctrl->pin_data[i].data, LCD_DATA_OUT0_IDX + i, ctrl->pin_data[i].data_inv, false);
        }
    } else {
        ESP_LOGE(TAG, "lcd width wrong!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t lcd_dma_config(lcd_handle_t *handle) 
{
    lcd_ctrl_t *ctrl = handle->ctrl;
#if 0
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
        lcd_obj->lcd.dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        for (cnt = 0; cnt < config->max_dma_buffer_size - 8; cnt++) { // Find a buffer size that can divide dma_size
            if ((config->max_dma_buffer_size - cnt) % (lcd_obj->lcd.dma_node_buffer_size * 2) == 0) {
                break;
            }
        }
        lcd_obj->lcd.dma_buffer_size = config->max_dma_buffer_size - cnt;
    } else {
        lcd_obj->lcd.dma_node_buffer_size = config->max_dma_buffer_size / 2;
        lcd_obj->lcd.dma_buffer_size = lcd_obj->lcd.dma_node_buffer_size * 2;
    }
    
    lcd_obj->lcd.dma_half_buffer_size = lcd_obj->lcd.dma_buffer_size / 2;

    lcd_obj->lcd.dma_node_cnt = (lcd_obj->lcd.dma_buffer_size) / lcd_obj->lcd.dma_node_buffer_size; // Number of DMA nodes
    lcd_obj->lcd.dma_half_node_cnt = lcd_obj->lcd.dma_node_cnt / 2;

    ESP_LOGI(TAG, "lcd_buffer_size: %d, lcd_dma_size: %d, lcd_dma_node_cnt: %d\n", lcd_obj->lcd.dma_buffer_size, lcd_obj->lcd.dma_node_buffer_size, lcd_obj->lcd.dma_node_cnt);

    lcd_obj->lcd.dma    = (lldesc_t *)heap_caps_malloc(lcd_obj->lcd.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_obj->lcd.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_obj->lcd.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);
#else
    int x = 0, y = 0, left = 0, cnt = 0;
    int len = ctrl->res.x * ctrl->res.y * ctrl->ctr.pix_bytes;

    if (len % LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE) {
        cnt = len / LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        left = len % LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
    } else {
        cnt = len / LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1;
        left = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
    }
    lcd_obj->dma = (lldesc_t **)heap_caps_malloc((ctrl->ctr.frame_buffer_num) * sizeof(lldesc_t *), MALLOC_CAP_DMA);
    for (y = 0; y < ctrl->ctr.frame_buffer_num; y++) {
        lcd_obj->dma[y] = (lldesc_t *)heap_caps_malloc((cnt + 1) * sizeof(lldesc_t), MALLOC_CAP_DMA);
        lldesc_t *dma = lcd_obj->dma[y];
        for (x = 0; x < cnt; x++) {
            dma[x].size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
            dma[x].length = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
            dma[x].buf = (ctrl->frame_buffer[y].addr + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
            dma[x].eof = 0;
            dma[x].empty = &dma[(x + 1)];
        }
        if (left) {
            dma[x].size = left;
            dma[x].length = left;
            dma[x].buf = (ctrl->frame_buffer[y].addr + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
            dma[x].eof = 1;
            dma[x].empty = NULL;
        }
    }
#endif
    return ESP_OK;
}

static esp_err_t lcd_run(lcd_handle_t *handle)
{
    esp_err_t ret = ESP_OK;

    //Enable LCD_CAM periph

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_LCD_CAM_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_LCD_CAM_RST);
    }

    if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN) == 0) {
        REG_CLR_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_CLK_EN1_REG, SYSTEM_DMA_CLK_EN);
        REG_SET_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
        REG_CLR_BIT(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_DMA_RST);
    }

    for (int x = LCD_CAM_DMA_MAX_NUM - 1; x >= 0; x++) {
        if (GDMA.channel[x].out.link.start == 0x0) {
            lcd_obj->dma_num = x;
            break;
        }
        if (x == LCD_CAM_DMA_MAX_NUM - 1) {
            ESP_LOGE(TAG, "DMA error\n");
            lcd_remove(handle);
            return ESP_FAIL;
        }
    }

    ret |= lcd_config(handle);
    ret |= lcd_set_pin(handle);
    ret |= lcd_dma_config(handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd config fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }

    lcd_obj->event_queue = xQueueCreate(1, sizeof(int));
    if (lcd_obj->event_queue == NULL) {
        ESP_LOGE(TAG, "lcd config fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }

    ret |= esp_intr_alloc((DMA_OUT_CH0_INTR_SOURCE + lcd_obj->dma_num), ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, dma_isr, NULL, &lcd_obj->dma_out_intr_handle);
    ret |= esp_intr_alloc(LCD_CAM_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, lcd_isr, NULL, &lcd_obj->lcd_intr_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd intr alloc fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }

    xTaskCreate(lcd_task, "lcd_task", 2048, NULL, configMAX_PRIORITIES, NULL);

    ESP_LOGI(TAG, "lcd run\n");

    return ESP_OK;
}

esp_err_t lcd_create(lcd_handle_t *handle)
{
    esp_err_t ret = ESP_OK;
    if (handle == NULL) {
        ESP_LOGE(TAG, "arg error\n");
        return ESP_ERR_INVALID_ARG;
    }
    lcd_obj = (lcd_obj_t *)heap_caps_calloc(1, sizeof(lcd_obj_t), MALLOC_CAP_DEFAULT);
    if (lcd_obj == NULL) {
        ESP_LOGE(TAG, "lcd_obj malloc error\n");
        return ESP_ERR_NO_MEM;
    }
    handle->obj = (void *)lcd_obj;
    lcd_obj->handle = handle;

    lcd_obj->handle->ctrl = (lcd_ctrl_t *)heap_caps_calloc(1, sizeof(lcd_ctrl_t), MALLOC_CAP_DEFAULT);
    lcd_obj->handle->port = (lcd_port_t *)heap_caps_calloc(1, sizeof(lcd_port_t), MALLOC_CAP_DEFAULT);
    lcd_obj->event_queue = xQueueCreate(1, sizeof(int));
    if (lcd_obj->handle->ctrl == NULL || \
        lcd_obj->handle->port == NULL || \
        lcd_obj->event_queue == NULL) {
        ESP_LOGE(TAG, "lcd create fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }
    lcd_obj->fb_pos = -1;

    memcpy(lcd_obj->handle->ctrl, &LCD_DEFAULT_CTRL(), sizeof(lcd_ctrl_t));

    lcd_obj->handle->port->s.ctr.run = lcd_run;
    lcd_obj->handle->port->s.data.write = lcd_write_data;
    lcd_obj->handle->port->s.fb.write = lcd_frame_write;

    return ESP_OK;
}

esp_err_t lcd_remove(lcd_handle_t *handle)
{
    if (!lcd_obj) {
        return ESP_FAIL;
    }

    if (lcd_obj->event_queue) {
        vQueueDelete(lcd_obj->event_queue);
    }
    if (lcd_obj->dma) {
        free(lcd_obj->dma);
    }
    
    free(lcd_obj);
    lcd_obj = NULL;
    return ESP_OK;
}
