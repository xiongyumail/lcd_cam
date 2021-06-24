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
#include "driver/gpio.h"
#include "spi_struct.h"
#include "gdma_struct.h"
#include "interrupt_core0_reg.h"
#include "system_reg.h"
#include "lcd_cam_struct.h"
#include "lcd_cam.h"

static const char *TAG = "lcd_cam";

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4000)
#define LCD_CAM_DMA_MAX_NUM               (5) // Maximum number of DMA channels
#define LCD_CAM_INTR_SOURCE               (((INTERRUPT_CORE0_LCD_CAM_INT_MAP_REG - DR_REG_INTERRUPT_CORE0_BASE) / 4))
#define DMA_IN_CH0_INTR_SOURCE            (((INTERRUPT_CORE0_DMA_IN_CH0_INT_MAP_REG - DR_REG_INTERRUPT_CORE0_BASE) / 4))
#define DMA_OUT_CH0_INTR_SOURCE            (((INTERRUPT_CORE0_DMA_OUT_CH0_INT_MAP_REG - DR_REG_INTERRUPT_CORE0_BASE) / 4))

typedef enum {
    CAM_IN_SUC_EOF_EVENT = 0,
    CAM_VSYNC_EVENT
} cam_event_t;

typedef struct {
    uint8_t *frame_buffer;
    size_t len;
} frame_buffer_event_t;

typedef enum {
    CAM_STATE_IDLE = 0,
    CAM_STATE_READ_BUF = 1,
} cam_state_t;

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
    uint32_t dma_buffer_size;
    uint32_t dma_half_buffer_size;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;
    uint32_t frame_copy_cnt;
    uint32_t frame_cnt;
    uint32_t recv_size;
    lldesc_t *dma;
    uint8_t  *dma_buffer;
    uint8_t *frame_buffer;
    uint8_t *frame_en;
    QueueHandle_t event_queue;
    QueueHandle_t frame_buffer_queue;
    TaskHandle_t task_handle;
    uint8_t jpeg_mode;
    uint8_t vsync_pin;
    uint8_t vsync_invert;
    bool swap_data;
} cam_obj_t;

typedef struct {
    bool lcd_en;
    bool cam_en;
    lcd_obj_t lcd;
    cam_obj_t cam;
    uint8_t dma_num;
    intr_handle_t lcd_cam_intr_handle;
    intr_handle_t dma_in_intr_handle;
    intr_handle_t dma_out_intr_handle;
} lcd_cam_obj_t;

static lcd_cam_obj_t *lcd_cam_obj = NULL;

static void IRAM_ATTR lcd_cam_isr(void *arg)
{
    cam_event_t cam_event = {0};
    typeof(LCD_CAM.lc_dma_int_st) status = LCD_CAM.lc_dma_int_st;
    BaseType_t HPTaskAwoken = pdFALSE;
    if (status.val == 0) {
        return;
    }
    LCD_CAM.lc_dma_int_clr.val = status.val;
    if (status.cam_vsync) {
        cam_event = CAM_VSYNC_EVENT;
        xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR dma_isr(void *arg)
{
    cam_event_t cam_event = {0};
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(GDMA.channel[lcd_cam_obj->dma_num].in.int_st) in_status = GDMA.channel[lcd_cam_obj->dma_num].in.int_st;
    if (in_status.val != 0) {
        GDMA.channel[lcd_cam_obj->dma_num].in.int_clr.val = in_status.val;
        if (in_status.in_suc_eof) {
            cam_event = CAM_IN_SUC_EOF_EVENT;
            xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
        }
    }
    typeof(GDMA.channel[lcd_cam_obj->dma_num].out.int_st) out_status = GDMA.channel[lcd_cam_obj->dma_num].out.int_st;
    if (out_status.val != 0) {
        GDMA.channel[lcd_cam_obj->dma_num].out.int_clr.val = out_status.val;
        if (out_status.out_eof) {
            xQueueSendFromISR(lcd_cam_obj->lcd.event_queue, &out_status.val, &HPTaskAwoken);
        }
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

static void lcd_start(uint32_t addr, size_t len)
{
    while (LCD_CAM.lcd_user.lcd_start);
    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_user.lcd_reset = 0;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
    while (GDMA.channel[lcd_cam_obj->dma_num].out.link.start);
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf1.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_clr.val = ~0;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_ena.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_rst = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_rst = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.peri_sel.sel = 5;
    GDMA.channel[lcd_cam_obj->dma_num].out.pri.tx_pri = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_ena.out_eof = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.link.addr = addr;
    GDMA.channel[lcd_cam_obj->dma_num].out.link.start = 1;
    esp_rom_delay_us(1);
    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
}

static void lcd_write_data(uint8_t *data, size_t len)
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
            for (y = 0; y < lcd_cam_obj->lcd.dma_half_buffer_size; y+=2) {
                out[y+1] = in[y+0];
                out[y+0] = in[y+1];
            }
        } else {
            memcpy(out, in, lcd_cam_obj->lcd.dma_half_buffer_size);
        }
        data += lcd_cam_obj->lcd.dma_half_buffer_size;
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        lcd_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, lcd_cam_obj->lcd.dma_half_buffer_size);
    }
    left = len % lcd_cam_obj->lcd.dma_half_buffer_size;
    // Process remaining incomplete segment data
    if (left) {
        uint8_t *out = lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf;
        uint8_t *in  = data;
        cnt = left - left % 2;
        if (cnt) {
            if (lcd_cam_obj->lcd.swap_data) {
                for (y = 0; y < cnt; y+=2) {
                    out[y+1] = in[y+0];
                    out[y+0] = in[y+1];
                }
            } else {
                memcpy(out, in, cnt);
            }
        }

        if (left % 2) {
            out[cnt] = in[cnt];
        }
        lcd_dma_set_left(x, left);
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        lcd_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, left);
    }
    xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
}

static void spi_start(uint32_t addr, size_t len)
{
    while (GPSPI3.cmd.usr);
    GPSPI3.ms_dlen.ms_data_bitlen = len * 8 - 1;
    while (GDMA.channel[lcd_cam_obj->dma_num].out.link.start);
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf1.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_clr.val = ~0;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_ena.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_rst = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_rst = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.peri_sel.sel = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.pri.tx_pri = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_ena.out_eof = 1;
    GDMA.channel[lcd_cam_obj->dma_num].out.link.addr = addr;
    GDMA.channel[lcd_cam_obj->dma_num].out.link.start = 1;
    esp_rom_delay_us(1);
    GPSPI3.cmd.update = 1;
    while (GPSPI3.cmd.update);
    GPSPI3.cmd.usr = 1;
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
    GDMA.channel[lcd_cam_obj->dma_num].out.conf0.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.conf1.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].in.conf0.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].in.conf1.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_clr.val = ~0;
    GDMA.channel[lcd_cam_obj->dma_num].out.int_ena.val = 0;
    GDMA.channel[lcd_cam_obj->dma_num].in.int_clr.val = ~0;
    GDMA.channel[lcd_cam_obj->dma_num].in.int_ena.val = 0;

    if (lcd_cam_obj->lcd_en) {
        if (config->lcd.width == 1) { // SPI mode
            GPSPI3.clk_gate.clk_en = 1;
            GPSPI3.clk_gate.mst_clk_active = 1;
            GPSPI3.clk_gate.mst_clk_sel = 1;

            int div = 2;
            if (config->lcd.fre >= 80000000) {
                GPSPI3.clock.clk_equ_sysclk = 1;
            } else {
                GPSPI3.clock.clk_equ_sysclk = 0;
                div = 80000000 / config->lcd.fre;
            }
            GPSPI3.slave.clk_mode = 0;
            GPSPI3.clock.clkdiv_pre = 1 - 1;
            GPSPI3.clock.clkcnt_n = div - 1;
            GPSPI3.clock.clkcnt_l = div - 1;
            GPSPI3.clock.clkcnt_h = ((div >> 1) - 1);
            
            GPSPI3.misc.ck_dis = 0;

            GPSPI3.user1.val = 0;
            GPSPI3.slave.val = 0;
            GPSPI3.misc.ck_idle_edge = 0;
            GPSPI3.user.ck_out_edge = 0;
            GPSPI3.ctrl.wr_bit_order = 0;
            GPSPI3.ctrl.rd_bit_order = 0;
            GPSPI3.user.val = 0;
            GPSPI3.user.cs_setup = 1;
            GPSPI3.user.cs_hold = 1;
            GPSPI3.user.usr_mosi = 1;
            GPSPI3.user.usr_mosi_highpart = 0;
            GPSPI3.dma_conf.val = 0;
            GPSPI3.dma_conf.dma_tx_ena = 1;
            GPSPI3.dma_conf.tx_seg_trans_clr_en = 1;
            GPSPI3.dma_conf.dma_seg_trans_en = 0;
            
            GPSPI3.dma_conf.dma_afifo_rst = 1;
            GPSPI3.dma_conf.dma_afifo_rst = 0;
            GPSPI3.dma_conf.buf_afifo_rst = 1;
            GPSPI3.dma_conf.buf_afifo_rst = 0;

            GPSPI3.cmd.update = 1;
            while (GPSPI3.cmd.update);
            GPSPI3.cmd.usr = 0;
        } else {
            LCD_CAM.lcd_clock.val = 0;
            LCD_CAM.lcd_clock.clk_en = 1;
            LCD_CAM.lcd_clock.lcd_clk_sel = 3;
            LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
            LCD_CAM.lcd_clock.lcd_clkm_div_a = 10;
            LCD_CAM.lcd_clock.lcd_clkm_div_num = 2;
            LCD_CAM.lcd_clock.lcd_clkcnt_n = 80000000 / config->lcd.fre - 1;
            LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
            LCD_CAM.lcd_clock.lcd_ck_idle_edge = 1; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
            LCD_CAM.lcd_clock.lcd_ck_out_edge = 0; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
            LCD_CAM.lcd_user.val = 0;
            LCD_CAM.lcd_user.lcd_2byte_en = (config->lcd.width == 16) ? 1 : 0;
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
        }

        GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_rst = 1;
        GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_rst = 0;
        GDMA.channel[lcd_cam_obj->dma_num].out.conf0.outdscr_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].out.conf0.out_data_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].out.peri_sel.sel = (config->lcd.width == 1) ? 1 : 5;
        GDMA.channel[lcd_cam_obj->dma_num].out.pri.tx_pri = 1;
        GDMA.channel[lcd_cam_obj->dma_num].out.int_ena.out_eof = 1;
    }

    if (lcd_cam_obj->cam_en) {
        LCD_CAM.cam_ctrl.val = 0;
        LCD_CAM.cam_ctrl.cam_clkm_div_b = 0;
        LCD_CAM.cam_ctrl.cam_clkm_div_a = 10;
        LCD_CAM.cam_ctrl.cam_clkm_div_num = 160000000 / config->cam.fre;
        LCD_CAM.cam_ctrl.cam_clk_sel = 3;
        LCD_CAM.cam_ctrl.cam_stop_en = 0;
        LCD_CAM.cam_ctrl.cam_vsync_filter_thres = 7 - 1; // Filter by LCD_CAM clock
        LCD_CAM.cam_ctrl.cam_update = 0;
        LCD_CAM.cam_ctrl.cam_byte_order = 0;
        LCD_CAM.cam_ctrl.cam_bit_order = 0; 
        LCD_CAM.cam_ctrl.cam_line_int_en = 0;
        LCD_CAM.cam_ctrl.cam_vs_eof_en = 0; // in_suc_eof interrupt generation method
        LCD_CAM.cam_ctrl1.val = 0;
        LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1; // Cannot be assigned to 0, and it is easy to overflow
        LCD_CAM.cam_ctrl1.cam_line_int_num = 1  - 1; // The number of hsyncs that generate hs interrupts
        LCD_CAM.cam_ctrl1.cam_clk_inv = 0;
        LCD_CAM.cam_ctrl1.cam_vsync_filter_en = 1;
        LCD_CAM.cam_ctrl1.cam_2byte_en = 0;
        LCD_CAM.cam_ctrl1.cam_de_inv = 0;
        LCD_CAM.cam_ctrl1.cam_hsync_inv = 0;
        LCD_CAM.cam_ctrl1.cam_vsync_inv = 0;
        LCD_CAM.cam_ctrl1.cam_vh_de_mode_en = 0;

        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.in_rst = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.in_rst = 0;
        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.indscr_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.in_data_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.peri_sel.sel = 5;
        GDMA.channel[lcd_cam_obj->dma_num].in.pri.rx_pri = 1;

        LCD_CAM.cam_ctrl.cam_update = 1;
        LCD_CAM.cam_ctrl1.cam_start = 1;
    }
    return ESP_OK;
}

static esp_err_t lcd_set_pin(lcd_config_t *config)
{
    if (config->width == 1) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.clk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.clk, GPIO_FLOATING);
        gpio_matrix_out(config->pin.clk, SPI3_CLK_OUT_IDX, config->invert.clk, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[0]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.data[0], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.data[0], GPIO_FLOATING);
        gpio_matrix_out(config->pin.data[0], SPI3_D_OUT_IDX, config->invert.data[0], false);
    } else if (config->width <= LCD_DATA_MAX_WIDTH && config->width % 8 == 0){
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.clk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.clk, GPIO_FLOATING);
        gpio_matrix_out(config->pin.clk, LCD_PCLK_IDX, config->invert.clk, false);

        for(int i = 0; i < config->width; i++) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[i]], PIN_FUNC_GPIO);
            gpio_set_direction(config->pin.data[i], GPIO_MODE_OUTPUT);
            gpio_set_pull_mode(config->pin.data[i], GPIO_FLOATING);
            gpio_matrix_out(config->pin.data[i], LCD_DATA_OUT0_IDX + i, config->invert.data[i], false);
        }
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

static void cam_vsync_intr_enable(bool en)
{
    LCD_CAM.lc_dma_int_clr.cam_vsync = 1;
    if (en) {
        LCD_CAM.lc_dma_int_ena.cam_vsync = 1;
    } else {
        LCD_CAM.lc_dma_int_ena.cam_vsync = 0;
    }
}

static void cam_dma_stop(void)
{
    if (GDMA.channel[lcd_cam_obj->dma_num].in.int_ena.in_suc_eof == 1) {
        GDMA.channel[lcd_cam_obj->dma_num].in.int_ena.in_suc_eof = 0;
        GDMA.channel[lcd_cam_obj->dma_num].in.int_clr.in_suc_eof = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.link.stop = 1;
        LCD_CAM.cam_ctrl.cam_update = 1;
    }
}

static void cam_dma_start(void)
{
    if (GDMA.channel[lcd_cam_obj->dma_num].in.int_ena.in_suc_eof == 0) {
        LCD_CAM.cam_ctrl1.cam_start = 0;
        GDMA.channel[lcd_cam_obj->dma_num].in.int_clr.in_suc_eof = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.int_ena.in_suc_eof = 1;
        LCD_CAM.cam_ctrl1.cam_reset = 1;
        LCD_CAM.cam_ctrl1.cam_reset = 0;
        LCD_CAM.cam_ctrl1.cam_afifo_reset = 1;
        LCD_CAM.cam_ctrl1.cam_afifo_reset = 0;
        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.in_rst = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.in_rst = 0;
        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.indscr_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.conf0.in_data_burst_en = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.peri_sel.sel = 5;
        GDMA.channel[lcd_cam_obj->dma_num].in.pri.rx_pri = 1;
        GDMA.channel[lcd_cam_obj->dma_num].in.link.start = 1;
        LCD_CAM.cam_ctrl.cam_update = 1;
        LCD_CAM.cam_ctrl1.cam_start = 1;
        if(lcd_cam_obj->cam.jpeg_mode) {
            // Vsync the first frame manually
            gpio_matrix_in(lcd_cam_obj->cam.vsync_pin, CAM_V_SYNC_IDX, !lcd_cam_obj->cam.vsync_invert);
            gpio_matrix_in(lcd_cam_obj->cam.vsync_pin, CAM_V_SYNC_IDX, lcd_cam_obj->cam.vsync_invert);
        }
    }
}

static void cam_stop(void)
{
    cam_vsync_intr_enable(false);
    cam_dma_stop();
}

static void cam_start(void)
{
    cam_vsync_intr_enable(true);
}

static void cam_memcpy(uint8_t *out, uint8_t *in, size_t len) 
{
    if (lcd_cam_obj->cam.swap_data) {
        int cnt = len - len % 2;
        for (int x = 0; x < cnt; x += 2) {
            out[x + 1] = in[x + 0];
            out[x + 0] = in[x + 1];
        }
        if (len % 2) {
            out[cnt] = in[cnt];
        }
    } else {
        memcpy(out, in, len);
    }  
}

// Copy fram from DMA dma_buffer to fram dma_buffer
static void cam_task(void *arg)
{
    int cnt = 0;
    int frame_pos = 0;
    int state = CAM_STATE_IDLE;
    cam_event_t cam_event = {0};
    frame_buffer_event_t frame_buffer_event = {0};
    xQueueReset(lcd_cam_obj->cam.event_queue);
    while (1) {
        xQueueReceive(lcd_cam_obj->cam.event_queue, (void *)&cam_event, portMAX_DELAY);
        switch (state) {
            case CAM_STATE_IDLE: {
                if (cam_event == CAM_VSYNC_EVENT) { 
                    for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
                        if (lcd_cam_obj->cam.frame_en[x]) {
                            frame_pos = x;
                            cam_dma_start();
                            cam_vsync_intr_enable(false);
                            state = CAM_STATE_READ_BUF;
                            break;
                        }
                    }
                    cnt = 0;
                }
            }
            break;

            case CAM_STATE_READ_BUF: {
                if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                    if (cnt == 0) {
                        cam_vsync_intr_enable(true); // Need to start cam to receive the first buf data and then turn on the vsync interrupt
                    }
                    cam_memcpy(&lcd_cam_obj->cam.frame_buffer[(frame_pos * lcd_cam_obj->cam.recv_size) + cnt * lcd_cam_obj->cam.dma_half_buffer_size], &lcd_cam_obj->cam.dma_buffer[(cnt % 2) * lcd_cam_obj->cam.dma_half_buffer_size], lcd_cam_obj->cam.dma_half_buffer_size);

                    if(lcd_cam_obj->cam.jpeg_mode) {
                        if (lcd_cam_obj->cam.frame_en[frame_pos] == 0) {
                            cam_dma_stop();
                        }
                    } else {
                        if(cnt == lcd_cam_obj->cam.frame_copy_cnt - 1) {
                            lcd_cam_obj->cam.frame_en[frame_pos] = 0;
                        }
                    }

                    if(lcd_cam_obj->cam.frame_en[frame_pos] == 0) {
                        state = CAM_STATE_IDLE;
                        frame_buffer_event.frame_buffer = &lcd_cam_obj->cam.frame_buffer[frame_pos * lcd_cam_obj->cam.recv_size];
                        frame_buffer_event.len = (cnt + 1) * lcd_cam_obj->cam.dma_half_buffer_size;
                        if(xQueueSend(lcd_cam_obj->cam.frame_buffer_queue, (void *)&frame_buffer_event, 0) != pdTRUE) {
                            lcd_cam_obj->cam.frame_en[frame_pos] = 1;
                        }
                    } else {
                        cnt++;
                    }
                } else if (cam_event == CAM_VSYNC_EVENT) {
                    if(lcd_cam_obj->cam.jpeg_mode) {
                        lcd_cam_obj->cam.frame_en[frame_pos] = 0;
                    }
                }
            }
            break;
        }
    }
}

static size_t cam_take(uint8_t **buffer_p)
{
    frame_buffer_event_t frame_buffer_event;
    xQueueReceive(lcd_cam_obj->cam.frame_buffer_queue, (void *)&frame_buffer_event, portMAX_DELAY);
    *buffer_p = frame_buffer_event.frame_buffer;
    return frame_buffer_event.len;
}

static void cam_give(uint8_t *dma_buffer)
{
    for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
        if (&lcd_cam_obj->cam.frame_buffer[x * lcd_cam_obj->cam.recv_size] == dma_buffer) {
            lcd_cam_obj->cam.frame_en[x] = 1;
            break;
        }
    }
}

static esp_err_t cam_set_pin(cam_config_t *config)
{
    if (config->width <= CAM_DATA_MAX_WIDTH && config->width % 8 == 0) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.href], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.href, GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.href, GPIO_FLOATING);
        gpio_matrix_in(config->pin.href, CAM_H_ENABLE_IDX, config->invert.href);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.vsync], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.vsync, GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.vsync, GPIO_FLOATING);
        gpio_matrix_in(config->pin.vsync, CAM_V_SYNC_IDX, config->invert.vsync);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.pclk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.pclk, GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.pclk, GPIO_FLOATING);
        gpio_matrix_in(config->pin.pclk, CAM_PCLK_IDX, config->invert.pclk);

        for(int i = 0; i < config->width; i++) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[i]], PIN_FUNC_GPIO);
            gpio_set_direction(config->pin.data[i], GPIO_MODE_INPUT);
            gpio_set_pull_mode(config->pin.data[i], GPIO_FLOATING);
            gpio_matrix_in(config->pin.data[i], CAM_DATA_IN0_IDX + i, config->invert.data[i]);
        }

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.xclk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.xclk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.xclk, GPIO_FLOATING);
        gpio_matrix_out(config->pin.xclk, CAM_CLK_IDX, config->invert.xclk, false);
    } else {
        ESP_LOGE(TAG, "cam width wrong!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t cam_dma_config(cam_config_t *config) 
{
    int cnt = 0;
    if (config->mode.jpeg) {
        lcd_cam_obj->cam.dma_buffer_size = 2048;
        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        lcd_cam_obj->cam.dma_node_buffer_size = lcd_cam_obj->cam.dma_half_buffer_size;
    } else {
        if (config->max_dma_buffer_size / 2.0 > 16384) { // must less than max(cam_rec_data_bytelen)
            config->max_dma_buffer_size = 16384 * 2;
        }
        for (cnt = 0; cnt < config->max_dma_buffer_size; cnt++) { // Find a buffer size that can be divisible by
            if (lcd_cam_obj->cam.recv_size % (config->max_dma_buffer_size - cnt) == 0) {
                break;
            }
        }
        lcd_cam_obj->cam.dma_buffer_size = config->max_dma_buffer_size - cnt;
        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        for (cnt = 0; cnt < LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE; cnt++) { // Find a divisible dma size
            if ((lcd_cam_obj->cam.dma_half_buffer_size) % (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt) == 0) {
                break;
            }
        }
        lcd_cam_obj->cam.dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt;
    }

    lcd_cam_obj->cam.dma_node_cnt = (lcd_cam_obj->cam.dma_buffer_size) / lcd_cam_obj->cam.dma_node_buffer_size; // Number of DMA nodes
    lcd_cam_obj->cam.frame_copy_cnt = lcd_cam_obj->cam.recv_size / lcd_cam_obj->cam.dma_half_buffer_size; // Number of interrupted copies, ping-pong copy

    ESP_LOGI(TAG, "cam_buffer_size: %d, cam_dma_size: %d, cam_dma_node_cnt: %d, cam_total_cnt: %d\n", lcd_cam_obj->cam.dma_buffer_size, lcd_cam_obj->cam.dma_node_buffer_size, lcd_cam_obj->cam.dma_node_cnt, lcd_cam_obj->cam.frame_copy_cnt);

    lcd_cam_obj->cam.dma    = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    lcd_cam_obj->cam.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);

    for (int x = 0; x < lcd_cam_obj->cam.dma_node_cnt; x++) {
        lcd_cam_obj->cam.dma[x].size = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].length = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].eof = 0;
        lcd_cam_obj->cam.dma[x].owner = 1;
        lcd_cam_obj->cam.dma[x].buf = (lcd_cam_obj->cam.dma_buffer + lcd_cam_obj->cam.dma_node_buffer_size * x);
        lcd_cam_obj->cam.dma[x].empty = &lcd_cam_obj->cam.dma[(x + 1) % lcd_cam_obj->cam.dma_node_cnt];
    }

    GDMA.channel[lcd_cam_obj->dma_num].in.link.addr = ((uint32_t)&lcd_cam_obj->cam.dma[0]) & 0xfffff;
    LCD_CAM.cam_ctrl1.cam_rec_data_bytelen = lcd_cam_obj->cam.dma_half_buffer_size - 1; // Ping pong operation
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

    if (lcd_cam_obj->cam_en) {
        cam_stop();
        if (lcd_cam_obj->cam.task_handle) {
            vTaskDelete(lcd_cam_obj->cam.task_handle);
        }
        if (lcd_cam_obj->cam.event_queue) {
            vQueueDelete(lcd_cam_obj->cam.event_queue);
        }
        if (lcd_cam_obj->cam.frame_buffer_queue) {
            vQueueDelete(lcd_cam_obj->cam.frame_buffer_queue);
        }
        if (lcd_cam_obj->cam.dma) {
            free(lcd_cam_obj->cam.dma);
        }
        if (lcd_cam_obj->cam.dma_buffer) {
            free(lcd_cam_obj->cam.dma_buffer);
        }
        if (lcd_cam_obj->cam.frame_en) {
            free(lcd_cam_obj->cam.frame_en);
        }
        if (lcd_cam_obj->cam.frame_buffer) {
            free(lcd_cam_obj->cam.frame_buffer);
        }
    }
    
    if (lcd_cam_obj->lcd_cam_intr_handle) {
        esp_intr_free(lcd_cam_obj->lcd_cam_intr_handle);
    }   
    if (lcd_cam_obj->dma_in_intr_handle) {
        esp_intr_free(lcd_cam_obj->dma_in_intr_handle);
    }   
    if (lcd_cam_obj->dma_out_intr_handle) {
        esp_intr_free(lcd_cam_obj->dma_out_intr_handle);
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
        if (REG_GET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI3_CLK_EN) == 0) {
            REG_CLR_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI3_CLK_EN);
            REG_SET_BIT(SYSTEM_PERIP_CLK_EN0_REG, SYSTEM_SPI3_CLK_EN);
            REG_SET_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI3_RST);
            REG_CLR_BIT(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SPI3_RST);
        }
    }

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
        if (GDMA.channel[x].out.link.start == 0x0 && GDMA.channel[x].in.link.start == 0x0) {
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
    lcd_cam_obj->cam_en = config->cam.en;
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
        } else if (config->lcd.width <= LCD_DATA_MAX_WIDTH && config->lcd.width % 8 == 0){
            handle->lcd.write_data = lcd_write_data;
        }
        
        handle->lcd.swap_data = lcd_swap_data;
        ESP_LOGI(TAG, "lcd init ok\n");
    }

    if (lcd_cam_obj->cam_en) {
        lcd_cam_obj->cam.jpeg_mode = config->cam.mode.jpeg;
        lcd_cam_obj->cam.vsync_pin = config->cam.pin.vsync;
        lcd_cam_obj->cam.vsync_invert = config->cam.invert.vsync;
        lcd_cam_obj->cam.frame_cnt = config->cam.frame_cnt;
        lcd_cam_obj->cam.recv_size = config->cam.recv_size;
        lcd_cam_obj->cam.swap_data = config->cam.swap_data;
        ret |= cam_set_pin(&config->cam);
        ret |= cam_dma_config(&config->cam);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "cam config fail!");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }

        lcd_cam_obj->cam.event_queue = xQueueCreate(2, sizeof(cam_event_t));
        lcd_cam_obj->cam.frame_buffer_queue = xQueueCreate(lcd_cam_obj->cam.frame_cnt, sizeof(frame_buffer_event_t));
        lcd_cam_obj->cam.frame_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.frame_cnt * lcd_cam_obj->cam.recv_size * sizeof(uint8_t), config->cam.frame_caps);
        lcd_cam_obj->cam.frame_en = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.frame_cnt * sizeof(uint8_t), MALLOC_CAP_DEFAULT);

        for (int x = 0; x < lcd_cam_obj->cam.frame_cnt; x++) {
            lcd_cam_obj->cam.frame_en[x] = 1;
        }
        xTaskCreate(cam_task, "cam_task", config->cam.task_stack, NULL, config->cam.task_pri, &lcd_cam_obj->cam.task_handle);

        if (lcd_cam_obj->cam.event_queue == NULL || \
            lcd_cam_obj->cam.frame_buffer_queue == NULL || \
            lcd_cam_obj->cam.frame_buffer == NULL || \
            lcd_cam_obj->cam.frame_en == NULL || \
            lcd_cam_obj->cam.task_handle == NULL) {
            ESP_LOGE(TAG, "cam config fail!");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }

        handle->cam.start = cam_start;
        handle->cam.stop  = cam_stop;
        handle->cam.take  = cam_take;
        handle->cam.give  = cam_give;
        ESP_LOGI(TAG, "cam init ok\n");
    }

    if (lcd_cam_obj->lcd_en || lcd_cam_obj->cam_en) {
        ret |= esp_intr_alloc((DMA_IN_CH0_INTR_SOURCE + lcd_cam_obj->dma_num), ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, dma_isr, NULL, &lcd_cam_obj->dma_in_intr_handle);
        ret |= esp_intr_alloc((DMA_OUT_CH0_INTR_SOURCE + lcd_cam_obj->dma_num), ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, dma_isr, NULL, &lcd_cam_obj->dma_out_intr_handle);
        ret |= esp_intr_alloc(LCD_CAM_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, lcd_cam_isr, NULL, &lcd_cam_obj->lcd_cam_intr_handle);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "lcd_cam intr alloc fail!");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}
