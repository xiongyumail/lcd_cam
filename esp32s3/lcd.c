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
                                                           .ctr = {.mode_sel = 1, .frame_buffer_num = 0, .max_cp_buffer_size = 32000, .swap_data = 0, .pix_bytes = 2} \
    }

typedef struct lcd_self_struct lcd_self_t;

struct lcd_self_struct {
    lcd_handle_t *handle;
    QueueHandle_t event_queue;
    struct {
        uint32_t buffer_size;
        uint32_t half_buffer_size;
        uint32_t node_buffer_size;
        uint32_t node_cnt;
        uint32_t half_node_cnt;
        lldesc_t *lldesc;
        uint8_t  *buffer;
    } dma_copy;
    struct {
        uint32_t buffer_size;
        lldesc_t **lldesc_table;
    } dma_fb;
    uint8_t dma_num;
    int fb_pos;
    intr_handle_t lcd_intr_handle;
    intr_handle_t dma_out_intr_handle;
    lcd_self_t *next;
};

static void IRAM_ATTR lcd_isr(void *arg)
{
    lcd_self_t *self = (lcd_self_t *)arg;

    typeof(LCD_CAM.lc_dma_int_st) status = LCD_CAM.lc_dma_int_st;
    BaseType_t HPTaskAwoken = pdFALSE;

    if (status.val == 0) {
        return;
    }

    LCD_CAM.lc_dma_int_clr.val = status.val;

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR lcd_start(lcd_self_t *self, uint32_t addr, size_t len)
{
    lcd_ctrl_t *ctrl = self->handle->ctrl;

    while (LCD_CAM.lcd_user.lcd_start);

    LCD_CAM.lcd_user.lcd_reset = 1;
    LCD_CAM.lcd_user.lcd_reset = 0;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
    LCD_CAM.lcd_misc.lcd_afifo_reset = 0;

    while (GDMA.channel[self->dma_num].out.link.start);

    GDMA.channel[self->dma_num].out.conf0.val = 0;
    GDMA.channel[self->dma_num].out.conf1.val = 0;
    GDMA.channel[self->dma_num].out.int_clr.val = ~0;
    GDMA.channel[self->dma_num].out.int_ena.val = 0;
    GDMA.channel[self->dma_num].out.conf0.out_rst = 1;
    GDMA.channel[self->dma_num].out.conf0.out_rst = 0;
    GDMA.channel[self->dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[self->dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[self->dma_num].out.conf1.out_ext_mem_bk_size = 2;
    GDMA.sram_size[self->dma_num].out.out_size = 14; // fifo 128 Bytes
    GDMA.channel[self->dma_num].out.peri_sel.sel = 5;
    GDMA.channel[self->dma_num].out.pri.tx_pri = 1;
    GDMA.channel[self->dma_num].out.int_ena.out_eof = 1;
    GDMA.channel[self->dma_num].out.link.addr = addr;
    GDMA.channel[self->dma_num].out.link.start = 1;

    if (ctrl->ctr.swap_data && len > 1) {
        LCD_CAM.lcd_user.lcd_8bits_order = 1;
    } else {
        LCD_CAM.lcd_user.lcd_8bits_order = 0;
    }

    if (len > 16) { // test value
        while (GDMA.channel[self->dma_num].out.outfifo_status.outfifo_cnt_l3 < 16);
    } else {
        while (GDMA.channel[self->dma_num].out.outfifo_status.outfifo_cnt_l3 < len);
    }

    LCD_CAM.lcd_user.lcd_update = 1;
    LCD_CAM.lcd_user.lcd_start = 1;
}

static void IRAM_ATTR dma_isr(void *arg)
{
    lcd_self_t *self = (lcd_self_t *)arg;

    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(GDMA.channel[self->dma_num].out.int_st) out_status = GDMA.channel[self->dma_num].out.int_st;

    if (out_status.val != 0) {
        GDMA.channel[self->dma_num].out.int_clr.val = out_status.val;

        if (out_status.out_eof) {
            lcd_start(self, ((uint32_t)self->dma_fb.lldesc_table[self->fb_pos]) & 0xfffff, self->dma_fb.buffer_size);
            xQueueSendFromISR(self->event_queue, &out_status.val, &HPTaskAwoken);
        }
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void lcd_dma_copy_int(lcd_self_t *self)
{
    // Generate a data DMA linked list
    lldesc_t *lldesc = self->dma_copy.lldesc;

    for (int x = 0; x < self->dma_copy.node_cnt; x++) {
        lldesc[x].size = self->dma_copy.node_buffer_size;
        lldesc[x].length = self->dma_copy.node_buffer_size;
        lldesc[x].buf = (self->dma_copy.buffer + self->dma_copy.node_buffer_size * x);
        lldesc[x].eof = !((x + 1) % self->dma_copy.half_node_cnt);
        lldesc[x].empty = &lldesc[(x + 1) % self->dma_copy.node_cnt];
    }

    lldesc[self->dma_copy.half_node_cnt - 1].empty = NULL;
    lldesc[self->dma_copy.node_cnt - 1].empty = NULL;
}

static void lcd_dma_copy_left(lcd_self_t *self, int pos, size_t len)
{
    int end_pos = 0, size = 0;
    lldesc_t *lldesc = self->dma_copy.lldesc;

    // Processing data length is an integer multiple of self->dma_copy.node_buffer_size
    if (len % self->dma_copy.node_buffer_size) {
        end_pos = (pos % 2) * self->dma_copy.half_node_cnt + len / self->dma_copy.node_buffer_size;
        size = len % self->dma_copy.node_buffer_size;
    } else {
        end_pos = (pos % 2) * self->dma_copy.half_node_cnt + len / self->dma_copy.node_buffer_size - 1;
        size = self->dma_copy.node_buffer_size;
    }

    // Process the tail node to make it a DMA tail
    lldesc[end_pos].size = size;
    lldesc[end_pos].length = size;
    lldesc[end_pos].eof = 1;
    lldesc[end_pos].empty = NULL;
}

#include "esp_wifi.h"

static void lcd_task(void *arg)
{
    lcd_self_t *self = (lcd_self_t *)arg;
    lcd_port_t *port = self->handle->port;
    static uint32_t ticks_now = 0, ticks_last = 0;
    struct timeval now;
    int event  = 0;

    while (self->fb_pos == -1) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    lcd_start(self, ((uint32_t)self->dma_fb.lldesc_table[self->fb_pos]) & 0xfffff, self->dma_fb.buffer_size);

    while (1) {
        if (xQueueReceive(self->event_queue, (void *)&event, 100 / portTICK_PERIOD_MS) != pdTRUE) {
            ESP_LOGE(TAG, "error frame\n");
            LCD_CAM.lcd_user.lcd_reset = 1;
            LCD_CAM.lcd_user.lcd_reset = 0;
            LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
            LCD_CAM.lcd_misc.lcd_afifo_reset = 0;
            GDMA.channel[self->dma_num].out.conf0.out_rst = 1;
            GDMA.channel[self->dma_num].out.conf0.out_rst = 0;
            lcd_start(self, ((uint32_t)self->dma_fb.lldesc_table[self->fb_pos]) & 0xfffff, self->dma_fb.buffer_size);
        }

        gettimeofday(&now, NULL);
        ticks_now = now.tv_sec * 1000 + now.tv_usec / 1000;

        if (ticks_now - ticks_last > 0) {
            ESP_LOGI(TAG, "fps: %.2f\n", 1000.0 / (int)(ticks_now - ticks_last));
        }

        ticks_last = ticks_now;
    }
}

static void lcd_calc_clk(lcd_self_t *self, uint32_t fre, uint8_t *calc_m, uint8_t *calc_b, uint8_t *calc_a, uint8_t *calc_n)
{
    float F = 160000000;
    float f = 0, f_diff = 0, f_diff_min = 80000000, f_calc = 0;

    int n = (fre <= 40000000) ? 2 : 1;

    for (int m = 2; m < 256; m++) {
        for (int a = 2; a < 64; a++) {
            for (int b = 0; b < a; b++) {
                f = (F / (m + ((float)b / a))) / n;
                f_diff = abs(f - fre);

                if (f_diff < f_diff_min) {
                    f_diff_min = f_diff;
                    f_calc = f;
                    *calc_m = m;
                    *calc_b = b;
                    *calc_a = a;
                    *calc_n = n;
                }
            }
        }
    }

    ESP_LOGI(TAG, "f_calc: %f, f_diff_min: %f, m: %d, b: %d, a: %d, n: %d\n", f_calc, f_diff_min, *calc_m, *calc_b, *calc_a, *calc_n);
}

// LCD rgb 模式配置水平显示时序参数
static void lcd_cam_rgb_set_horizontal(uint32_t ha, uint32_t hsync, uint32_t hbp, uint32_t hfp)
{
    // Thsync + Thbp + Thd + Thfp  = Th
    LCD_CAM.lcd_ctrl1.lcd_ha_width = ha - 1;
    LCD_CAM.lcd_ctrl.lcd_hb_front = hbp - 1;
    LCD_CAM.lcd_ctrl2.lcd_hsync_width = hsync - 1;
    LCD_CAM.lcd_ctrl1.lcd_ht_width = (ha + hsync + hbp + hfp) - 1;
}

// LCD rgb 模式配置垂直显示时序参数
static void lcd_cam_rgb_set_vertical(uint32_t va, uint32_t vsync, uint32_t vbp, uint32_t vfp)
{
    // Tvsync + Tvbp + Tvd + Tvfp  = Tv
    LCD_CAM.lcd_ctrl.lcd_va_height = va - 1;
    LCD_CAM.lcd_ctrl1.lcd_vb_front = vbp - 1;
    LCD_CAM.lcd_ctrl2.lcd_vsync_width = vsync - 1;
    LCD_CAM.lcd_ctrl.lcd_vt_height = (va + vsync + vbp + vfp) - 1;
}

static esp_err_t lcd_reg_config(lcd_self_t *self)
{
    lcd_ctrl_t *ctrl = self->handle->ctrl;

    GDMA.channel[self->dma_num].out.conf0.val = 0;
    GDMA.channel[self->dma_num].out.conf1.val = 0;
    GDMA.channel[self->dma_num].out.int_clr.val = ~0;
    GDMA.channel[self->dma_num].out.int_ena.val = 0;

    uint8_t m = 2, b = 0, a = 10, n = 2;
    lcd_calc_clk(self, ctrl->bus.fre, &m, &b, &a, &n);

    LCD_CAM.lcd_clock.val = 0;
    LCD_CAM.lcd_clock.clk_en = 1;
    LCD_CAM.lcd_clock.lcd_clk_sel = 3;
    LCD_CAM.lcd_clock.lcd_clkm_div_b = b;
    LCD_CAM.lcd_clock.lcd_clkm_div_a = a;
    LCD_CAM.lcd_clock.lcd_clkm_div_num = m;
    LCD_CAM.lcd_clock.lcd_clkcnt_n = (n - 1);
    LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = !(n - 1);
    LCD_CAM.lcd_clock.lcd_ck_idle_edge = 1; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_clock.lcd_ck_out_edge = 0; // After lcd_clk_equ_sysclk is set to 1, this bit has no effect
    LCD_CAM.lcd_user.val = 0;
    LCD_CAM.lcd_user.lcd_2byte_en = (ctrl->bus.width == 16) ? 1 : 0;
    LCD_CAM.lcd_user.lcd_byte_order = 0;
    LCD_CAM.lcd_user.lcd_bit_order = 0;
    LCD_CAM.lcd_user.lcd_cmd = 0;		// FSM CMD phase
    LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;	// 2 cycle command
    LCD_CAM.lcd_user.lcd_dout = 1;	// FSM DOUT phase
    LCD_CAM.lcd_user.lcd_dout_cyclelen = 1 - 1;
    LCD_CAM.lcd_user.lcd_8bits_order = 0;
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
    LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = (ctrl->ctr.mode_sel == 0) ? 1 : 0;
    LCD_CAM.lcd_cmd_val = 0;	// write command

    if (ctrl->ctr.mode_sel == 0) {
        lcd_cam_rgb_set_horizontal(ctrl->res.x, ctrl->video_timing_sync.hsync, ctrl->video_timing_h.hbp, ctrl->video_timing_h.hfp);
        lcd_cam_rgb_set_vertical(ctrl->res.y, ctrl->video_timing_sync.vsync, ctrl->video_timing_v.vbp, ctrl->video_timing_v.vfp);

        LCD_CAM.lcd_ctrl2.lcd_vsync_idle_pol = 0;
        LCD_CAM.lcd_ctrl2.lcd_de_idle_pol = 0;
        LCD_CAM.lcd_ctrl2.lcd_hs_blank_en = 0;

        LCD_CAM.lcd_ctrl2.lcd_hsync_idle_pol = 0;
        LCD_CAM.lcd_ctrl2.lcd_hsync_position = 1 - 1;

        LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
    }

    LCD_CAM.lcd_user.lcd_update = 1;

    GDMA.channel[self->dma_num].out.conf0.out_rst = 1;
    GDMA.channel[self->dma_num].out.conf0.out_rst = 0;
    GDMA.channel[self->dma_num].out.conf0.outdscr_burst_en = 1;
    GDMA.channel[self->dma_num].out.conf0.out_data_burst_en = 1;
    GDMA.channel[self->dma_num].out.peri_sel.sel = (ctrl->bus.width == 1) ? 1 : 5;
    GDMA.channel[self->dma_num].out.pri.tx_pri = 1;
    GDMA.channel[self->dma_num].out.int_ena.out_eof = 1;

    return ESP_OK;
}

static esp_err_t lcd_set_pin(lcd_self_t *self)
{
    lcd_ctrl_t *ctrl = self->handle->ctrl;

    if (ctrl->bus.width == 1) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_clk.clk], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_clk.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_clk.clk, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_clk.clk, SPI3_CLK_OUT_IDX, ctrl->pin_clk.clk_inv, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_data[0].data], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_data[0].data, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_data[0].data, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_data[0].data, SPI3_D_OUT_IDX, ctrl->pin_data[0].data_inv, false);
    } else if (ctrl->bus.width <= LCD_DATA_MAX_WIDTH && ctrl->bus.width % 8 == 0) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_clk.clk], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_clk.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_clk.clk, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_clk.clk, LCD_PCLK_IDX, ctrl->pin_clk.clk_inv, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_vsync.vsync], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_vsync.vsync, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_vsync.vsync, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_vsync.vsync, LCD_V_SYNC_IDX, ctrl->pin_vsync.vsync_inv, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_hsync.hsync], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_hsync.hsync, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_hsync.hsync, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_hsync.hsync, LCD_H_SYNC_IDX, ctrl->pin_hsync.hsync_inv, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[ctrl->pin_href.href], PIN_FUNC_GPIO);
        gpio_set_direction(ctrl->pin_href.href, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(ctrl->pin_href.href, GPIO_FLOATING);
        gpio_matrix_out(ctrl->pin_href.href, LCD_H_ENABLE_IDX, ctrl->pin_href.href_inv, false);

        for (int i = 0; i < ctrl->bus.width; i++) {
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

static esp_err_t lcd_dma_copy_config(lcd_self_t *self)
{
    lcd_ctrl_t *ctrl = self->handle->ctrl;

    int cnt = 0;

    if (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE % 2 != 0) {
        ESP_LOGE(TAG, "need 2-byte aligned data length");
        return ESP_FAIL;
    }

    if (ctrl->bus.width == 1) {
        if (ctrl->ctr.max_cp_buffer_size > 65536) {
            ctrl->ctr.max_cp_buffer_size = 65536;
        }
    }

    if (ctrl->ctr.max_cp_buffer_size >= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * 2) {
        self->dma_copy.node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;

        for (cnt = 0; cnt < ctrl->ctr.max_cp_buffer_size - 8; cnt++) { // Find a buffer size that can divide dma_size
            if ((ctrl->ctr.max_cp_buffer_size - cnt) % (self->dma_copy.node_buffer_size * 2) == 0) {
                break;
            }
        }

        self->dma_copy.buffer_size = ctrl->ctr.max_cp_buffer_size - cnt;
    } else {
        self->dma_copy.node_buffer_size = ctrl->ctr.max_cp_buffer_size / 2;
        self->dma_copy.buffer_size = self->dma_copy.node_buffer_size * 2;
    }

    self->dma_copy.half_buffer_size = self->dma_copy.buffer_size / 2;

    self->dma_copy.node_cnt = (self->dma_copy.buffer_size) / self->dma_copy.node_buffer_size; // Number of DMA nodes
    self->dma_copy.half_node_cnt = self->dma_copy.node_cnt / 2;

    ESP_LOGI(TAG, "lcd_buffer_size: %d, lcd_dma_size: %d, lcd_dma_node_cnt: %d\n", self->dma_copy.buffer_size, self->dma_copy.node_buffer_size, self->dma_copy.node_cnt);

    self->dma_copy.lldesc  = (lldesc_t *)heap_caps_malloc(self->dma_copy.node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA);
    self->dma_copy.buffer = (uint8_t *)heap_caps_malloc(self->dma_copy.buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA);

    return ESP_OK;
}

static esp_err_t lcd_dma_fb_config(lcd_self_t *self)
{
    lcd_ctrl_t *ctrl = self->handle->ctrl;

    int x = 0, y = 0, left = 0, cnt = 0;
    int len = ctrl->res.x * ctrl->res.y * ctrl->ctr.pix_bytes;
    self->dma_fb.buffer_size = len;

    if (len % LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE) {
        cnt = len / LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        left = len % LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
    } else {
        cnt = len / LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - 1;
        left = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
    }

    self->dma_fb.lldesc_table = (lldesc_t **)heap_caps_malloc((ctrl->ctr.frame_buffer_num) * sizeof(lldesc_t *), MALLOC_CAP_DMA);

    for (y = 0; y < ctrl->ctr.frame_buffer_num; y++) {
        self->dma_fb.lldesc_table[y] = (lldesc_t *)heap_caps_malloc((cnt + 1) * sizeof(lldesc_t), MALLOC_CAP_DMA);
        lldesc_t *lldesc = self->dma_fb.lldesc_table[y];

        for (x = 0; x < cnt; x++) {
            lldesc[x].size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
            lldesc[x].length = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
            lldesc[x].buf = (ctrl->frame_buffer[y].addr + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
            lldesc[x].eof = 0;
            lldesc[x].empty = &lldesc[(x + 1)];
        }

        if (left) {
            lldesc[x].size = left;
            lldesc[x].length = left;
            lldesc[x].buf = (ctrl->frame_buffer[y].addr + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
            lldesc[x].eof = 1;
            lldesc[x].empty = NULL;
        }
    }

    return ESP_OK;
}

static void lcd_run(lcd_handle_t *handle)
{
    lcd_self_t *self = handle->self;
    lcd_ctrl_t *ctrl = handle->ctrl;

    self->fb_pos = -1;

    if (ctrl->ctr.frame_buffer_num > 0) {
        xTaskCreate(lcd_task, "lcd_task", 3072, (void *)self, configMAX_PRIORITIES, NULL);
    }

    ESP_LOGI(TAG, "lcd run\n");

    return ESP_OK;
}

static void lcd_frame_write(lcd_handle_t *handle, int pos)
{
    lcd_self_t *self = handle->self;

    self->fb_pos = pos;
}

static void lcd_dma_copy_data(lcd_handle_t *handle, uint8_t *data, size_t len)
{
    lcd_self_t *self = handle->self;
    lcd_ctrl_t *ctrl = handle->ctrl;

    int event  = 0;
    int x = 0, y = 0, left = 0, cnt = 0;

    if (len <= 0) {
        ESP_LOGE(TAG, "wrong len!");
        return;
    }

    lcd_dma_copy_int(self);
    cnt = len / self->dma_copy.half_buffer_size;
    // Start signal
    xQueueSend(self->event_queue, &event, 0);

    // Process a complete piece of data, ping-pong operation
    for (x = 0; x < cnt; x++) {
        uint8_t *out = self->dma_copy.lldesc[(x % 2) * self->dma_copy.half_node_cnt].buf;
        uint8_t *in  = data;

        if (ctrl->ctr.swap_data) {
            for (y = 0; y < self->dma_copy.half_buffer_size; y += 2) {
                out[y + 1] = in[y + 0];
                out[y + 0] = in[y + 1];
            }
        } else {
            memcpy(out, in, self->dma_copy.half_buffer_size);
        }

        data += self->dma_copy.half_buffer_size;
        xQueueReceive(self->event_queue, (void *)&event, portMAX_DELAY);
        lcd_start(self, ((uint32_t)&self->dma_copy.lldesc[(x % 2) * self->dma_copy.half_node_cnt]) & 0xfffff, self->dma_copy.half_buffer_size);
    }

    left = len % self->dma_copy.half_buffer_size;

    // Process remaining incomplete segment data
    if (left) {
        uint8_t *out = self->dma_copy.lldesc[(x % 2) * self->dma_copy.half_node_cnt].buf;
        uint8_t *in  = data;
        cnt = left - left % 2;

        if (cnt) {
            if (ctrl->ctr.swap_data) {
                for (y = 0; y < cnt; y += 2) {
                    out[y + 1] = in[y + 0];
                    out[y + 0] = in[y + 1];
                }
            } else {
                memcpy(out, in, cnt);
            }
        }

        if (left % 2) {
            out[cnt] = in[cnt];
        }

        lcd_dma_copy_left(self, x, left);
        xQueueReceive(self->event_queue, (void *)&event, portMAX_DELAY);
        lcd_start(self, ((uint32_t)&self->dma_copy.lldesc[(x % 2) * self->dma_copy.half_node_cnt]) & 0xfffff, left);
    }

    xQueueReceive(self->event_queue, (void *)&event, portMAX_DELAY);
}


static void lcd_dma_fb_data(lcd_handle_t *handle, uint8_t *data, size_t len)
{
    lcd_self_t *self = handle->self;
    lcd_ctrl_t *ctrl = handle->ctrl;

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

    lldesc_t *lldesc = (lldesc_t *)heap_caps_malloc((cnt + 1) * sizeof(lldesc_t), MALLOC_CAP_DMA);

    // lldesc_t *dma = self->dma_copy.lldesc;
    for (x = 0; x < cnt; x++) {
        lldesc[x].size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        lldesc[x].length = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE;
        lldesc[x].buf = (data + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
        lldesc[x].eof = 0;
        lldesc[x].empty = &lldesc[(x + 1)];
    }

    if (left) {
        lldesc[x].size = left;
        lldesc[x].length = left;
        lldesc[x].buf = (data + LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE * x);
        lldesc[x].eof = 1;
        lldesc[x].empty = NULL;
    }

    xQueueSend(self->event_queue, &event, 0);
    xQueueReceive(self->event_queue, (void *)&event, portMAX_DELAY);
    lcd_start(self, ((uint32_t)&lldesc[0]) & 0xfffff, len);
    xQueueReceive(self->event_queue, (void *)&event, portMAX_DELAY);
    free(lldesc);
    LCD_CAM.lcd_user.lcd_8bits_order = 1;
}

esp_err_t lcd_create(lcd_handle_t *handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "arg error\n");
        return ESP_ERR_INVALID_ARG;
    }

    handle->self = (void *)heap_caps_calloc(1, sizeof(lcd_self_t), MALLOC_CAP_DEFAULT);
    handle->ctrl = (lcd_ctrl_t *)heap_caps_calloc(1, sizeof(lcd_ctrl_t), MALLOC_CAP_DEFAULT);
    handle->port = (lcd_port_t *)heap_caps_calloc(1, sizeof(lcd_port_t), MALLOC_CAP_DEFAULT);

    if (handle->self == NULL || \
            handle->ctrl == NULL || \
            handle->port == NULL) {
        ESP_LOGE(TAG, "lcd create fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }

    memcpy(handle->ctrl, &LCD_DEFAULT_CTRL(), sizeof(lcd_ctrl_t));

    return ESP_OK;
}

esp_err_t lcd_config(lcd_handle_t *handle)
{
    lcd_self_t *self = handle->self;
    lcd_ctrl_t *ctrl = handle->ctrl;
    lcd_port_t *port = handle->port;
    esp_err_t ret = ESP_OK;

    self->handle = handle;

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

    for (int x = LCD_CAM_DMA_MAX_NUM - 1; x >= 0; x--) {
        if (GDMA.channel[x].out.link.start == 0x0) {
            self->dma_num = x;
            break;
        }

        if (x == LCD_CAM_DMA_MAX_NUM - 1) {
            ESP_LOGE(TAG, "DMA error\n");
            lcd_remove(handle);
            return ESP_FAIL;
        }
    }

    ret |= lcd_reg_config(self);
    ret |= lcd_set_pin(self);

    if (ctrl->ctr.mode_sel) {
        ret |= lcd_dma_copy_config(self);
    } else {
        if (ctrl->ctr.frame_buffer_num > 0) {
            ret |= lcd_dma_fb_config(self);
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd config fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }

    self->event_queue = xQueueCreate(1, sizeof(int));

    if (self->event_queue == NULL) {
        ESP_LOGE(TAG, "lcd config fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }

    ret |= esp_intr_alloc((DMA_OUT_CH0_INTR_SOURCE + self->dma_num), ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, dma_isr, (void *)self, &self->dma_out_intr_handle);
    ret |= esp_intr_alloc(LCD_CAM_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, lcd_isr, (void *)self, &self->lcd_intr_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "lcd intr alloc fail!");
        lcd_remove(handle);
        return ESP_FAIL;
    }

    port->ctr.slv.run = lcd_run;

    if (ctrl->ctr.mode_sel == 1) {
        port->data.slv.write = lcd_dma_copy_data;
    } else {
        port->data.slv.write = lcd_dma_fb_data;
    }

    port->fb.slv.write = lcd_frame_write;

    return ESP_OK;
}

esp_err_t lcd_remove(lcd_handle_t *handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "arg error\n");
        return ESP_ERR_INVALID_ARG;
    }

    lcd_self_t *self = handle->self;

    if (self->event_queue) {
        vQueueDelete(self->event_queue);
    }

    if (self->dma_copy.lldesc) {
        free(self->dma_copy.lldesc);
    }

    if (self->dma_fb.lldesc_table) {
        for (int x = 0; x < self->handle->ctrl->ctr.frame_buffer_num; x++) {
            if (self->dma_fb.lldesc_table[x]) {
                free(self->dma_fb.lldesc_table[x]);
            }
        }

        free(self->dma_fb.lldesc_table);
    }

    free(self);
    self = NULL;
    return ESP_OK;
}
