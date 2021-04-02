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
#include "soc/apb_ctrl_reg.h"
#include "esp32/rom/lldesc.h"
#include "esp32/rom/cache.h"
#include "hal/gpio_ll.h"
#include "driver/ledc.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "i2s_struct.h"
#include "spi_struct.h"
#include "lcd_cam.h"

static const char *TAG = "lcd_cam";

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4000) // 4-byte aligned

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
    uint8_t width;
    uint8_t fifo_mode;
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
    intr_handle_t lcd_cam_intr_handle;
    intr_handle_t spi_intr_handle;
} lcd_cam_obj_t;

static lcd_cam_obj_t *lcd_cam_obj = NULL;

static void IRAM_ATTR i2s_isr(void *arg)
{
    cam_event_t cam_event = {0};
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(I2S0.int_st) status = I2S0.int_st;
    I2S0.int_clr.val = status.val;
    if (status.val == 0) {
        return;
    }

    if (status.out_eof) {
        xQueueSendFromISR(lcd_cam_obj->lcd.event_queue, &status.val, &HPTaskAwoken);
    }

    if (status.in_suc_eof) {
        cam_event = CAM_IN_SUC_EOF_EVENT;
        xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR cam_vsync_isr(void *arg)
{
    cam_event_t cam_event = {0};
    BaseType_t HPTaskAwoken = pdFALSE;
    // filter
    esp_rom_delay_us(1);
    if (gpio_ll_get_level(&GPIO, lcd_cam_obj->cam.vsync_pin) == !lcd_cam_obj->cam.vsync_invert) {
        cam_event = CAM_VSYNC_EVENT;
        xQueueSendFromISR(lcd_cam_obj->cam.event_queue, (void *)&cam_event, &HPTaskAwoken);
    }

    if(HPTaskAwoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void IRAM_ATTR spi_isr(void *arg)
{
    int event;
    BaseType_t HPTaskAwoken = pdFALSE;
    typeof(SPI3.dma_int_st) status = SPI3.dma_int_st;
    SPI3.dma_int_clr.val = status.val;
    if (status.val == 0) {
        return;
    }

    if (status.out_eof) {
        xQueueSendFromISR(lcd_cam_obj->lcd.event_queue, &status.val, &HPTaskAwoken);
    }

    if (HPTaskAwoken == pdTRUE) {
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

static void i2s_start(uint32_t addr, size_t len)
{
    while (!I2S0.state.tx_idle);
    I2S0.fifo_conf.tx_fifo_mod = lcd_cam_obj->lcd.fifo_mode;
    I2S0.conf.tx_start = 0;
    I2S0.conf.tx_reset = 1;
    I2S0.conf.tx_reset = 0;
    I2S0.lc_conf.out_rst = 1;
    I2S0.lc_conf.out_rst = 0;
    I2S0.conf.tx_fifo_reset = 1;
    I2S0.conf.tx_fifo_reset = 0;
    I2S0.out_link.addr = addr;
    I2S0.out_link.start = 1;
    esp_rom_delay_us(1);
    I2S0.conf.tx_start = 1;
}

static void i2s_write_8bit_data(uint8_t *data, size_t len)
{
    int event  = 0;
    int x = 0, y = 0, left = 0, cnt = 0;
    if (len <= 0) {
        ESP_LOGE(TAG, "wrong len!");
        return;
    }
    len = len * 2;
    lcd_dma_set_int();
    lcd_cam_obj->lcd.fifo_mode = 1;
    // Start signal
    xQueueSend(lcd_cam_obj->lcd.event_queue, &event, 0);
    cnt = len / lcd_cam_obj->lcd.dma_half_buffer_size;
    // Process a complete piece of data, ping-pong operation
    for (x = 0; x < cnt; x++) {
        uint8_t *out = lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf;
        uint8_t *in  = data;
        if (lcd_cam_obj->lcd.swap_data) {
            for (y = 0; y < lcd_cam_obj->lcd.dma_half_buffer_size; y+=4) {
                out[y+3] = in[(y>>1)+0];
                out[y+1] = in[(y>>1)+1];
            }
        } else {
            for (y = 0; y < lcd_cam_obj->lcd.dma_half_buffer_size; y+=4) {
                out[y+1] = in[(y>>1)+0];
                out[y+3] = in[(y>>1)+1];
            }
        }
        data += lcd_cam_obj->lcd.dma_half_buffer_size >> 1;
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        i2s_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, lcd_cam_obj->lcd.dma_half_buffer_size);
    }
    left = len % lcd_cam_obj->lcd.dma_half_buffer_size;
    // Process remaining incomplete segment data
    while (left) {
        uint8_t *out = lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf;
        uint8_t *in  = data;
        if (left > 2) {
            cnt = left - left % 4;
            left = left % 4;
            data += cnt >> 1;
            if (lcd_cam_obj->lcd.swap_data) {
                for (y = 0; y < cnt; y+=4) {
                    out[y+3] = in[(y>>1)+0];
                    out[y+1] = in[(y>>1)+1];
                }
            } else {
                for (y = 0; y < cnt; y+=4) {
                    out[y+1] = in[(y>>1)+0];
                    out[y+3] = in[(y>>1)+1];
                }
            }
        } else {
            cnt = 4;
            left = 0;
            lcd_cam_obj->lcd.fifo_mode = 3;
            out[3] = in[0];
        }
        lcd_dma_set_left(x, cnt);
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        i2s_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, cnt);
        x++;
    }
    xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
    while (!I2S0.state.tx_idle);
}

static void i2s_write_16bit_data(uint8_t *data, size_t len)
{
    int event  = 0;
    int x = 0, y = 0, left = 0, cnt = 0;
    if (len <= 0 || len % 2 != 0) {
        ESP_LOGE(TAG, "wrong len!");
        return;
    }
    lcd_dma_set_int();
    lcd_cam_obj->lcd.fifo_mode = 1;
    // Start signal
    xQueueSend(lcd_cam_obj->lcd.event_queue, &event, 0);
    cnt = len / lcd_cam_obj->lcd.dma_half_buffer_size;
    // Process a complete piece of data, ping-pong operation
    for (x = 0; x < cnt; x++) {
        uint8_t *out = lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf;
        uint8_t *in  = data;
        if (lcd_cam_obj->lcd.swap_data) {
            for (y = 0; y < lcd_cam_obj->lcd.dma_half_buffer_size; y+=4) {
                out[y+3] = in[y+0];
                out[y+2] = in[y+1];
                out[y+1] = in[y+2];
                out[y+0] = in[y+3];
            }
        } else {
            for (y = 0; y < lcd_cam_obj->lcd.dma_half_buffer_size; y+=4) {
                out[y+2] = in[y+0];
                out[y+3] = in[y+1];
                out[y+0] = in[y+2];
                out[y+1] = in[y+3];
            }
        }     
        data += lcd_cam_obj->lcd.dma_half_buffer_size;
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        i2s_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, lcd_cam_obj->lcd.dma_half_buffer_size);
    }
    left = len % lcd_cam_obj->lcd.dma_half_buffer_size;
    // Process remaining incomplete segment data
    while (left) {
        uint8_t *out = lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt].buf;
        uint8_t *in = data;
        if (left > 2) {
            cnt = left - left % 4;
            left = left % 4;
            data += cnt;
            if (lcd_cam_obj->lcd.swap_data) {
                for (y = 0; y < cnt; y+=4) {
                    out[y+3] = in[y+0];
                    out[y+2] = in[y+1];
                    out[y+1] = in[y+2];
                    out[y+0] = in[y+3];
                }
            } else {
                for (y = 0; y < cnt; y+=4) {
                    out[y+2] = in[y+0];
                    out[y+3] = in[y+1];
                    out[y+0] = in[y+2];
                    out[y+1] = in[y+3];
                }
            } 
        } else {
            cnt = 4;
            left = 0;
            lcd_cam_obj->lcd.fifo_mode = 3;
            if (lcd_cam_obj->lcd.swap_data) {
                out[3] = in[0];
                out[2] = in[1];
            } else {
                out[2] = in[0];
                out[3] = in[1];
            }
        }
        lcd_dma_set_left(x, cnt);
        xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
        i2s_start(((uint32_t)&lcd_cam_obj->lcd.dma[(x % 2) * lcd_cam_obj->lcd.dma_half_node_cnt]) & 0xfffff, cnt);
        x++;
    }
    xQueueReceive(lcd_cam_obj->lcd.event_queue, (void *)&event, portMAX_DELAY);
    while (!I2S0.state.tx_idle);
}

static void spi_start(uint32_t addr, size_t len)
{
    SPI3.mosi_dlen.usr_mosi_dbitlen = len * 8 - 1;
    SPI3.dma_out_link.addr = addr;
    SPI3.dma_out_link.start = 1;
    esp_rom_delay_us(1);
    SPI3.cmd.usr = 1;
}

static void spi_write_data(uint8_t *data, size_t len)
{
    int event  = 0;
    int x = 0, y = 0, cnt = 0, left = 0;
    if (len <= 0) {
        return;
    }
    lcd_dma_set_int();
    // Start signal
    xQueueSend(lcd_cam_obj->lcd.event_queue, &event, 0);
    cnt = len / lcd_cam_obj->lcd.dma_half_buffer_size;
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
   // Configure the clock
    I2S0.clkm_conf.clkm_div_num = 2; // 160MHz / 2 = 80MHz
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_a = 10;
    I2S0.clkm_conf.clk_en = 1;

    I2S0.conf.val = 0;
    I2S0.fifo_conf.val = 0;
    I2S0.fifo_conf.dscr_en = 1;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;
    I2S0.lc_conf.check_owner = 0;
    I2S0.lc_conf.out_loop_test = 0;
    I2S0.lc_conf.out_auto_wrback = 0;
    I2S0.lc_conf.out_data_burst_en = 1;
    I2S0.lc_conf.out_no_restart_clr = 0;
    I2S0.lc_conf.indscr_burst_en = 0;
    I2S0.lc_conf.out_eof_mode = 1;

    I2S0.timing.val = 0;

    I2S0.int_ena.val = 0;
    I2S0.int_clr.val = ~0;

    if (lcd_cam_obj->lcd_en) {
        if (config->lcd.width == 1) { // SPI mode
            int div = 2;
            if (config->lcd.fre == 80000000) {
                SPI3.clock.clk_equ_sysclk = 1;
            } else {
                SPI3.clock.clk_equ_sysclk = 0;
                div = 80000000 / config->lcd.fre;
            }
            SPI3.clock.clkdiv_pre = 1 - 1;
            SPI3.clock.clkcnt_n = div - 1;
            SPI3.clock.clkcnt_l = div - 1;
            SPI3.clock.clkcnt_h = ((div >> 1) - 1);
            
            SPI3.pin.ck_dis = 0;

            SPI3.user1.val = 0;
            SPI3.slave.val = 0;
            SPI3.pin.ck_idle_edge = 0;
            SPI3.user.ck_out_edge = 0;
            SPI3.ctrl.wr_bit_order = 0;
            SPI3.ctrl.rd_bit_order = 0;
            SPI3.user.val = 0;
            SPI3.user.cs_setup = 1;
            SPI3.user.cs_hold = 1;
            SPI3.user.usr_mosi = 1;
            SPI3.user.usr_mosi_highpart = 0;

            SPI3.dma_conf.val = 0;
            SPI3.dma_conf.out_rst = 1;
            SPI3.dma_conf.out_rst = 0;
            SPI3.dma_conf.ahbm_fifo_rst = 1;
            SPI3.dma_conf.ahbm_fifo_rst = 0;
            SPI3.dma_conf.ahbm_rst = 1;
            SPI3.dma_conf.ahbm_rst = 0;
            SPI3.dma_conf.out_eof_mode = 1;
            SPI3.cmd.usr = 0;

            SPI3.dma_int_clr.val = ~0;
            SPI3.dma_int_ena.out_eof = 1;
        } else {
            // Configure sampling rate
            I2S0.sample_rate_conf.tx_bck_div_num = 40000000 / config->lcd.fre; // Fws = Fbck / 2
            I2S0.sample_rate_conf.tx_bits_mod = (config->lcd.width == 8) ? 0 : 1;
            // Configuration data format
            I2S0.conf.tx_start = 0;
            I2S0.conf.tx_reset = 1;
            I2S0.conf.tx_reset = 0;
            I2S0.conf.tx_fifo_reset = 1;
            I2S0.conf.tx_fifo_reset = 0;
            I2S0.conf.tx_slave_mod = 0;
            I2S0.conf.tx_right_first = 1; // Must be set to 1, otherwise the clock line will change during reset
            I2S0.conf.tx_msb_right = 0;
            I2S0.conf.tx_short_sync = 0;
            I2S0.conf.tx_mono = 0;
            I2S0.conf.tx_msb_shift = 0;

            I2S0.conf1.tx_pcm_bypass = 1;
            I2S0.conf1.tx_stop_en = 1;

            I2S0.conf_chan.tx_chan_mod = 1;

            I2S0.fifo_conf.tx_fifo_mod_force_en = 1;
            I2S0.fifo_conf.tx_data_num = 32;
            I2S0.fifo_conf.tx_fifo_mod = 1;

            I2S0.lc_conf.out_rst  = 1;
            I2S0.lc_conf.out_rst  = 0;

            I2S0.int_ena.out_eof = 1;
        }
    }

    if (lcd_cam_obj->cam_en) {
        // Configure sampling rate
        I2S0.sample_rate_conf.rx_bck_div_num = 1;
        I2S0.sample_rate_conf.rx_bits_mod = (config->cam.width == 8) ? 0 : 1;
        // Configuration data format
        I2S0.conf.rx_start = 0;
        I2S0.conf.rx_reset = 1;
        I2S0.conf.rx_reset = 0;
        I2S0.conf.rx_fifo_reset = 1;
        I2S0.conf.rx_fifo_reset = 0;
        I2S0.conf.rx_slave_mod = 1;
        I2S0.conf.rx_right_first = 0;
        I2S0.conf.rx_msb_right = 0;
        I2S0.conf.rx_short_sync = 0;
        I2S0.conf.rx_mono = 0;
        I2S0.conf.rx_msb_shift = 0;

        I2S0.conf1.rx_pcm_bypass = 1;

        I2S0.conf_chan.rx_chan_mod = 1;

        I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
        I2S0.fifo_conf.rx_data_num = 32;
        I2S0.fifo_conf.rx_fifo_mod = 1;

        I2S0.lc_conf.in_rst  = 1;
        I2S0.lc_conf.in_rst  = 0;

        I2S0.conf.rx_start = 1;
    }
    return ESP_OK;
}

static esp_err_t lcd_set_pin(lcd_config_t *config)
{
    if (config->width == 1) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.clk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.clk, GPIO_FLOATING);
        gpio_matrix_out(config->pin.clk, VSPICLK_OUT_IDX, config->invert.clk, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[0]], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.data[0], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.data[0], GPIO_FLOATING);
        gpio_matrix_out(config->pin.data[0], VSPID_OUT_IDX, config->invert.data[0], false);
    } else if (config->width <= LCD_DATA_MAX_WIDTH && config->width % 8 == 0){
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.clk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.clk, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(config->pin.clk, GPIO_FLOATING);
        gpio_matrix_out(config->pin.clk, I2S0O_WS_OUT_IDX, !config->invert.clk, false);

        for(int i = 0; i < config->width; i++) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[i]], PIN_FUNC_GPIO);
            gpio_set_direction(config->pin.data[i], GPIO_MODE_OUTPUT);
            gpio_set_pull_mode(config->pin.data[i], GPIO_FLOATING);
            // High bit aligned, OUT23 is always the highest bit
            gpio_matrix_out(config->pin.data[i], I2S0O_DATA_OUT0_IDX + (LCD_DATA_MAX_WIDTH - config->width) + i, config->invert.data[i], false);
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
         ESP_LOGE(TAG, "ESP32 only supports 2-byte aligned data length");
         return ESP_FAIL;
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

    lcd_cam_obj->lcd.dma    = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->lcd.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    lcd_cam_obj->lcd.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->lcd.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    return ESP_OK;
}

static void cam_vsync_intr_enable(bool en)
{
    if (en) {
        gpio_intr_enable(lcd_cam_obj->cam.vsync_pin);
    } else {
        gpio_intr_disable(lcd_cam_obj->cam.vsync_pin);
    }
}

static void cam_dma_stop(void)
{
    if (I2S0.int_ena.in_suc_eof == 1) {
        I2S0.conf.rx_start = 0;
        I2S0.int_ena.in_suc_eof = 0;
        I2S0.int_clr.in_suc_eof = 1;
        I2S0.in_link.stop = 1;
    }
}

static void cam_dma_start(void)
{
    if (I2S0.int_ena.in_suc_eof == 0) {
        I2S0.conf.rx_start = 0;
        I2S0.int_clr.in_suc_eof = 1;
        I2S0.int_ena.in_suc_eof = 1;
        I2S0.conf.rx_reset = 1;
        I2S0.conf.rx_reset = 0;
        I2S0.conf.rx_fifo_reset = 1;
        I2S0.conf.rx_fifo_reset = 0;
        I2S0.lc_conf.in_rst = 1;
        I2S0.lc_conf.in_rst = 0;
        I2S0.lc_conf.ahbm_fifo_rst = 1;
        I2S0.lc_conf.ahbm_fifo_rst = 0;
        I2S0.lc_conf.ahbm_rst = 1;
        I2S0.lc_conf.ahbm_rst = 0;
        I2S0.in_link.start = 1;
        I2S0.conf.rx_start = 1;
        if(lcd_cam_obj->cam.jpeg_mode) {
            // Vsync the first frame manually
            gpio_matrix_in(lcd_cam_obj->cam.vsync_pin, I2S0I_V_SYNC_IDX, lcd_cam_obj->cam.vsync_invert);
            gpio_matrix_in(lcd_cam_obj->cam.vsync_pin, I2S0I_V_SYNC_IDX, !lcd_cam_obj->cam.vsync_invert);
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
        for (int x = 0; x < len; x += 4) {
            out[(x >> 1) + 0] = in[x + 1];
            out[(x >> 1) + 1] = in[x + 3];
        }
    } else {
        for (int x = 0; x < len; x += 4) {
            out[(x >> 1) + 1] = in[x + 1];
            out[(x >> 1) + 0] = in[x + 3];
        }
    }  
}

//Copy fram from DMA dma_buffer to fram dma_buffer
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
                    cam_memcpy(&lcd_cam_obj->cam.frame_buffer[(frame_pos * lcd_cam_obj->cam.recv_size) + cnt * (lcd_cam_obj->cam.dma_half_buffer_size>>1)], &lcd_cam_obj->cam.dma_buffer[(cnt % 2) * lcd_cam_obj->cam.dma_half_buffer_size], lcd_cam_obj->cam.dma_half_buffer_size);
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
                        frame_buffer_event.len = (cnt + 1) * (lcd_cam_obj->cam.dma_half_buffer_size>>1);
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
        gpio_config_t io_conf = {0};
        io_conf.intr_type = config->invert.vsync ? GPIO_PIN_INTR_NEGEDGE : GPIO_PIN_INTR_POSEDGE;
        io_conf.pin_bit_mask = 1ULL << config->pin.vsync; 
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = 1;
        io_conf.pull_down_en = 0;
        gpio_config(&io_conf);
        gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
        gpio_isr_handler_add(config->pin.vsync, cam_vsync_isr, NULL);
        gpio_intr_disable(config->pin.vsync);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.pclk], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.pclk, GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.pclk, GPIO_FLOATING);
        gpio_matrix_in(config->pin.pclk, I2S0I_WS_IN_IDX, config->invert.pclk);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.vsync], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.vsync, GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.vsync, GPIO_FLOATING);
        gpio_matrix_in(config->pin.vsync, I2S0I_V_SYNC_IDX, !config->invert.vsync);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.href], PIN_FUNC_GPIO);
        gpio_set_direction(config->pin.href, GPIO_MODE_INPUT);
        gpio_set_pull_mode(config->pin.href, GPIO_FLOATING);
        gpio_matrix_in(config->pin.href, I2S0I_H_SYNC_IDX, config->invert.href);

        for(int i = 0; i < config->width; i++) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin.data[i]], PIN_FUNC_GPIO);
            gpio_set_direction(config->pin.data[i], GPIO_MODE_INPUT);
            gpio_set_pull_mode(config->pin.data[i], GPIO_FLOATING);
            // High bit alignment, IN16 is always the highest bit
            gpio_matrix_in(config->pin.data[i], I2S0I_DATA_IN0_IDX + (CAM_DATA_MAX_WIDTH - config->width) + i, config->invert.data[i]);
        }

        ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_1_BIT,
            .freq_hz = config->fre,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_1
        };
        ledc_timer_config(&ledc_timer);
        ledc_channel_config_t ledc_channel = {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 1,
            .gpio_num   = config->pin.xclk,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_1,
            .hpoint     = 0
        };
        ledc_channel_config(&ledc_channel);

        gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false);
    } else {
        ESP_LOGE(TAG, "cam width wrong!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t cam_dma_config(cam_config_t *config) 
{
    if (config->recv_size % 2 != 0) {
         ESP_LOGE(TAG, "ESP32 only supports 2-byte aligned data length");
         return ESP_FAIL;
    }
    int cnt = 0;
    uint32_t recv_size = config->recv_size * 2; // ESP32 4byte data-> 2byte valid data 
    if (config->mode.jpeg) {
        lcd_cam_obj->cam.dma_buffer_size = 2048;
        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        lcd_cam_obj->cam.dma_node_buffer_size = lcd_cam_obj->cam.dma_half_buffer_size;
    } else {
        for (cnt = 0; cnt < config->max_dma_buffer_size; cnt++) { // Find a buffer size that can be divisible by
            if ((recv_size % (config->max_dma_buffer_size - cnt) == 0) && (((config->max_dma_buffer_size - cnt) / 2) % 4 == 0)) {
                break;
            }
        }

        if (config->max_dma_buffer_size - cnt <= 4) {
            ESP_LOGE(TAG, "Can't find suitable dma_buffer_size");
            return ESP_FAIL;
        }

        lcd_cam_obj->cam.dma_buffer_size = config->max_dma_buffer_size - cnt;
        lcd_cam_obj->cam.dma_half_buffer_size = lcd_cam_obj->cam.dma_buffer_size / 2;
        for (cnt = 0; cnt < LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE; cnt++) { // Find a divisible dma size
            if (((lcd_cam_obj->cam.dma_half_buffer_size) % (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt) == 0)  && ((LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt) % 4 == 0)) {
                break;
            }
        }

        if (LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt <= 4) {
            ESP_LOGE(TAG, "Can't find suitable dma_node_buffer_size");
            return ESP_FAIL;
        }
        lcd_cam_obj->cam.dma_node_buffer_size = LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE - cnt;
    }

    lcd_cam_obj->cam.dma_node_cnt = (lcd_cam_obj->cam.dma_buffer_size) / lcd_cam_obj->cam.dma_node_buffer_size; // Number of DMA nodes
    lcd_cam_obj->cam.frame_copy_cnt = recv_size / lcd_cam_obj->cam.dma_half_buffer_size; // Number of interrupted copies, ping-pong copy

    ESP_LOGI(TAG, "cam_buffer_size: %d, cam_dma_size: %d, cam_dma_node_cnt: %d, cam_total_cnt: %d\n", lcd_cam_obj->cam.dma_buffer_size, lcd_cam_obj->cam.dma_node_buffer_size, lcd_cam_obj->cam.dma_node_cnt, lcd_cam_obj->cam.frame_copy_cnt);

    lcd_cam_obj->cam.dma    = (lldesc_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_node_cnt * sizeof(lldesc_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    lcd_cam_obj->cam.dma_buffer = (uint8_t *)heap_caps_malloc(lcd_cam_obj->cam.dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);

    for (int x = 0; x < lcd_cam_obj->cam.dma_node_cnt; x++) {
        lcd_cam_obj->cam.dma[x].size = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].length = lcd_cam_obj->cam.dma_node_buffer_size;
        lcd_cam_obj->cam.dma[x].eof = 0;
        lcd_cam_obj->cam.dma[x].owner = 1;
        lcd_cam_obj->cam.dma[x].buf = (lcd_cam_obj->cam.dma_buffer + lcd_cam_obj->cam.dma_node_buffer_size * x);
        lcd_cam_obj->cam.dma[x].empty = &lcd_cam_obj->cam.dma[(x + 1) % lcd_cam_obj->cam.dma_node_cnt];
    }

    I2S0.in_link.addr = ((uint32_t)&lcd_cam_obj->cam.dma[0]) & 0xfffff;
    I2S0.rx_eof_num = lcd_cam_obj->cam.dma_half_buffer_size / 4; // Ping-pong operation, ESP32 4Byte
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
        gpio_isr_handler_remove(lcd_cam_obj->cam.vsync_pin);
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
    
    if (lcd_cam_obj->spi_intr_handle) {
        esp_intr_free(lcd_cam_obj->spi_intr_handle);
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
        if (DPORT_REG_GET_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI3_CLK_EN) == 0) {
            DPORT_REG_CLR_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI3_CLK_EN);
            DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI3_CLK_EN);
            DPORT_REG_SET_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);
            DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI3_RST);
            DPORT_REG_CLR_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_DMA_CLK_EN);
            DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_DMA_CLK_EN);
            DPORT_REG_SET_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);
            DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, DPORT_SPI_DMA_RST);
            DPORT_SET_PERI_REG_BITS(DPORT_SPI_DMA_CHAN_SEL_REG, DPORT_SPI3_DMA_CHAN_SEL, 1, DPORT_SPI3_DMA_CHAN_SEL_S); // ESP32 SPI DMA Channel config, channel 0, SPI3
        }
    }

    periph_module_enable(PERIPH_I2S0_MODULE);
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

        switch (config->lcd.width) {
            case 1: {
                handle->lcd.write_data = spi_write_data;
            }
            break;

            case 8: {
                handle->lcd.write_data = i2s_write_8bit_data;
            }
            break;

            case 16: {
                handle->lcd.write_data = i2s_write_16bit_data;
            }
            break;
        }
        
        handle->lcd.swap_data = lcd_swap_data;
        ESP_LOGI(TAG, "lcd init ok");
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
        ESP_LOGI(TAG, "cam init ok");
    }

    if (lcd_cam_obj->lcd_en || lcd_cam_obj->cam_en) {
        ret |= esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, i2s_isr, NULL, &lcd_cam_obj->lcd_cam_intr_handle);
        if (lcd_cam_obj->lcd_en && config->lcd.width == 1) {
            ret |= esp_intr_alloc(ETS_SPI3_DMA_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, spi_isr, NULL, &lcd_cam_obj->spi_intr_handle);
            ret |= esp_intr_enable(lcd_cam_obj->spi_intr_handle);
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "lcd_cam intr alloc fail!");
            lcd_cam_deinit(handle);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}
