#include <stdio.h>
#include <stdint.h>
#include "lcd_cam.h"

lcd_cam_handle_t lcd_cam;

void app_main() 
{
    lcd_cam_config_t lcd_cam_config = {
#if LCD_DATA_MAX_WIDTH
        .lcd = {
            .en = false,
        },
#endif
#if CAM_DATA_MAX_WIDTH
        .cam = {
            .en = false,
        }
#endif
    };

    lcd_cam_init(&lcd_cam, &lcd_cam_config);
}