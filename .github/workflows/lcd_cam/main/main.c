#include <stdio.h>
#include <stdint.h>
#include "lcd_cam.h"

void app_main() 
{
    lcd_cam_handle_t lcd_cam;
    lcd_cam_config_t lcd_cam_config = {
        .lcd = {
            .en = false,
        },
        .cam = {
            .en = true,
        }
    };

    lcd_cam_init(&lcd_cam, &lcd_cam_config);
}