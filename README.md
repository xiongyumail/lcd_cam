# lcd_cam

[![CI](https://github.com/xiongyumail/lcd_cam/actions/workflows/main.yml/badge.svg)](https://github.com/xiongyumail/lcd_cam/actions/workflows/main.yml)

lcd & cam driver for esp32, esp32s2, esp32c3, esp32s3

1. Add lcd_cam to your project components

```bash
mkdir -p components
cd components
git submodule add https://github.com/xiongyumail/lcd_cam.git
```

2. Add lcd_cam header file

```c
#include "lcd_cam.h"
...

lcd_cam_handle_t lcd_cam;

void app_main() 
{
    lcd_cam_config_t lcd_cam_config = {
        ...
    };

    lcd_cam_init(&lcd_cam, &lcd_cam_config);
}
```