#include "config.h"
#include "main.h"

extern "C" void app_main(void) {
    enable_nvs();
    disable_radio();
    enable_i2c();

    enable_pwm_input();
    enable_pwm_output();
    learn_throttle_range();
    
    enable_bmp280();
    enable_esc();

    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
