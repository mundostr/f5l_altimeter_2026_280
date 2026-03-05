#pragma once

#include "config.h"

void enable_nvs() {
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    ESP_ERROR_CHECK(ret);
}

void disable_radio() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());

    printf("2.4 radio disabled\n");
}

void enable_i2c() {
    ESP_ERROR_CHECK(i2cdev_init());
    
    printf("I2C enabled\n");
}

float limit_vertical_speed(float altitude) {
    float dt = ALTITUDE_REPORT_FREQ / 1000.0f;
    float max_up = MAX_CLIMB_RATE_MPS * dt;
    float max_down = MAX_SINK_RATE_MPS * dt;
    float delta = altitude - altitude_last;

    if (delta > max_up) altitude = altitude_last + max_up;
    if (delta < -max_down) altitude = altitude_last - max_down;

    altitude_last = altitude;

    return altitude;
}

/* float calculate_altitude(float pressure_pa) {
    float altitude = (g_pressure_zero - pressure_pa) / 12.0f;

    if (altitude < 0.0f) altitude = 0.0f;

    return altitude;
} */

float calculate_altitude(float pressure_pa) {
    float altitude = 44330.0f * (1.0f - powf(pressure_pa / g_pressure_zero, 0.190295f));

    if (altitude < 0.0f) altitude = 0.0f;

    return altitude;
}

float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    
    return b;
}

float median_filter(float new_alt) {
    alt_buf[alt_idx] = new_alt;
    alt_idx = (alt_idx + 1) % 3;

    return median3(alt_buf[0], alt_buf[1], alt_buf[2]);
}

float filter_altitude(float new_alt) {
    g_altitude_filtered += alpha_filter * (new_alt - g_altitude_filtered);

    return g_altitude_filtered;
}

void calibrate_zero(bmp280_t *dev) {
    const int discard = 20; // discard unstable readings
    const int samples = 80; // valid samples used for average

    float pressure, temperature, humidity;
    float values[samples];

    int valid = 0;
    int total = discard + samples;

    for (int i = 0; i < total; i++) {
        if (bmp280_read_float(dev, &temperature, &pressure, &humidity) == ESP_OK) {
            if (i >= discard) values[valid++] = pressure;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    for (int i = 0; i < valid - 1; i++) {
        for (int j = i + 1; j < valid; j++) {
            if (values[j] < values[i]) {
                float t = values[i];
                values[i] = values[j];
                values[j] = t;
            }
        }
    }

    // discard 10% extremes
    int start = valid * 0.1;
    int end   = valid * 0.9;

    float sum = 0;
    int count = 0;

    for (int i = start; i < end; i++) {
        sum += values[i];
        count++;
    }

    g_pressure_zero = sum / count;

    printf("Ground pressure calibrated: %.2f Pa\n", g_pressure_zero);
}

void update_peak_altitude(float altitude) {
    if (!g_motor_enabled) return;
    if (altitude < 2.0f) return; // ignore ground noise
    if (altitude > g_altitude_peak) g_altitude_peak = altitude;
}

void motor_start() {
    g_motor_enabled = true;
    g_altitude_peak = 0.0f;
    g_motor_start_time = esp_timer_get_time(); // us

    printf("Motor START\n");
}

void motor_stop() {
    if (!g_motor_enabled) return;

    g_motor_enabled = false;
    esc_ramp_active = true;
    esc_ramp_value = rx_pulse_us;

    printf("Motor STOP\n");
}

void check_rules(float altitude) {
    if (!g_motor_enabled) return;

    int64_t now = esp_timer_get_time();
    int64_t elapsed_ms = (now - g_motor_start_time) / 1000;

    if (elapsed_ms >= MOTOR_MAX_TIME_MS) {
        motor_stop();
        printf("Motor stop: TIME LIMIT\n");

        return;
    }

    if (altitude >= ALTITUDE_LIMIT_M) {
        motor_stop();
        printf("Motor stop: ALTITUDE LIMIT\n");

        return;
    }
}

uint16_t read_throttle_us() {
    // placeholder, will return the measured PWM width from CH3

    return 1000;
}

void learn_throttle_range() {
    const int samples = 30;
    uint32_t sum = 0;

    vTaskDelay(pdMS_TO_TICKS(100));

    for(int i = 0; i < samples; i++) {
        uint16_t v = read_throttle_us();
        sum += v;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    throttle_min = sum / samples;
    if(throttle_min < DEFAULT_THROTTLE_MIN - 200 || throttle_min > DEFAULT_THROTTLE_MIN + 200) {
        throttle_min = DEFAULT_THROTTLE_MIN;
        printf("Throttle learn failed, using default\n");
    }
    throttle_max = throttle_min + DEFAULT_THROTTLE_MIN;

    printf("Throttle learned MIN: %u us\n", throttle_min);
    printf("Throttle learned MAX: %u us\n", throttle_max);
}

void update_throttle_arm(uint16_t pulse) {
    static bool was_low = false;

    if (pulse <= throttle_min + 20) was_low = true;
    if (was_low && pulse > throttle_min + 100) throttle_armed = true;
}

bool pwm_rx_callback(rmt_channel_handle_t chan, const rmt_rx_done_event_data_t *edata, void *user_ctx) {
    if (edata->num_symbols < 1) return false;

    rmt_symbol_word_t sym = edata->received_symbols[0];

    uint32_t high_time = sym.duration0;

    rx_pulse_us = high_time;
    last_rx_time = esp_timer_get_time();

    return false;
}

void enable_pwm_input() {
    rmt_rx_channel_config_t rx_config = {
        .gpio_num = RX_PWM_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 64,
    };
    rmt_new_rx_channel(&rx_config, &rx_chan);

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = pwm_rx_callback,
    };

    rmt_rx_register_event_callbacks(rx_chan, &cbs, NULL);

    rmt_enable(rx_chan);

    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 500000,
        .signal_range_max_ns = 2500000,
    };

    static rmt_symbol_word_t rx_buffer[64];

    rmt_receive(rx_chan, rx_buffer, sizeof(rx_buffer), &receive_config);

    printf("PWM RX input enabled\n");
}

void enable_pwm_output() {
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = ESC_PWM_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    rmt_new_tx_channel(&tx_config, &esc_chan);
    rmt_copy_encoder_config_t encoder_config = {};
    rmt_new_copy_encoder(&encoder_config, &esc_encoder);
    rmt_enable(esc_chan);

    printf("PWM ESC output enabled\n");
}

void send_pwm(uint16_t pulse_us) {
    rmt_symbol_word_t symbol;

    symbol.level0 = 1;
    symbol.duration0 = pulse_us;

    symbol.level1 = 0;
    symbol.duration1 = 20000 - pulse_us;

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0
    };

    rmt_transmit(esc_chan, esc_encoder, &symbol, sizeof(symbol), &tx_cfg);
    rmt_tx_wait_all_done(esc_chan, 1000);
}

bool pwm_failsafe() {
    int64_t now = esp_timer_get_time();

    if ((now - last_rx_time) > FAILSAFE_TIMEOUT_US) return true;

    return false;
}

void bmp280_task(void *pvParameters) {
    bmp280_params_t params = {};
    params.mode = BMP280_MODE_NORMAL;
    params.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
    params.oversampling_temperature = BMP280_STANDARD;
    params.filter = BMP280_FILTER_16;
    params.standby = BMP280_STANDBY_62;
    bmp280_init_default_params(&params);
    
    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));
    bool bme280p = dev.id == BME280_CHIP_ID;
    float pressure, temperature, humidity;

    ESP_ERROR_CHECK(bmp280_init_desc(&dev, BMP280_I2C_ADDRESS_0, (i2c_port_t)0, MASTER_SDA_PIN, MASTER_SCL_PIN));
    ESP_ERROR_CHECK(bmp280_init(&dev, &params));
    
    printf("BMP280: found %s\n", bme280p ? "BME280" : "BMP280");

    calibrate_zero(&dev);
    // motor_start(); // DISABLE!, just for testing

    for(;;) {
        if (bmp280_read_float(&dev, &temperature, &pressure, &humidity) != ESP_OK) {
            printf("Reading failed\n");
            continue;
        }

        float altitude_raw = calculate_altitude(pressure);
        float altitude_med = median_filter(altitude_raw);
        float altitude_flt = filter_altitude(altitude_med);
        float altitude = limit_vertical_speed(altitude_flt);
        update_peak_altitude(altitude);
        check_rules(altitude);
        
        printf("Alt %.2f, Pre %.2f, Tmp %.2f, Pck: %.2f\n", altitude, pressure, temperature, g_altitude_peak);
        
        vTaskDelay(pdMS_TO_TICKS(ALTITUDE_REPORT_FREQ));
    }
}

void enable_bmp280() {
    xTaskCreatePinnedToCore(bmp280_task, "bmp280_task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

void esc_task(void *arg) {
    for(;;) {
        uint16_t pulse = rx_pulse_us;

        update_throttle_arm(pulse);

        if (pwm_failsafe()) {
            send_pwm(throttle_min);
        } else if (esc_ramp_active) {
            if (esc_ramp_value > throttle_min + ESC_RAMP_STEP) {
                esc_ramp_value -= ESC_RAMP_STEP;
            } else {
                esc_ramp_value = throttle_min;
                esc_ramp_active = false;
            }

            send_pwm(esc_ramp_value);
        } else if (!g_motor_enabled || !throttle_armed) {
            send_pwm(throttle_min);
        } else {
            send_pwm(pulse);
        }

        vTaskDelay(pdMS_TO_TICKS(ESC_UPDATE_PERIOD_MS));
    }
}

void enable_esc() {
    xTaskCreatePinnedToCore(esc_task, "esc_task", 4096, NULL, 5, NULL, APP_CPU_NUM);
}
