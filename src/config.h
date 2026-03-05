#pragma once

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "bmp280.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#define RX_PWM_PIN                  GPIO_NUM_2
#define ESC_PWM_PIN                 GPIO_NUM_3
#define MASTER_SDA_PIN              GPIO_NUM_6
#define MASTER_SCL_PIN              GPIO_NUM_7

#define ALTITUDE_REPORT_FREQ        200
#define MOTOR_MAX_TIME_MS           30000
#define ALTITUDE_LIMIT_M            90.0f
#define SAFE_THROTTLE_MIN           800
#define SAFE_THROTTLE_MAX           2200
#define DEFAULT_THROTTLE_MIN        1000
#define DEFAULT_THROTTLE_MAX        2000
#define FAILSAFE_TIMEOUT_US         100000
#define ESC_UPDATE_PERIOD_MS        5
#define MAX_CLIMB_RATE_MPS          15.0f
#define MAX_SINK_RATE_MPS           20.0f
#define CALIBRATION_INIT_DELAY_MS   1500
#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

static const char *TAG = "ALTMTR280";
const float alpha_filter = 0.15f;

static float g_temperature_zero = 15.0f;
static float g_pressure_zero = 0.0f;
static float g_pressure_filtered = 0.0f;
static float g_altitude_filtered = 0.0f;
static float g_altitude_peak = 0.0f;
static bool g_motor_enabled = false;
static int64_t g_motor_start_time = 0;
static float alt_buf[3] = {0};
static int alt_idx = 0;
static float altitude_last = 0.0f;
uint16_t throttle_min = DEFAULT_THROTTLE_MAX; // updated later according to throttle pulse
uint16_t throttle_max = DEFAULT_THROTTLE_MAX;
static volatile uint16_t rx_pulse_us = DEFAULT_THROTTLE_MIN;
static volatile int64_t last_rx_time = 0;
static rmt_channel_handle_t rx_chan = NULL;
static rmt_channel_handle_t esc_chan = NULL;
static bool esc_ramp_active = false;
static uint16_t esc_ramp_value = 0;
static const uint16_t ESC_RAMP_STEP = 20; // µs per cycle
static bool throttle_armed = false;
static rmt_encoder_handle_t esc_encoder = NULL;
