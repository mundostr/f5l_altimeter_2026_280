#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

/* ─── Pin definitions ───────────────────────────────────────────────────── */
#define PIN_SCK                 GPIO_NUM_10 // GPIO_NUM_8
#define PIN_MISO                GPIO_NUM_20 // GPIO_NUM_9
#define PIN_MOSI                GPIO_NUM_9 // GPIO_NUM_10
#define PIN_CS                  GPIO_NUM_8 // GPIO_NUM_20
#define THR_IN_PIN              GPIO_NUM_3
#define ESC_OUT_PIN             GPIO_NUM_21

/* ─── BMP280 registers ──────────────────────────────────────────────────── */
#define BMP280_REG_CALIB_START  0x88
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_RESET_VALUE      0xB6

/* osrs_t=001(x1), osrs_p=100(x8), mode=11(normal) → ~26 Hz ODR
   t_sb=000(0.5ms), filter=100(coeff 16), spi3w=0                  */
#define BMP280_CTRL_MEAS_VAL    0x37
#define BMP280_CONFIG_VAL       0x10

/* ─── F5L competition rules ─────────────────────────────────────────────── */
#define MAX_ALTITUDE_M          90.0f   /* competition ceiling, meters      */
#define MAX_MOTOR_TIME_S        30      /* max motor run time, seconds       */

/* ─── PWM input config ──────────────────────────────────────────────────── */
#define PWM_MIN_US              1000    /* pulse at 0% throttle (µs)        */
#define PWM_MAX_US              2000    /* pulse at 100% throttle (µs)      */
#define PWM_TIMEOUT_MS          100     /* signal lost if no pulse this long */
#define FULLDOWN_DEADBAND_US    50      /* µs above PWM_MIN_US still = down */
#define FULLDOWN_DEBOUNCE_MS    200     /* stick must stay down this long    */
#define LAUNCH_THROTTLE_PCT     40      /* throttle % that triggers launch   */

/* ─── RMT ESC output config ─────────────────────────────────────────────── */
#define RMT_RESOLUTION_HZ       1000000 /* 1 MHz → 1 tick = 1 µs           */
#define ESC_FRAME_PERIOD_US     20000   /* 50 Hz RC frame = 20ms            */

/* ─── Flight detection ──────────────────────────────────────────────────── */
#define DESCENT_HYSTERESIS_M    2.0f
#define ALT_10S_DELAY_MS        10000

/* ─── Ground calibration ────────────────────────────────────────────────── */
#define CALIB_IIR_WARMUP        40
#define CALIB_IIR_WARMUP_MS     40

/* ─── Drift compensation ────────────────────────────────────────────────── */
#define DRIFT_EMA_ALPHA         0.01f
#define DRIFT_TIMER_PERIOD_MS   30000

/* ─── Serial console report (disable for production flight) ────────────── */
#define SERIAL_REPORT_ENABLED   1       /* 1 = on, 0 = off                  */
#define NVS_NAMESPACE           "f5l_alt"
#define NVS_KEY_FLIGHT          "last_flight"

/* ─── FreeRTOS ──────────────────────────────────────────────────────────── */
#define SAMPLE_TASK_STACK       4096
#define REPORT_TASK_STACK       2048
#define SAMPLE_TASK_PRIO        5
#define REPORT_TASK_PRIO        4
#define ALTITUDE_QUEUE_DEPTH    32
#define SAMPLE_INTERVAL_MS      38      /* ~26 Hz */
#define REPORT_INTERVAL_MS      1000

static const char *TAG = "F5L_ALT";

/* ─── Flight states ─────────────────────────────────────────────────────── */
typedef enum {
    STATE_GROUND   = 0,
    STATE_CLIMBING = 1,
    STATE_COASTING = 2,
} flight_state_t;

/* ─── Cutoff reasons ────────────────────────────────────────────────────── */
typedef enum {
    CUTOFF_NONE     = 0,
    CUTOFF_TIME     = 1,
    CUTOFF_ALTITUDE = 2,
    CUTOFF_MANUAL   = 3,
} cutoff_reason_t;

/* ─── NVS flight record ─────────────────────────────────────────────────── */
typedef struct {
    uint32_t        launch_ms;         /* ms since boot at launch             */
    uint32_t        cutoff_ms;         /* ms since boot at motor cutoff       */
    float           peak_motor_m;      /* highest altitude during motor run   */
    float           peak_flight_m;     /* highest altitude entire flight      */
    float           cutoff_altitude_m; /* altitude at moment of cutoff        */
    float           alt_10s_m;         /* altitude 10s after cutoff           */
    cutoff_reason_t cutoff_reason;
} flight_record_t;

/* ─── BMP280 calibration trim ───────────────────────────────────────────── */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
    int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
} sample_t;

/* ─── Global state ──────────────────────────────────────────────────────── */
static spi_device_handle_t      s_spi              = NULL;
static bmp280_calib_t           s_calib            = {};
static float                    s_p0_pa            = 101325.0f;
static QueueHandle_t            s_alt_queue        = NULL;
static TimerHandle_t            s_drift_timer      = NULL;

static volatile flight_state_t  s_state            = STATE_GROUND;
static volatile float           s_peak_m           = 0.0f;   /* whole flight peak */
static volatile float           s_peak_motor_m     = 0.0f;   /* motor-run peak only */
static volatile TickType_t      s_launch_tick      = 0;
static volatile cutoff_reason_t s_cutoff_reason    = CUTOFF_NONE;
static volatile float           s_cutoff_alt_m     = 0.0f;
static volatile bool            s_alt_10s_recorded = false;

/* ─── PWM input state (ISR-written) ─────────────────────────────────────── */
static volatile int64_t         s_pwm_rise_us      = 0;
static volatile uint32_t        s_pwm_pulse_us     = 0;
static volatile int64_t         s_pwm_last_us      = 0;

/* Learned PWM min — captured during IIR warmup, falls back to PWM_MIN_US */
static uint32_t                 s_pwm_min_us       = PWM_MIN_US;

/* ─── Full-down debounce state ──────────────────────────────────────────── */
static volatile int64_t         s_fulldown_since_us = 0; /* when stick went down */

/* ─── RMT ESC output ────────────────────────────────────────────────────── */
static rmt_channel_handle_t     s_rmt_chan          = NULL;
static rmt_encoder_handle_t     s_rmt_encoder       = NULL;

/* ══════════════════════════════════════════════════════════════════════════
   PWM input — GPIO ISR + esp_timer_get_time()
   ══════════════════════════════════════════════════════════════════════════ */

static void IRAM_ATTR pwm_gpio_isr(void *arg)
{
    int64_t now_us = esp_timer_get_time();
    if (gpio_get_level(THR_IN_PIN)) {
        s_pwm_rise_us = now_us;
    } else {
        uint32_t pulse_us = (uint32_t)(now_us - s_pwm_rise_us);
        if (pulse_us >= 800 && pulse_us <= 2200) {
            s_pwm_pulse_us = pulse_us;
            s_pwm_last_us  = now_us;
        }
    }
}

static void pwm_input_init(void)
{
    gpio_config_t io_cfg = {};
    io_cfg.pin_bit_mask = (1ULL << THR_IN_PIN);
    io_cfg.mode         = GPIO_MODE_INPUT;
    io_cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    io_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_cfg.intr_type    = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_cfg));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(THR_IN_PIN, pwm_gpio_isr, NULL));
    ESP_LOGI(TAG, "PWM input ready on GPIO%d", THR_IN_PIN);
}

/* Returns throttle 0–100%, or -1 if signal absent.
 * Uses learned s_pwm_min_us so the range reflects the actual radio. */
static int pwm_throttle_pct(void)
{
    if ((esp_timer_get_time() - s_pwm_last_us) > (PWM_TIMEOUT_MS * 1000LL))
        return -1;
    uint32_t pulse = s_pwm_pulse_us;
    if (pulse <= s_pwm_min_us) return 0;
    if (pulse >= (uint32_t)PWM_MAX_US) return 100;
    return (int)(((pulse - s_pwm_min_us) * 100) / (PWM_MAX_US - s_pwm_min_us));
}

/* Returns true if stick is in the full-down deadband */
static bool pwm_is_fulldown(void)
{
    if ((esp_timer_get_time() - s_pwm_last_us) > (PWM_TIMEOUT_MS * 1000LL))
        return false;
    return s_pwm_pulse_us <= (s_pwm_min_us + FULLDOWN_DEADBAND_US);
}

/* ══════════════════════════════════════════════════════════════════════════
   RMT ESC output
   Uses a simple copy encoder: we hand it a pre-built rmt_symbol_word_t
   representing a single PWM pulse (high phase + low phase) each frame.

   RC PWM frame = 20ms (50Hz):
     high phase = pulse_us  (1000–2000µs)
     low phase  = 20000 - pulse_us µs

   RMT resolution = 1MHz → duration values are directly in µs.
   ══════════════════════════════════════════════════════════════════════════ */

static void esc_output_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num            = ESC_OUT_PIN;
    tx_cfg.clk_src             = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz       = RMT_RESOLUTION_HZ;
    tx_cfg.mem_block_symbols   = 64;
    tx_cfg.trans_queue_depth   = 4;
    tx_cfg.flags.invert_out    = false;
    tx_cfg.flags.with_dma      = false;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_rmt_chan));

    /* Use the built-in copy encoder: we supply raw rmt_symbol_word_t */
    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &s_rmt_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));
    ESP_LOGI(TAG, "RMT ESC output ready on GPIO%d", ESC_OUT_PIN);
}

/**
 * Send a single RC PWM pulse to the ESC.
 * Raw passthrough — we never clamp or scale the pulse value, so the ESC
 * always receives exactly what the receiver sent. The only safety clamp is
 * a wide 800–2200µs sanity bound to protect the ESC from garbage values.
 */
static void esc_send_pulse(uint32_t pulse_us)
{
    /* Wide sanity clamp only — never restrict legitimate radio range */
    if (pulse_us < 800)  pulse_us = 800;
    if (pulse_us > 2200) pulse_us = 2200;

    rmt_symbol_word_t symbols[2];

    /* High phase */
    symbols[0].level0    = 1;
    symbols[0].duration0 = (uint16_t)pulse_us;
    symbols[0].level1    = 0;
    symbols[0].duration1 = 0;   /* will be filled by symbol[1] */

    /* Low phase — pad to complete 20ms frame */
    symbols[1].level0    = 0;
    symbols[1].duration0 = (uint16_t)(ESC_FRAME_PERIOD_US - pulse_us);
    symbols[1].level1    = 0;
    symbols[1].duration1 = 0;

    rmt_transmit_config_t tx_cfg = {};
    tx_cfg.loop_count = 0;   /* single shot */

    /* Non-blocking: queue the transmission.
     * If the previous frame hasn't finished, rmt_transmit will wait
     * internally (trans_queue_depth=4 gives us buffer room). */
    rmt_transmit(s_rmt_chan, s_rmt_encoder, symbols, sizeof(symbols), &tx_cfg);
}

/* ══════════════════════════════════════════════════════════════════════════
   NVS — flight record storage
   ══════════════════════════════════════════════════════════════════════════ */

static void nvs_init_storage(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition issue, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void nvs_save_flight(const flight_record_t *rec)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = nvs_set_blob(h, NVS_KEY_FLIGHT, rec, sizeof(flight_record_t));
    if (ret == ESP_OK) ret = nvs_commit(h);
    nvs_close(h);
    if (ret == ESP_OK)
        ESP_LOGI(TAG, "Flight record saved to NVS");
    else
        ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(ret));
}

static bool nvs_load_flight(flight_record_t *rec)
{
    nvs_handle_t h;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (ret != ESP_OK) return false;
    size_t sz = sizeof(flight_record_t);
    ret = nvs_get_blob(h, NVS_KEY_FLIGHT, rec, &sz);
    nvs_close(h);
    return (ret == ESP_OK && sz == sizeof(flight_record_t));
}

static const char *cutoff_reason_str(cutoff_reason_t r)
{
    switch (r) {
        case CUTOFF_TIME:     return "TIME LIMIT (30s)";
        case CUTOFF_ALTITUDE: return "ALT LIMIT (90m)";
        case CUTOFF_MANUAL:   return "MANUAL (pilot)";
        default:              return "UNKNOWN";
    }
}

static void nvs_print_last_flight(void)
{
    flight_record_t rec = {};
    if (!nvs_load_flight(&rec)) {
        printf("═══════════════════════════════════════════\n");
        printf("  No previous flight data found.\n");
        printf("═══════════════════════════════════════════\n\n");
        return;
    }
    uint32_t motor_time_ms = rec.cutoff_ms - rec.launch_ms;
    printf("═══════════════════════════════════════════\n");
    printf("  LAST FLIGHT RECORD\n");
    printf("───────────────────────────────────────────\n");
    printf("  Motor run time : %u.%03u s\n",
           motor_time_ms / 1000, motor_time_ms % 1000);
    printf("  Cutoff reason  : %s\n", cutoff_reason_str(rec.cutoff_reason));
    printf("  Peak (motor)   : %.2f m\n", rec.peak_motor_m);
    printf("  Alt at cutoff  : %.2f m\n", rec.cutoff_altitude_m);
    printf("  Alt at T+10s   : %.2f m\n", rec.alt_10s_m);
    printf("  Peak (flight)  : %.2f m\n", rec.peak_flight_m);
    printf("═══════════════════════════════════════════\n\n");
}

/* ══════════════════════════════════════════════════════════════════════════
   SPI helpers (DMA-safe static buffers)
   ══════════════════════════════════════════════════════════════════════════ */

static esp_err_t bmp280_write_reg(uint8_t reg, uint8_t value)
{
    static DRAM_ATTR uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = value;
    spi_transaction_t t = {};
    t.length    = 16;
    t.tx_buffer = tx;
    return spi_device_transmit(s_spi, &t);
}

static esp_err_t bmp280_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    static DRAM_ATTR uint8_t tx_buf[33];
    static DRAM_ATTR uint8_t rx_buf[33];
    if (len > 32) return ESP_ERR_INVALID_SIZE;
    memset(tx_buf, 0x00, len + 1);
    tx_buf[0] = reg | 0x80;
    spi_transaction_t t = {};
    t.length    = (len + 1) * 8;
    t.tx_buffer = tx_buf;
    t.rx_buffer = rx_buf;
    esp_err_t ret = spi_device_transmit(s_spi, &t);
    if (ret == ESP_OK) memcpy(data, &rx_buf[1], len);
    return ret;
}

/* ══════════════════════════════════════════════════════════════════════════
   BMP280 compensation (datasheet section 8.2, integer path)
   ══════════════════════════════════════════════════════════════════════════ */

static int32_t s_t_fine = 0;

static float bmp280_compensate_temperature(int32_t adc_T)
{
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) *
                    ((int32_t)s_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
                    ((int32_t)s_calib.dig_T3)) >> 14;
    s_t_fine = var1 + var2;
    return (float)((s_t_fine * 5 + 128) >> 8) / 100.0f;
}

static float bmp280_compensate_pressure(int32_t adc_P)
{
    int64_t var1 = (int64_t)s_t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 += (var1 * (int64_t)s_calib.dig_P5) << 17;
    var2 += (int64_t)s_calib.dig_P4 << 35;
    var1  = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) +
            ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1  = (((int64_t)1 << 47) + var1) * (int64_t)s_calib.dig_P1 >> 33;
    if (var1 == 0) return 0.0f;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)s_calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)s_calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)s_calib.dig_P7 << 4);
    return (float)p / 256.0f;
}

/* ══════════════════════════════════════════════════════════════════════════
   Altitude — hypsometric, AGL from boot ground level
   ══════════════════════════════════════════════════════════════════════════ */

static float pressure_to_altitude(float pressure_pa, float temp_c)
{
    if (pressure_pa <= 0.0f || s_p0_pa <= 0.0f) return 0.0f;
    return 29.271f * (temp_c + 273.15f) * logf(s_p0_pa / pressure_pa);
}

/* ══════════════════════════════════════════════════════════════════════════
   BMP280 init
   ══════════════════════════════════════════════════════════════════════════ */

static esp_err_t bmp280_init(void)
{
    uint8_t id = 0;
    ESP_ERROR_CHECK(bmp280_read_regs(BMP280_REG_ID, &id, 1));
    if (id != 0x60 && id != 0x58) {
        ESP_LOGE(TAG, "BMP280 not found! chip_id=0x%02X", id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMP280 chip_id=0x%02X", id);
    ESP_ERROR_CHECK(bmp280_write_reg(BMP280_REG_RESET, BMP280_RESET_VALUE));
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t raw[24];
    ESP_ERROR_CHECK(bmp280_read_regs(BMP280_REG_CALIB_START, raw, 24));
    s_calib.dig_T1 = (uint16_t)(raw[1]  << 8 | raw[0]);
    s_calib.dig_T2 = (int16_t) (raw[3]  << 8 | raw[2]);
    s_calib.dig_T3 = (int16_t) (raw[5]  << 8 | raw[4]);
    s_calib.dig_P1 = (uint16_t)(raw[7]  << 8 | raw[6]);
    s_calib.dig_P2 = (int16_t) (raw[9]  << 8 | raw[8]);
    s_calib.dig_P3 = (int16_t) (raw[11] << 8 | raw[10]);
    s_calib.dig_P4 = (int16_t) (raw[13] << 8 | raw[12]);
    s_calib.dig_P5 = (int16_t) (raw[15] << 8 | raw[14]);
    s_calib.dig_P6 = (int16_t) (raw[17] << 8 | raw[16]);
    s_calib.dig_P7 = (int16_t) (raw[19] << 8 | raw[18]);
    s_calib.dig_P8 = (int16_t) (raw[21] << 8 | raw[20]);
    s_calib.dig_P9 = (int16_t) (raw[23] << 8 | raw[22]);

    ESP_ERROR_CHECK(bmp280_write_reg(BMP280_REG_CONFIG,    BMP280_CONFIG_VAL));
    ESP_ERROR_CHECK(bmp280_write_reg(BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VAL));
    ESP_LOGI(TAG, "BMP280 ready: osrs_p=x8, osrs_t=x1, IIR=16, ~26Hz");
    return ESP_OK;
}

static esp_err_t bmp280_read_sample(float *pressure_pa, float *temperature_c)
{
    uint8_t data[6];
    esp_err_t ret = bmp280_read_regs(BMP280_REG_PRESS_MSB, data, 6);
    if (ret != ESP_OK) return ret;
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);
    int32_t adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | (data[5] >> 4);
    *temperature_c = bmp280_compensate_temperature(adc_T);
    *pressure_pa   = bmp280_compensate_pressure(adc_P);
    return ESP_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
   Ground zero calibration — with motion detection
   ══════════════════════════════════════════════════════════════════════════

   Phase 1 — IIR warmup (runs once, never repeated):
     Discards CALIB_IIR_WARMUP samples so the sensor's onboard IIR filter
     converges. Takes ~1.6s. Model can be moving during this — doesn't matter.

   Phase 2 — P0 capture (repeats if motion detected):
     Collects GROUND_CALIB_SAMPLES pressure readings and checks their
     standard deviation. If stddev > CALIB_MOTION_THRESHOLD_PA (equivalent
     to ~CALIB_MOTION_THRESHOLD_PA * ~8.5cm per Pa of altitude noise), the
     batch is discarded and we wait CALIB_RETRY_WAIT_MS before trying again.
     Once a still batch is confirmed, P0 is set to the batch mean.

   CALIB_MOTION_THRESHOLD_PA = 1.5 Pa ≈ ~12cm altitude variation across
   the sample window. Tight enough to catch hand-carry, loose enough to
   ignore normal sensor noise (~0.6 Pa RMS at OSR×8).
   ══════════════════════════════════════════════════════════════════════════ */

#define GROUND_CALIB_SAMPLES        10      /* samples per attempt (fast: ~0.5s) */
#define GROUND_CALIB_MS             50      /* interval between samples          */
#define CALIB_MOTION_THRESHOLD_PA   1.5f    /* max allowed stddev across batch   */
#define CALIB_RETRY_WAIT_MS         300     /* wait before retrying after motion */

static void calibrate_ground_zero(void)
{
    /* ── Phase 1: IIR warmup + learn PWM min ─────────────────────────────── */
    ESP_LOGI(TAG, "IIR warmup (%d samples, ~%.1fs) — stick must be full down...",
             CALIB_IIR_WARMUP, CALIB_IIR_WARMUP * CALIB_IIR_WARMUP_MS / 1000.0f);

    uint32_t pwm_min_sum   = 0;
    int      pwm_min_count = 0;

    for (int i = 0; i < CALIB_IIR_WARMUP; i++) {
        float p, t;
        bmp280_read_sample(&p, &t);

        /* Accumulate pulse width samples for min learning.
         * Only count pulses in a sane idle range (800–1300µs).
         * This rejects spurious readings and any accidental stick movement. */
        uint32_t pulse = s_pwm_pulse_us;
        if (pulse >= 800 && pulse <= 1300) {
            pwm_min_sum += pulse;
            pwm_min_count++;
        }

        vTaskDelay(pdMS_TO_TICKS(CALIB_IIR_WARMUP_MS));
    }

    if (pwm_min_count >= CALIB_IIR_WARMUP / 2) {
        s_pwm_min_us = pwm_min_sum / pwm_min_count;
        ESP_LOGI(TAG, "PWM min learned: %luµs (from %d samples)",
                 (unsigned long)s_pwm_min_us, pwm_min_count);
    } else {
        s_pwm_min_us = PWM_MIN_US;   /* fallback to hardcoded default */
        ESP_LOGW(TAG, "PWM min not learned (only %d valid samples) — "
                 "using default %dµs", pwm_min_count, PWM_MIN_US);
    }

    /* ── Phase 2: P0 capture with motion detection ───────────────────────── */
    ESP_LOGI(TAG, "Waiting for stillness to capture P0...");

    while (1) {
        float   samples[GROUND_CALIB_SAMPLES];
        int     valid = 0;
        double  sum   = 0.0;

        /* Collect a batch */
        for (int i = 0; i < GROUND_CALIB_SAMPLES; i++) {
            float p, t;
            if (bmp280_read_sample(&p, &t) == ESP_OK && p > 50000.0f) {
                samples[valid++] = p;
                sum += p;
            }
            vTaskDelay(pdMS_TO_TICKS(GROUND_CALIB_MS));
        }

        if (valid < GROUND_CALIB_SAMPLES / 2) {
            /* Too many bad reads — sensor issue, just keep trying */
            ESP_LOGW(TAG, "Too few valid samples (%d), retrying...", valid);
            vTaskDelay(pdMS_TO_TICKS(CALIB_RETRY_WAIT_MS));
            continue;
        }

        /* Compute mean and standard deviation */
        float mean = (float)(sum / valid);
        float variance = 0.0f;
        for (int i = 0; i < valid; i++) {
            float diff = samples[i] - mean;
            variance += diff * diff;
        }
        float stddev = sqrtf(variance / valid);

        if (stddev > CALIB_MOTION_THRESHOLD_PA) {
            /* Motion detected — discard batch and wait */
            ESP_LOGW(TAG, "Motion detected (stddev=%.3f Pa > %.1f Pa threshold) "
                     "— recalibrating...", stddev, CALIB_MOTION_THRESHOLD_PA);
            vTaskDelay(pdMS_TO_TICKS(CALIB_RETRY_WAIT_MS));
            continue;
        }

        /* Still — lock P0 */
        s_p0_pa = mean;
        ESP_LOGI(TAG, "P0 = %.2f Pa (stddev=%.3f Pa) — zero locked at ground level",
                 s_p0_pa, stddev);
        break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
   Drift compensation — GROUND state only
   ══════════════════════════════════════════════════════════════════════════ */

static void drift_compensation_cb(TimerHandle_t xTimer)
{
    if (s_state != STATE_GROUND) return;
    float p, t;
    if (bmp280_read_sample(&p, &t) != ESP_OK) return;
    float p0_new = DRIFT_EMA_ALPHA * p + (1.0f - DRIFT_EMA_ALPHA) * s_p0_pa;
    ESP_LOGI(TAG, "[DRIFT] P0: %.2f → %.2f Pa (%+.3f Pa)",
             s_p0_pa, p0_new, p0_new - s_p0_pa);
    s_p0_pa = p0_new;
}

/* ══════════════════════════════════════════════════════════════════════════
   SPI init
   ══════════════════════════════════════════════════════════════════════════ */

static void spi_init(void)
{
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num    = PIN_MOSI;
    bus_cfg.miso_io_num    = PIN_MISO;
    bus_cfg.sclk_io_num    = PIN_SCK;
    bus_cfg.quadwp_io_num  = -1;
    bus_cfg.quadhd_io_num  = -1;
    bus_cfg.max_transfer_sz = 64;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode           = 0;
    dev_cfg.clock_speed_hz = 5 * 1000 * 1000;
    dev_cfg.spics_io_num   = PIN_CS;
    dev_cfg.queue_size     = 4;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi));
    ESP_LOGI(TAG, "SPI2+DMA ready, CLK=5MHz");
}

/* ══════════════════════════════════════════════════════════════════════════
   Sampling task — ~26 Hz
   Owns the state machine, ESC output, and flight record writing.
   ══════════════════════════════════════════════════════════════════════════ */

static void bmp280_sample_task(void *pvParam)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    /* Working copy of flight record, written to NVS at cutoff */
    flight_record_t rec = {};

    while (1) {
        sample_t s = {};
        if (bmp280_read_sample(&s.pressure_pa, &s.temperature_c) == ESP_OK) {
            s.altitude_m = pressure_to_altitude(s.pressure_pa, s.temperature_c);

            switch (s_state) {

                /* ── GROUND ──────────────────────────────────────────────── */
                case STATE_GROUND: {
                    esc_send_pulse(PWM_MIN_US);   /* motor always off on ground */

                    int thr = pwm_throttle_pct();
                    if (thr >= LAUNCH_THROTTLE_PCT) {
                        s_state             = STATE_CLIMBING;
                        s_peak_m            = s.altitude_m;
                        s_peak_motor_m      = s.altitude_m;
                        s_launch_tick       = xTaskGetTickCount();
                        s_fulldown_since_us = 0;
                        rec                 = {};
                        rec.launch_ms       = (uint32_t)(esp_timer_get_time() / 1000);
                        ESP_LOGI(TAG, "*** LAUNCH | thr=%d%% | alt=%.2fm ***",
                                 thr, s.altitude_m);
                    }
                    break;
                }

                /* ── CLIMBING ────────────────────────────────────────────── */
                case STATE_CLIMBING: {
                    /* Pass throttle 1:1 to ESC */
                    esc_send_pulse(s_pwm_pulse_us > 0 ? s_pwm_pulse_us : PWM_MIN_US);

                    /* Track both peaks during motor run */
                    if (s.altitude_m > s_peak_m)       s_peak_m       = s.altitude_m;
                    if (s.altitude_m > s_peak_motor_m) s_peak_motor_m = s.altitude_m;

                    uint32_t elapsed_s = (xTaskGetTickCount() - s_launch_tick)
                                         / configTICK_RATE_HZ;

                    /* Check automatic cutoff conditions */
                    bool time_cutoff = (elapsed_s >= (uint32_t)MAX_MOTOR_TIME_S);
                    bool alt_cutoff  = (s_peak_m  >= MAX_ALTITUDE_M);

                    /* Check manual cutoff with debounce */
                    bool manual_cutoff = false;
                    if (pwm_is_fulldown()) {
                        if (s_fulldown_since_us == 0) {
                            s_fulldown_since_us = esp_timer_get_time();
                        } else if ((esp_timer_get_time() - s_fulldown_since_us)
                                    >= (FULLDOWN_DEBOUNCE_MS * 1000LL)) {
                            manual_cutoff = true;
                        }
                    } else {
                        s_fulldown_since_us = 0;
                    }

                    if (time_cutoff || alt_cutoff || manual_cutoff) {
                        cutoff_reason_t reason = manual_cutoff ? CUTOFF_MANUAL
                                               : alt_cutoff    ? CUTOFF_ALTITUDE
                                                               : CUTOFF_TIME;
                        s_state          = STATE_COASTING;
                        s_cutoff_reason  = reason;
                        s_cutoff_alt_m   = s.altitude_m;
                        s_alt_10s_recorded = false;

                        rec.cutoff_ms         = (uint32_t)(esp_timer_get_time() / 1000);
                        rec.peak_motor_m      = s_peak_motor_m;  /* frozen at cutoff */
                        rec.cutoff_altitude_m = s.altitude_m;
                        rec.cutoff_reason     = reason;
                        rec.peak_flight_m     = 0.0f;   /* still tracking in COASTING */
                        rec.alt_10s_m         = 0.0f;   /* filled 10s later */

                        ESP_LOGI(TAG, "*** CUTOFF [%s] | MOTOR PEAK: %.2fm | ALT: %.2fm ***",
                                 cutoff_reason_str(reason), s_peak_motor_m, s.altitude_m);
                    }
                    break;
                }

                /* ── COASTING ────────────────────────────────────────────── */
                case STATE_COASTING: {
                    esc_send_pulse(PWM_MIN_US);

                    if (!s_alt_10s_recorded) {
                        /* Keep tracking whole-flight peak (thermals, momentum) */
                        if (s.altitude_m > s_peak_m) s_peak_m = s.altitude_m;

                        uint32_t since_cutoff_ms =
                            (uint32_t)(esp_timer_get_time() / 1000) - rec.cutoff_ms;
                        if (since_cutoff_ms >= ALT_10S_DELAY_MS) {
                            rec.alt_10s_m      = s.altitude_m;
                            rec.peak_flight_m  = s_peak_m;   /* true flight peak */
                            s_alt_10s_recorded = true;
                            nvs_save_flight(&rec);
                            ESP_LOGI(TAG, "T+10s: %.2fm | Motor peak: %.2fm | "
                                     "Flight peak: %.2fm — record saved",
                                     s.altitude_m, rec.peak_motor_m, s_peak_m);
                        }
                    }
                    break;
                }
            }

            /* Push to report queue */
            if (xQueueSend(s_alt_queue, &s, 0) != pdTRUE) {
                sample_t discard;
                xQueueReceive(s_alt_queue, &discard, 0);
                xQueueSend(s_alt_queue, &s, 0);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
   Report task — 1 Hz console output
   ══════════════════════════════════════════════════════════════════════════ */

static void altitude_report_task(void *pvParam)
{
    while (1) {
        sample_t latest = {};
        sample_t s;
        TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(REPORT_INTERVAL_MS);
        while (xTaskGetTickCount() < deadline) {
            TickType_t remaining = deadline - xTaskGetTickCount();
            if (remaining == 0) break;
            if (xQueueReceive(s_alt_queue, &s, remaining) == pdTRUE) latest = s;
        }

        flight_state_t state = s_state;
        float peak           = s_peak_m;

#if SERIAL_REPORT_ENABLED
        switch (state) {
            case STATE_GROUND: {
                int thr = pwm_throttle_pct();
                if (thr < 0)
                    printf("GROUND  | Alt: %+7.2f m   Thr: --no signal--\n",
                           latest.altitude_m);
                else
                    printf("GROUND  | Alt: %+7.2f m   Thr: %3d%%\n",
                           latest.altitude_m, thr);
                break;
            }
            case STATE_CLIMBING: {
                uint32_t elapsed_s = (xTaskGetTickCount() - s_launch_tick)
                                      / configTICK_RATE_HZ;
                int thr = pwm_throttle_pct();
                printf("CLIMB   | Peak: %+7.2f m   T+%us   Thr: %3d%%\n",
                       s_peak_motor_m, (unsigned)elapsed_s, thr < 0 ? 0 : thr);
                break;
            }
            case STATE_COASTING:
                printf("COAST   | Peak: %+7.2f m   Alt: %+7.2f m   [%s]\n",
                       peak, latest.altitude_m,
                       s_alt_10s_recorded ? "SAVED" : "T+10s pending...");
                break;
        }
#endif
    }
}

/* ══════════════════════════════════════════════════════════════════════════
   app_main
   ══════════════════════════════════════════════════════════════════════════ */

extern "C" void app_main(void)
{
    /* NVS first — show last flight before anything else */
    nvs_init_storage();
    nvs_print_last_flight();

    ESP_LOGI(TAG, "F5L Altimeter — max alt: %.0fm, max motor: %ds, launch thr: %d%%",
             MAX_ALTITUDE_M, MAX_MOTOR_TIME_S, LAUNCH_THROTTLE_PCT);

    spi_init();
    pwm_input_init();
    esc_output_init();
    ESP_ERROR_CHECK(bmp280_init());
    calibrate_ground_zero();

    s_alt_queue = xQueueCreate(ALTITUDE_QUEUE_DEPTH, sizeof(sample_t));
    configASSERT(s_alt_queue != NULL);

    TimerHandle_t drift_timer = xTimerCreate("drift",
                                pdMS_TO_TICKS(DRIFT_TIMER_PERIOD_MS),
                                pdTRUE, NULL, drift_compensation_cb);
    configASSERT(drift_timer != NULL);
    xTimerStart(drift_timer, 0);

    xTaskCreate(bmp280_sample_task,   "bmp280_sample", SAMPLE_TASK_STACK,
                NULL, SAMPLE_TASK_PRIO, NULL);
    xTaskCreate(altitude_report_task, "alt_report",    REPORT_TASK_STACK,
                NULL, REPORT_TASK_PRIO, NULL);

    ESP_LOGI(TAG, "Ready — waiting for throttle > %d%% to detect launch",
             LAUNCH_THROTTLE_PCT);
}
