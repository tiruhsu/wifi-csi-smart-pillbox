/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_console.h"

#include "esp_mac.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "ping/ping_sock.h"
#include "hal/uart_ll.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"

#include "led_strip.h"
#include "esp_radar.h"
#include "csi_commands.h"

extern esp_ping_handle_t g_ping_handle;
static led_strip_handle_t led_strip;

#define WS2812_GPIO                 4
#define CONFIG_SEND_DATA_FREQUENCY  100
#define RADAR_BUFF_MAX_LEN          25

/* ================= 硬體腳位與門檻定義 ================= */
#define LED_ALERT_GPIO              GPIO_NUM_26    // 外接提示 LED 腳位
#define PHOTORESISTOR_ADC_CHANNEL   ADC_CHANNEL_6  // GPIO 34 (ADC1_CH6)
#define BUZZER_GPIO                 GPIO_NUM_25    // 蜂鳴器控制腳位
#define LID_OPEN_DELTA_THRESHOLD    45           // 開蓋亮度變化門檻 (比初始基準更亮超過 600 觸發)

/* ================= Wi-Fi 固定設定 ================= */
#define ROUTER_SSID                 "Tommy_AP"
#define ROUTER_PASSWORD             "0932695275"

/* ================= 全域狀態變數 ================= */
static volatile bool g_is_someone_in_room = false;
static volatile bool g_is_human_moving    = false;
static volatile bool g_waiting_leave_room = false;
static bool g_wifi_connect_status         = false;
static uint32_t g_send_data_interval      = 1000 / CONFIG_SEND_DATA_FREQUENCY;
static const char *TAG                    = "pillbox_app";

static adc_oneshot_unit_handle_t g_adc1_handle = NULL;
static esp_timer_handle_t g_leave_delay_timer  = NULL;
static esp_timer_handle_t g_train_stop_timer   = NULL;

static struct {
    struct arg_lit *train_start;
    struct arg_lit *train_stop;
    struct arg_lit *train_add;
    struct arg_str *predict_someone_threshold;
    struct arg_str *predict_someone_sensitivity;
    struct arg_str *predict_move_threshold;
    struct arg_str *predict_move_sensitivity;
    struct arg_int *predict_buff_size;
    struct arg_int *predict_outliers_number;
    struct arg_str *collect_taget;
    struct arg_int *collect_number;
    struct arg_int *collect_duration;
    struct arg_lit *csi_start;
    struct arg_lit *csi_stop;
    struct arg_str *csi_output_type;
    struct arg_str *csi_output_format;
    struct arg_int *csi_scale_shift;
    struct arg_int *channel_filter;
    struct arg_int *send_data_interval;
    struct arg_end *end;
} radar_args;

static struct console_input_config {
    bool train_start;
    float predict_someone_threshold;
    float predict_someone_sensitivity;
    float predict_move_threshold;
    float predict_move_sensitivity;
    uint32_t predict_buff_size;
    uint32_t predict_outliers_number;
} g_console_input_config = {
    .predict_someone_threshold   = 0.00105,
    .predict_someone_sensitivity = 0.15,
    .predict_move_threshold      = 0.0003,
    .predict_move_sensitivity    = 0.20,
    .predict_buff_size           = 5,
    .predict_outliers_number     = 2,
    .train_start                 = false,
};
/* ================= 1. LED初始化================= */
static void alert_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_ALERT_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(LED_ALERT_GPIO, 0); // 預設熄滅
}
/* ================= 1. 光敏電阻初始化與讀取函式 ================= */
static void photoresistor_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_adc1_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1_handle, PHOTORESISTOR_ADC_CHANNEL, &chan_config));
    ESP_LOGI(TAG, "光敏電阻 ADC 初始化完成 (GPIO 34)");
}

static int photoresistor_read_raw(void)
{
    int raw_val = 0;
    if (g_adc1_handle != NULL) {
        adc_oneshot_read(g_adc1_handle, PHOTORESISTOR_ADC_CHANNEL, &raw_val);
    }
    return raw_val;
}

/* ================= 2. 蜂鳴器 PWM 初始化與開關函式 ================= */
static void buzzer_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT,
        .freq_hz          = 2500, // 2.5kHz 響亮提示音頻率（有源/無源蜂鳴器皆可響）
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = BUZZER_GPIO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(TAG, "蜂鳴器 PWM 初始化完成 (GPIO %d)", BUZZER_GPIO);
}

static void buzzer_set(bool on)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, on ? 512 : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

/* ================= 兩階段自動訓練計時器回呼 ================= */
static void auto_train_stop_cb(void *arg)
{
    esp_radar_train_stop(&g_console_input_config.predict_someone_threshold,
                         &g_console_input_config.predict_move_threshold);
    g_waiting_leave_room = false;
    g_console_input_config.train_start = false;

    led_strip_set_pixel(led_strip, 0, 0, 0, 0);
    led_strip_refresh(led_strip);

    ESP_LOGI(TAG, ">>> 30 秒無人學習完成！someone_th: %.6f, move_th: %.6f <<<",
             g_console_input_config.predict_someone_threshold,
             g_console_input_config.predict_move_threshold);
}

static void auto_train_begin_cb(void *arg)
{
    g_waiting_leave_room = false;
    esp_radar_train_remove();
    esp_radar_train_start();
    g_console_input_config.train_start = true;

    led_strip_set_pixel(led_strip, 0, 128, 0, 128);
    led_strip_refresh(led_strip);

    ESP_LOGI(TAG, ">>> 10 秒緩衝結束，開始 30 秒無人背景學習（請勿進入房間）... <<<");

    if (g_train_stop_timer == NULL) {
        const esp_timer_create_args_t stop_args = {
            .callback = &auto_train_stop_cb,
            .name = "train_stop_30s"
        };
        ESP_ERROR_CHECK(esp_timer_create(&stop_args, &g_train_stop_timer));
    }
    esp_timer_stop(g_train_stop_timer);
    ESP_ERROR_CHECK(esp_timer_start_once(g_train_stop_timer, 30 * 1000 * 1000));
}

/* ================= 官方原版雷達指令與回呼 ================= */
static int wifi_cmd_radar(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **) &radar_args) != ESP_OK) {
        arg_print_errors(stderr, radar_args.end, argv[0]);
        return ESP_FAIL;
    }

    if (radar_args.train_start->count) {
        if (g_leave_delay_timer != NULL) esp_timer_stop(g_leave_delay_timer);
        if (g_train_stop_timer != NULL)  esp_timer_stop(g_train_stop_timer);

        g_console_input_config.train_start = false;
        g_waiting_leave_room = true;
        ESP_LOGI(TAG, ">>> 請在 10 秒內離開房間！倒數結束後將自動開始 30 秒無人學習... <<<");

        if (g_leave_delay_timer == NULL) {
            const esp_timer_create_args_t delay_args = {
                .callback = &auto_train_begin_cb,
                .name = "leave_delay_10s"
            };
            ESP_ERROR_CHECK(esp_timer_create(&delay_args, &g_leave_delay_timer));
        }
        ESP_ERROR_CHECK(esp_timer_start_once(g_leave_delay_timer, 10 * 1000 * 1000));
    }

    if (radar_args.train_stop->count) {
        if (g_leave_delay_timer != NULL) esp_timer_stop(g_leave_delay_timer);
        if (g_train_stop_timer != NULL)  esp_timer_stop(g_train_stop_timer);
        auto_train_stop_cb(NULL);
    }

    if (radar_args.predict_someone_threshold->count) {
        g_console_input_config.predict_someone_threshold = atof(radar_args.predict_someone_threshold->sval[0]);
    }
    if (radar_args.predict_move_threshold->count) {
        g_console_input_config.predict_move_threshold = atof(radar_args.predict_move_threshold->sval[0]);
    }
    if (radar_args.csi_start->count) esp_radar_start();
    if (radar_args.csi_stop->count) esp_radar_stop();

    return ESP_OK;
}

void cmd_register_radar(void)
{
    radar_args.train_start = arg_lit0(NULL, "train_start", "Start calibrating (10s leave + 30s learn)");
    radar_args.train_stop  = arg_lit0(NULL, "train_stop", "Stop calibrating immediately");
    radar_args.train_add   = arg_lit0(NULL, "train_add", "Add calibration");
    radar_args.predict_someone_threshold   = arg_str0(NULL, "predict_someone_threshold", "<0 ~ 1.0>", "Threshold for someone");
    radar_args.predict_someone_sensitivity = arg_str0(NULL, "predict_someone_sensitivity", "<0 ~ 1.0>", "Sensitivity for someone");
    radar_args.predict_move_threshold      = arg_str0(NULL, "predict_move_threshold", "<0 ~ 1.0>", "Threshold for move");
    radar_args.predict_move_sensitivity    = arg_str0(NULL, "predict_move_sensitivity", "<0 ~ 1.0>", "Sensitivity for move");
    radar_args.predict_buff_size           = arg_int0(NULL, "predict_buff_size", "1 ~ 100", "Buffer size");
    radar_args.predict_outliers_number     = arg_int0(NULL, "predict_outliers_number", "<1 ~ 100>", "Outliers number");
    radar_args.collect_taget      = arg_str0(NULL, "collect_tagets", "<0 ~ 20>", "Target");
    radar_args.collect_number     = arg_int0(NULL, "collect_number", "sequence", "Sequence");
    radar_args.collect_duration   = arg_int0(NULL, "collect_duration", "duration", "Duration");
    radar_args.csi_start          = arg_lit0(NULL, "csi_start", "Start CSI");
    radar_args.csi_stop           = arg_lit0(NULL, "csi_stop", "Stop CSI");
    radar_args.csi_output_type    = arg_str0(NULL, "csi_output_type", "<NULL, LLTF>", "CSI type");
    radar_args.csi_output_format  = arg_str0(NULL, "csi_output_format", "<decimal, base64>", "CSI format");
    radar_args.csi_scale_shift    = arg_int0(NULL, "scale_shift", "<0~15>", "Scale shift");
    radar_args.channel_filter     = arg_int0(NULL, "channel_filter", "<0 or 1>", "Channel filter");
    radar_args.send_data_interval = arg_int0(NULL, "send_data_interval", "<interval_ms>", "Interval");
    radar_args.end                = arg_end(8);

    const esp_console_cmd_t radar_cmd = {
        .command = "radar",
        .help = "Radar config",
        .hint = NULL,
        .func = &wifi_cmd_radar,
        .argtable = &radar_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&radar_cmd));
}

static void wifi_radar_cb(void *ctx, const wifi_radar_info_t *info)
{
    static float *s_buff_wander  = NULL;
    static float *s_buff_jitter  = NULL;
    static uint32_t s_buff_count = 0;
    uint32_t buff_max_size       = g_console_input_config.predict_buff_size;
    uint32_t buff_outliers_num   = g_console_input_config.predict_outliers_number;
    uint32_t someone_count       = 0;
    uint32_t move_count          = 0;

    if (!s_buff_wander) s_buff_wander = calloc(RADAR_BUFF_MAX_LEN, sizeof(float));
    if (!s_buff_jitter) s_buff_jitter = calloc(RADAR_BUFF_MAX_LEN, sizeof(float));

    s_buff_wander[s_buff_count % RADAR_BUFF_MAX_LEN] = info->waveform_wander;
    s_buff_jitter[s_buff_count % RADAR_BUFF_MAX_LEN] = info->waveform_jitter;
    s_buff_count++;

    if (s_buff_count < buff_max_size) return;

    extern float trimmean(const float * array, size_t len, float percent);
    extern float median(const float * a, size_t len);

    float wander_average = trimmean(s_buff_wander, RADAR_BUFF_MAX_LEN, 0.5);
    float jitter_midean  = median(s_buff_jitter, RADAR_BUFF_MAX_LEN);

    for (int i = 0; i < buff_max_size; i++) {
        uint32_t index = (s_buff_count - 1 - i) % RADAR_BUFF_MAX_LEN;
        if (wander_average * g_console_input_config.predict_someone_sensitivity > g_console_input_config.predict_someone_threshold) {
            someone_count++;
        }
        if (s_buff_jitter[index] * g_console_input_config.predict_move_sensitivity > g_console_input_config.predict_move_threshold
                || (s_buff_jitter[index] * g_console_input_config.predict_move_sensitivity > jitter_midean && s_buff_jitter[index] > 0.0002)) {
            move_count++;
        }
    }

    if (g_waiting_leave_room) {
        static bool led_status = false;
        led_strip_set_pixel(led_strip, 0, led_status ? 0 : 255, led_status ? 0 : 255, 0);
        led_status = !led_status;
        led_strip_refresh(led_strip);
        return;
    }

    if (g_console_input_config.train_start) {
        return;
    }

    g_is_someone_in_room = (someone_count >= 1);
    g_is_human_moving    = (move_count >= buff_outliers_num);
}

/* ================= Wi-Fi 固定連線與 Ping 任務 ================= */
static void trigger_router_send_data_task(void *arg)
{
    esp_radar_config_t radar_config = {0};
    wifi_ap_record_t ap_info         = {0};
    uint8_t sta_mac[6]               = {0};

    esp_radar_get_config(&radar_config);
    esp_wifi_sta_get_ap_info(&ap_info);
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, sta_mac));

    radar_config.csi_config.csi_recv_interval = g_send_data_interval;
    memcpy(radar_config.csi_config.filter_dmac, sta_mac, sizeof(radar_config.csi_config.filter_dmac));
    memcpy(radar_config.csi_config.filter_mac, ap_info.bssid, sizeof(radar_config.csi_config.filter_mac));
    esp_radar_change_config(&radar_config);

    if (g_ping_handle != NULL) {
        esp_ping_stop(g_ping_handle);
        esp_ping_delete_session(g_ping_handle);
        g_ping_handle = NULL;
    }

    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
    config.count       = 0;
    config.data_size   = 1;
    config.interval_ms = g_send_data_interval;

    esp_netif_ip_info_t local_ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &local_ip);
    ESP_LOGI(TAG, "Wi-Fi 已連線並啟動 Ping -> IP: " IPSTR, IP2STR(&local_ip.ip));
    config.target_addr.u_addr.ip4.addr = ip4_addr_get_u32(&local_ip.gw);
    config.target_addr.type = ESP_IPADDR_TYPE_V4;

    esp_ping_callbacks_t cbs = { 0 };
    esp_ping_new_session(&config, &cbs, &g_ping_handle);
    esp_ping_start(g_ping_handle);

    vTaskDelete(NULL);
}

static void wifi_connect_router(void)
{
    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, ROUTER_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, ROUTER_PASSWORD, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.sta.pmf_cfg.capable    = true;
    cfg.sta.pmf_cfg.required   = false;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        g_wifi_connect_status = true;
        xTaskCreate(trigger_router_send_data_task, "trigger_ping", 4 * 1024, NULL, 5, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connect_status = false;
        ESP_LOGW(TAG, "Wi-Fi 斷線，自動重連中...");
        esp_wifi_connect();
    }
}

esp_err_t ws2812_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds       = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src       = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags         = { .with_dma = false }
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
    return ESP_OK;
}

/* ================= 智慧藥盒主任務 ================= */
static void pillbox_task(void *arg)
{
    ESP_LOGI(TAG, "pillbox_task 啟動！");

    // 1. 開機自動取樣 10 次（0.5 秒）鎖定「蓋著狀態」的初始亮度基準
    int baseline_sum = 0;
    for (int i = 0; i < 10; i++) {
        baseline_sum += photoresistor_read_raw();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    int closed_light_baseline = baseline_sum / 10;
    ESP_LOGI(TAG, "光敏初始基準已鎖定 (蓋著 ADC: %d，開蓋觸發門檻: > %d)",
             closed_light_baseline, closed_light_baseline + LID_OPEN_DELTA_THRESHOLD);

    bool last_someone  = false;
    bool last_lid_open = false;
    bool beep_toggle   = false;
    int print_counter  = 0; // 用於控制每秒印出一次數值

    while (1) {
        if (!g_waiting_leave_room && !g_console_input_config.train_start) {
            // 2. 讀取雷達狀態
            bool someone_now = g_is_someone_in_room;
            if (someone_now != last_someone) {
                last_someone = someone_now;
                ESP_LOGI(TAG, "雷達狀態切換 -> %s", someone_now ? "有人 (PRESENT)" : "無人 (NONE)");
                led_strip_set_pixel(led_strip, 0, 0, someone_now ? 255 : 0, 0);
                led_strip_refresh(led_strip);
            }

            // 3. 讀取光敏電阻，與初始基準比較
            int light_raw = photoresistor_read_raw();
            bool is_lid_open = (light_raw < closed_light_baseline - LID_OPEN_DELTA_THRESHOLD);

            // 4. 連動控制：有人在房間 且 藥盒未打開 -> 蜂鳴器間歇鳴叫；開蓋或無人 -> 靜音
            if (someone_now && !is_lid_open) {
                beep_toggle = !beep_toggle;
                buzzer_set(beep_toggle);
                gpio_set_level(LED_ALERT_GPIO, 1);
            } else {
                beep_toggle = false;
                buzzer_set(false);
                gpio_set_level(LED_ALERT_GPIO, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ws2812_led_init();
    photoresistor_init(); // GPIO 34
    buzzer_init();        // GPIO 25
    alert_led_init();
    
    esp_log_level_set("esp_radar", ESP_LOG_WARN);

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    repl_config.prompt = "csi>";
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));

    uart_ll_set_sclk(UART_LL_GET_HW(CONFIG_ESP_CONSOLE_UART_NUM), UART_SCLK_APB);
    uart_ll_set_baudrate(UART_LL_GET_HW(CONFIG_ESP_CONSOLE_UART_NUM), CONFIG_ESP_CONSOLE_UART_BAUDRATE, APB_CLK_FREQ);

    esp_radar_csi_config_t csi_config   = ESP_RADAR_CSI_CONFIG_DEFAULT();
    esp_radar_wifi_config_t wifi_config = ESP_RADAR_WIFI_CONFIG_DEFAULT();
    esp_radar_dec_config_t dec_config   = ESP_RADAR_DEC_CONFIG_DEFAULT();
    memcpy(csi_config.filter_mac, "\xff\xff\xff\xff\xff\xff", 6);
    csi_config.csi_recv_interval  = g_send_data_interval;
    dec_config.wifi_radar_cb      = wifi_radar_cb;
    dec_config.outliers_threshold = 0;

    ESP_ERROR_CHECK(esp_radar_wifi_init(&wifi_config));
    ESP_ERROR_CHECK(esp_radar_csi_init(&csi_config));
    ESP_ERROR_CHECK(esp_radar_dec_init(&dec_config));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL));

    cmd_register_ping();
    cmd_register_system();
    cmd_register_wifi_config();
    cmd_register_wifi_scan();
    cmd_register_radar();
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    esp_radar_start();
    wifi_connect_router();

    xTaskCreate(pillbox_task, "pillbox_task", 4 * 1024, NULL, 3, NULL);
}