#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_bt.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLEIO-ESP32";

// サービスとキャラクタリスティックの UUID
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x4b, 0x91, 0x33, 0xc3, 0xc9, 0xc5, 0xcc, 0x8f,
                     0x9e, 0x45, 0xb5, 0x1f, 0x01, 0xc2, 0xaf, 0x4f);

static const ble_uuid128_t gatt_svr_chr_write_uuid =
    BLE_UUID128_INIT(0xa8, 0x26, 0x1b, 0x36, 0x07, 0xea, 0xf5, 0xb7,
                     0x88, 0x46, 0xe1, 0x36, 0x3e, 0x48, 0xb5, 0xbe);

static const ble_uuid128_t gatt_svr_chr_read_uuid =
    BLE_UUID128_INIT(0x7e, 0xe8, 0x7b, 0x5d, 0x2e, 0x7a, 0x3d, 0xbf,
                     0x3a, 0x41, 0xf7, 0xd8, 0xe3, 0xd5, 0x95, 0x1c);

static const ble_uuid128_t gatt_svr_chr_adc_read_uuid =
    BLE_UUID128_INIT(0x1d, 0x0c, 0x9b, 0x8a, 0x7f, 0x6e, 0x5d, 0x8c,
                     0x1b, 0x4a, 0x9f, 0x4e, 0x3c, 0x7b, 0x8a, 0x2d);

// GPIO コマンド定義
#define CMD_SET_OUTPUT_LOW 1
#define CMD_SET_OUTPUT_HIGH 2
#define CMD_SET_OUTPUT_BLINK_250MS 3
#define CMD_SET_OUTPUT_BLINK_500MS 4
#define CMD_SET_OUTPUT_PWM 5
#define CMD_SET_OUTPUT_ON_DISCONNECT 9
#define CMD_SET_INPUT_FLOATING 11
#define CMD_SET_INPUT_PULLUP 12
#define CMD_SET_INPUT_PULLDOWN 13
#define CMD_SET_ADC_ENABLE 21
#define CMD_SET_ADC_DISABLE 22

// 内部用途予約 GPIO
#define GPIO_AUTH_ENABLE 5  // 認証機能有効/無効 (LOW: 有効, HIGH: 無効)
#define GPIO_PAIRING_MODE 4 // ペアリングモード (LOW: ペアリングモード, HIGH: 通常モード)

// GPIO 最大数
#define MAX_USABLE_GPIO 22 // 使用可能な GPIO の総数 (GPIO4, GPIO5 を除く)

// BLE ATT MTU 計算
// WRITE データ構造: 1 (コマンド個数) + MAX_USABLE_GPIO * 4 (各コマンド 4 バイト)
// ATT ヘッダ: 3 バイト (Opcode 1 + Attribute Handle 2)
// 必要な MTU = ATT ヘッダ (3) + ペイロード (1 + 22 * 4) = 3 + 89 = 92 バイト
#define ATT_HEADER_SIZE 3
#define COMMAND_HEADER_SIZE 1 // コマンド個数フィールド
#define COMMAND_SIZE 4        // 各コマンドのサイズ (Pin + Command + Param1 + Param2)
#define PAYLOAD_SIZE (COMMAND_HEADER_SIZE + (MAX_USABLE_GPIO * COMMAND_SIZE))
#define REQUIRED_MTU (ATT_HEADER_SIZE + PAYLOAD_SIZE)

// 入力ラッチモード定義
#define LATCH_MODE_NONE 0 // ラッチなし
#define LATCH_MODE_LOW 1  // LOWラッチ
#define LATCH_MODE_HIGH 2 // HIGHラッチ

// ポーリング設定
#define INPUT_POLL_INTERVAL_MS 10 // 入力ポーリング間隔 (ms)
#define LATCH_STABLE_COUNT 2      // ラッチ判定に必要な連続安定回数

// PWM 設定
#define LEDC_CHANNEL_MAX 8                     // Low Speed モードで使用可能なチャネル数
#define LEDC_TIMER_RESOLUTION LEDC_TIMER_8_BIT // 8 ビット分解能 (0-255)
#define LEDC_TIMER_NUM LEDC_TIMER_0            // 使用するタイマー番号
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

// GPIO モード状態の定義
typedef enum
{
    BLEIO_MODE_UNSET = 0,      // モード未設定 (初期状態)
    BLEIO_MODE_INPUT_FLOATING, // ハイインピーダンス入力モード
    BLEIO_MODE_INPUT_PULLUP,   // 内部プルアップ付き入力モード
    BLEIO_MODE_INPUT_PULLDOWN, // 内部プルダウン付き入力モード
    BLEIO_MODE_OUTPUT_LOW,     // LOW (0V) 出力モード
    BLEIO_MODE_OUTPUT_HIGH,    // HIGH (3.3V) 出力モード
    BLEIO_MODE_BLINK_250MS,    // 250ms 点滅出力モード
    BLEIO_MODE_BLINK_500MS,    // 500ms 点滅出力モード
    BLEIO_MODE_PWM,            // PWM 出力モード
    BLEIO_MODE_ADC             // ADC 入力モード
} bleio_mode_state_t;

// PWM 設定保持用構造体
typedef struct
{
    uint8_t duty_cycle;  // デューティサイクル (0-255)
    uint8_t freq_preset; // 周波数プリセット (0-7)
    int8_t channel;      // 割り当てられた LEDC チャネル (-1: 未割り当て)
} pwm_config_t;

// LEDC チャネル管理用構造体
typedef struct
{
    bool in_use;      // チャネルが使用中か
    uint8_t gpio_num; // このチャネルを使用している GPIO 番号
} ledc_channel_info_t;

// ADC 設定保持用構造体
typedef struct
{
    int8_t channel;                // ADC1 チャネル (-1: 未設定)
    adc_atten_t attenuation;       // 減衰率
    bool calibrated;               // キャリブレーション済みか
    adc_cali_handle_t cali_handle; // キャリブレーションハンドル
} adc_config_t;

// GPIO ごとの状態管理
typedef struct
{
    bleio_mode_state_t mode;       // 現在のモード
    uint8_t current_level;         // 現在の出力レベル (点滅時に使用)
    uint8_t latch_mode;            // 入力ラッチモード (0-2)
    bool is_latched;               // ラッチ済みフラグ
    uint8_t stable_counter;        // 安定カウンタ
    uint8_t last_level;            // 前回の読み取り値
    uint8_t disconnect_behavior;   // BLE 切断時の振る舞い (0: 維持, 1: LOW, 2: HIGH)
} bleio_gpio_state_t;

// 周波数プリセットテーブル (Hz)
static const uint32_t pwm_freq_table[] = {
    1000,  // 0: 1 kHz (デフォルト)
    5000,  // 1: 5 kHz
    10000, // 2: 10 kHz
    25000, // 3: 25 kHz
    50,    // 4: 50 Hz (サーボモーター)
    100,   // 5: 100 Hz
    500,   // 6: 500 Hz
    20000  // 7: 20 kHz
};
#define PWM_FREQ_TABLE_SIZE (sizeof(pwm_freq_table) / sizeof(pwm_freq_table[0]))

// GPIO から ADC1 チャネルへのマッピングテーブル
static const struct
{
    uint8_t gpio_num;
    adc_channel_t channel;
} adc1_gpio_map[] = {
    {32, ADC_CHANNEL_4},
    {33, ADC_CHANNEL_5},
    {34, ADC_CHANNEL_6},
    {35, ADC_CHANNEL_7},
    {36, ADC_CHANNEL_0},
    {39, ADC_CHANNEL_3}};
#define ADC1_GPIO_MAP_SIZE (sizeof(adc1_gpio_map) / sizeof(adc1_gpio_map[0]))

// 減衰率マッピングテーブル
static const adc_atten_t adc_atten_map[] = {
    ADC_ATTEN_DB_0,   // 0: 0 dB (0-1.1V)
    ADC_ATTEN_DB_2_5, // 1: 2.5 dB (0-1.5V)
    ADC_ATTEN_DB_6,   // 2: 6 dB (0-2.2V)
    ADC_ATTEN_DB_12   // 3: 12 dB (0-3.3V、旧 11dB)
};
#define ADC_ATTEN_MAP_SIZE (sizeof(adc_atten_map) / sizeof(adc_atten_map[0]))

// グローバル変数
static uint16_t conn_handle = 0;
static bleio_gpio_state_t gpio_states[40] = {0};                  // 全 GPIO の状態
static pwm_config_t pwm_configs[40] = {0};                        // 全 GPIO の PWM 設定
static ledc_channel_info_t ledc_channels[LEDC_CHANNEL_MAX] = {0}; // LEDC チャネル管理
static adc_config_t adc_configs[40] = {0};                        // 全 GPIO の ADC 設定
static adc_oneshot_unit_handle_t adc1_handle = NULL;              // ADC1 ユニットハンドル
static uint8_t global_blink_counter = 0;                          // 全 GPIO 共通の点滅カウンタ
static esp_timer_handle_t blink_timer = NULL;
static esp_timer_handle_t input_poll_timer = NULL;
static portMUX_TYPE gpio_states_mux = portMUX_INITIALIZER_UNLOCKED; // gpio_states 保護用スピンロック
static bool auth_mode_enabled = false;                            // 認証モードフラグ

// 関数の前方宣言
static void ble_app_advertise(void);
static bool is_valid_gpio(uint8_t pin);

// 点滅タイマコールバック (250ms 周期)
static void blink_timer_callback(void *arg)
{
    // グローバルカウンタを更新 (0-3 の範囲)
    global_blink_counter++;
    if (global_blink_counter >= 4)
    {
        global_blink_counter = 0;
    }

    // 250ms 点滅: カウンタが偶数 (0, 2) で点灯、奇数 (1, 3) で消灯
    bool blink_250ms_level = (global_blink_counter % 2 == 0);

    // 500ms 点滅: カウンタが 0-1 で点灯、2-3 で消灯
    bool blink_500ms_level = (global_blink_counter < 2);

    for (int pin = 0; pin < 40; pin++)
    {
        if (!is_valid_gpio(pin))
        {
            continue;
        }

        portENTER_CRITICAL(&gpio_states_mux);
        bleio_gpio_state_t *state = &gpio_states[pin];
        bleio_mode_state_t mode = state->mode;
        portEXIT_CRITICAL(&gpio_states_mux);

        uint8_t new_level = 0;
        bool should_update = false;

        if (mode == BLEIO_MODE_BLINK_250MS)
        {
            // 250ms 周期: グローバルカウンタが偶数で点灯
            new_level = blink_250ms_level ? 1 : 0;
            should_update = true;
        }
        else if (mode == BLEIO_MODE_BLINK_500MS)
        {
            // 500ms 周期: グローバルカウンタが 0-1 で点灯
            new_level = blink_500ms_level ? 1 : 0;
            should_update = true;
        }

        if (should_update)
        {
            portENTER_CRITICAL(&gpio_states_mux);
            state->current_level = new_level;
            portEXIT_CRITICAL(&gpio_states_mux);

            gpio_set_level(pin, new_level);
        }
    }
}

// 入力ポーリングタイマコールバック (10ms 周期)
static void input_poll_timer_callback(void *arg)
{
    for (int pin = 0; pin < 40; pin++)
    {
        if (!is_valid_gpio(pin))
        {
            continue;
        }

        portENTER_CRITICAL(&gpio_states_mux);
        bleio_gpio_state_t *state = &gpio_states[pin];
        uint8_t latch_mode = state->latch_mode;
        bool is_latched = state->is_latched;
        bleio_mode_state_t mode = state->mode;
        uint8_t last_level = state->last_level;
        uint8_t stable_counter = state->stable_counter;
        portEXIT_CRITICAL(&gpio_states_mux);

        // ラッチモードが設定されていない、または既にラッチ済みの場合はスキップ
        if (latch_mode == LATCH_MODE_NONE || is_latched)
        {
            continue;
        }

        // 入力モードかチェック
        if (mode != BLEIO_MODE_INPUT_FLOATING &&
            mode != BLEIO_MODE_INPUT_PULLUP &&
            mode != BLEIO_MODE_INPUT_PULLDOWN)
        {
            continue;
        }

        uint8_t level = gpio_get_level(pin);
        uint8_t target = (latch_mode == LATCH_MODE_HIGH) ? 1 : 0;

        bool new_is_latched = is_latched;
        uint8_t new_stable_counter = stable_counter;

        if (level == target)
        {
            if (last_level == target)
            {
                new_stable_counter++;
                if (new_stable_counter >= LATCH_STABLE_COUNT)
                {
                    new_is_latched = true;
                    ESP_LOGI(TAG, "GPIO%d latched to %s", pin, target ? "HIGH" : "LOW");
                }
            }
            else
            {
                new_stable_counter = 1;
            }
        }
        else
        {
            new_stable_counter = 0;
        }

        portENTER_CRITICAL(&gpio_states_mux);
        state->is_latched = new_is_latched;
        state->stable_counter = new_stable_counter;
        state->last_level = level;
        portEXIT_CRITICAL(&gpio_states_mux);
    }
}

// 認証機能とペアリングモードの判定
static bool is_auth_enabled(void)
{
    // GPIO5 をプルアップ付き入力として設定
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_AUTH_ENABLE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(10)); // 安定待ち

    int level = gpio_get_level(GPIO_AUTH_ENABLE);
    return (level == 0); // LOW の場合 true (認証有効)
}

static bool is_pairing_mode_requested(void)
{
    // GPIO4 をプルアップ付き入力として設定
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_PAIRING_MODE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    vTaskDelay(pdMS_TO_TICKS(10)); // 安定待ち

    int level = gpio_get_level(GPIO_PAIRING_MODE);
    return (level == 0); // LOW の場合 true (ペアリングモード)
}

static void clear_bonding_info(void)
{
    ESP_LOGI(TAG, "ボンディング情報をクリアしています...");

    // NimBLE のボンディング情報をクリア
    ble_store_clear();

    ESP_LOGI(TAG, "ボンディング情報をクリアしました");
}

// GPIO 制御関数
static bool is_valid_gpio(uint8_t pin)
{
    // GPIO4, GPIO5 は内部用途に予約されているため除外
    if (pin == GPIO_AUTH_ENABLE || pin == GPIO_PAIRING_MODE)
    {
        return false;
    }

    // 使用可能な GPIO ピン
    if (pin == 2 || (pin >= 12 && pin <= 19) ||
        (pin >= 21 && pin <= 27) || (pin >= 32 && pin <= 36) || pin == 39)
    {
        return true;
    }
    return false;
}

static bool is_valid_output_pin(uint8_t pin)
{
    // 入力専用ピン (GPIO34, 35, 36, 39) を除外
    if (pin >= 34 && pin <= 39)
    {
        return false;
    }
    return is_valid_gpio(pin);
}

// PWM 関連関数
static void pwm_init(void)
{
    // LEDC タイマー設定
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_RESOLUTION,
        .timer_num = LEDC_TIMER_NUM,
        .freq_hz = pwm_freq_table[0], // デフォルト周波数 (1 kHz)
        .clk_cfg = LEDC_AUTO_CLK};
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return;
    }

    // チャネル管理テーブルの初期化
    for (int i = 0; i < LEDC_CHANNEL_MAX; i++)
    {
        ledc_channels[i].in_use = false;
        ledc_channels[i].gpio_num = 0;
    }

    // PWM 設定の初期化
    for (int i = 0; i < 40; i++)
    {
        pwm_configs[i].duty_cycle = 0;
        pwm_configs[i].freq_preset = 0;
        pwm_configs[i].channel = -1;
    }

    ESP_LOGI(TAG, "PWM initialized (timer=%d, resolution=%d bit, default freq=%d Hz)",
             LEDC_TIMER_NUM, (1 << LEDC_TIMER_RESOLUTION) - 1, pwm_freq_table[0]);
}

static int8_t allocate_ledc_channel(uint8_t gpio_num)
{
    // 既にこの GPIO に割り当てられているチャネルがあれば再利用
    for (int i = 0; i < LEDC_CHANNEL_MAX; i++)
    {
        if (ledc_channels[i].in_use && ledc_channels[i].gpio_num == gpio_num)
        {
            return (int8_t)i;
        }
    }

    // 空きチャネルを検索
    for (int i = 0; i < LEDC_CHANNEL_MAX; i++)
    {
        if (!ledc_channels[i].in_use)
        {
            ledc_channels[i].in_use = true;
            ledc_channels[i].gpio_num = gpio_num;
            return (int8_t)i;
        }
    }

    // チャネルが不足
    return -1;
}

static void free_ledc_channel(uint8_t gpio_num)
{
    for (int i = 0; i < LEDC_CHANNEL_MAX; i++)
    {
        if (ledc_channels[i].in_use && ledc_channels[i].gpio_num == gpio_num)
        {
            ledc_stop(LEDC_SPEED_MODE, (ledc_channel_t)i, 0);
            ledc_channels[i].in_use = false;
            ledc_channels[i].gpio_num = 0;
            pwm_configs[gpio_num].channel = -1;
            ESP_LOGI(TAG, "Freed LEDC channel %d for GPIO%d", i, gpio_num);
            break;
        }
    }
}

static void stop_pwm_if_active(uint8_t pin)
{
    portENTER_CRITICAL(&gpio_states_mux);
    bleio_mode_state_t mode = gpio_states[pin].mode;
    portEXIT_CRITICAL(&gpio_states_mux);

    if (mode == BLEIO_MODE_PWM)
    {
        free_ledc_channel(pin);
        ESP_LOGI(TAG, "Stopped PWM on GPIO%d", pin);
    }
}

static void stop_adc_if_active(uint8_t pin)
{
    portENTER_CRITICAL(&gpio_states_mux);
    bleio_mode_state_t mode = gpio_states[pin].mode;
    portEXIT_CRITICAL(&gpio_states_mux);

    if (mode == BLEIO_MODE_ADC)
    {
        // キャリブレーションハンドルを削除
        if (adc_configs[pin].cali_handle != NULL)
        {
            adc_cali_delete_scheme_line_fitting(adc_configs[pin].cali_handle);
            adc_configs[pin].cali_handle = NULL;
        }

        // 設定をクリア
        adc_configs[pin].channel = -1;
        adc_configs[pin].calibrated = false;

        portENTER_CRITICAL(&gpio_states_mux);
        gpio_states[pin].mode = BLEIO_MODE_UNSET;
        portEXIT_CRITICAL(&gpio_states_mux);

        ESP_LOGI(TAG, "Stopped ADC on GPIO%d", pin);
    }
}

static esp_err_t gpio_set_pwm(uint8_t pin, uint8_t duty_cycle, uint8_t freq_preset)
{
    // パラメータ検証
    if (!is_valid_output_pin(pin))
    {
        ESP_LOGE(TAG, "GPIO%d は PWM 出力に対応していません", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (freq_preset >= PWM_FREQ_TABLE_SIZE)
    {
        ESP_LOGE(TAG, "無効な周波数プリセット: %d (最大: %d)", freq_preset, PWM_FREQ_TABLE_SIZE - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // ADC が有効な場合は停止
    stop_adc_if_active(pin);

    // 既存のモードをクリーンアップ
    portENTER_CRITICAL(&gpio_states_mux);
    bleio_mode_state_t prev_mode = gpio_states[pin].mode;
    portEXIT_CRITICAL(&gpio_states_mux);

    if (prev_mode == BLEIO_MODE_BLINK_250MS || prev_mode == BLEIO_MODE_BLINK_500MS)
    {
        // 点滅モードは特にクリーンアップ不要 (タイマーコールバックで自動スキップされる)
    }

    if (prev_mode == BLEIO_MODE_PWM)
    {
        free_ledc_channel(pin);
    }

    // LEDC チャネルを割り当て
    int8_t channel = allocate_ledc_channel(pin);
    if (channel < 0)
    {
        ESP_LOGE(TAG, "LEDC チャネルが不足しています (最大 %d チャネル)", LEDC_CHANNEL_MAX);
        return ESP_ERR_NO_MEM;
    }

    // 周波数を設定 (タイマー 0 を使用)
    uint32_t freq_hz = pwm_freq_table[freq_preset];
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_RESOLUTION,
        .timer_num = LEDC_TIMER_NUM,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK};
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        ledc_channels[channel].in_use = false;
        ledc_channels[channel].gpio_num = 0;
        return ret;
    }

    // チャネルを設定
    ledc_channel_config_t channel_conf = {
        .gpio_num = pin,
        .speed_mode = LEDC_SPEED_MODE,
        .channel = (ledc_channel_t)channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_NUM,
        .duty = duty_cycle, // 8 ビット値 (0-255)
        .hpoint = 0};
    ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        ledc_channels[channel].in_use = false;
        ledc_channels[channel].gpio_num = 0;
        return ret;
    }

    // 設定を保存
    pwm_configs[pin].duty_cycle = duty_cycle;
    pwm_configs[pin].freq_preset = freq_preset;
    pwm_configs[pin].channel = channel;

    portENTER_CRITICAL(&gpio_states_mux);
    gpio_states[pin].mode = BLEIO_MODE_PWM;
    portEXIT_CRITICAL(&gpio_states_mux);

    ESP_LOGI(TAG, "GPIO%d を PWM 出力に設定しました (デューティ: %d/255 = %.1f%%, 周波数: %d Hz, チャネル: %d)",
             pin, duty_cycle, (duty_cycle * 100.0) / 255.0, freq_hz, channel);

    return ESP_OK;
}

// ADC 関連関数
static void adc_module_init(void)
{
    // ADC1 の初期化
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC1 ユニット初期化に失敗しました: %s", esp_err_to_name(ret));
        return;
    }

    // ADC 設定の初期化
    for (int i = 0; i < 40; i++)
    {
        adc_configs[i].channel = -1;
        adc_configs[i].attenuation = ADC_ATTEN_DB_12;
        adc_configs[i].calibrated = false;
        adc_configs[i].cali_handle = NULL;
    }

    ESP_LOGI(TAG, "ADC module initialized (12-bit resolution, ADC1 only)");
}

static adc_channel_t gpio_to_adc1_channel(uint8_t gpio_num)
{
    for (int i = 0; i < ADC1_GPIO_MAP_SIZE; i++)
    {
        if (adc1_gpio_map[i].gpio_num == gpio_num)
        {
            return adc1_gpio_map[i].channel;
        }
    }
    return -1; // ADC1 に対応していない
}

static esp_err_t gpio_enable_adc(uint8_t pin, uint8_t atten_param)
{
    // パラメータ検証
    adc_channel_t channel = gpio_to_adc1_channel(pin);
    if (channel < 0)
    {
        ESP_LOGE(TAG, "GPIO%d は ADC1 に対応していません (対応ピン: 32, 33, 34, 35, 36, 39)", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (atten_param >= ADC_ATTEN_MAP_SIZE)
    {
        ESP_LOGE(TAG, "無効な減衰率パラメータ: %d (最大: %d)", atten_param, ADC_ATTEN_MAP_SIZE - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 既存のモードをクリーンアップ
    portENTER_CRITICAL(&gpio_states_mux);
    bleio_mode_state_t prev_mode = gpio_states[pin].mode;
    portEXIT_CRITICAL(&gpio_states_mux);

    if (prev_mode == BLEIO_MODE_PWM)
    {
        free_ledc_channel(pin);
    }

    // 既存のキャリブレーションハンドルを削除
    if (adc_configs[pin].cali_handle != NULL)
    {
        adc_cali_delete_scheme_line_fitting(adc_configs[pin].cali_handle);
        adc_configs[pin].cali_handle = NULL;
        adc_configs[pin].calibrated = false;
    }

    // ADC チャネルの設定
    adc_atten_t attenuation = adc_atten_map[atten_param];
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = attenuation,
    };
    esp_err_t ret = adc_oneshot_config_channel(adc1_handle, channel, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC チャネル設定に失敗しました: %s", esp_err_to_name(ret));
        return ret;
    }

    // キャリブレーションの設定
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &adc_configs[pin].cali_handle);
    bool calibrated = (ret == ESP_OK);

    // 設定を保存
    adc_configs[pin].channel = channel;
    adc_configs[pin].attenuation = attenuation;
    adc_configs[pin].calibrated = calibrated;

    portENTER_CRITICAL(&gpio_states_mux);
    gpio_states[pin].mode = BLEIO_MODE_ADC;
    portEXIT_CRITICAL(&gpio_states_mux);

    const char *range_str =
        (atten_param == 0) ? "0-1.1V" : (atten_param == 1) ? "0-1.5V"
                                    : (atten_param == 2)   ? "0-2.2V"
                                                           : "0-3.3V";

    ESP_LOGI(TAG, "GPIO%d を ADC モードに設定しました (チャネル: %d, 減衰: %d dB, 範囲: %s, キャリブレーション: %s)",
             pin, channel, atten_param, range_str, calibrated ? "成功" : "失敗");

    return ESP_OK;
}

static esp_err_t gpio_disable_adc(uint8_t pin)
{
    portENTER_CRITICAL(&gpio_states_mux);
    bleio_mode_state_t mode = gpio_states[pin].mode;
    portEXIT_CRITICAL(&gpio_states_mux);

    if (mode != BLEIO_MODE_ADC)
    {
        ESP_LOGW(TAG, "GPIO%d は ADC モードではありません", pin);
        return ESP_OK;
    }

    // キャリブレーションハンドルを削除
    if (adc_configs[pin].cali_handle != NULL)
    {
        adc_cali_delete_scheme_line_fitting(adc_configs[pin].cali_handle);
        adc_configs[pin].cali_handle = NULL;
    }

    // 設定をクリア
    adc_configs[pin].channel = -1;
    adc_configs[pin].calibrated = false;

    portENTER_CRITICAL(&gpio_states_mux);
    gpio_states[pin].mode = BLEIO_MODE_UNSET;
    portEXIT_CRITICAL(&gpio_states_mux);

    ESP_LOGI(TAG, "GPIO%d の ADC モードを無効化しました", pin);
    return ESP_OK;
}

static uint16_t read_adc_value(uint8_t pin)
{
    adc_config_t *config = &adc_configs[pin];

    if (config->channel < 0)
    {
        ESP_LOGW(TAG, "GPIO%d は ADC モードではありません", pin);
        return 0;
    }

    // 生の ADC 値を読み取り
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, config->channel, &raw);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO%d の ADC 読み取りに失敗しました: %s", pin, esp_err_to_name(ret));
        return 0;
    }

    // キャリブレーションが有効な場合は補正された値を使用
    if (config->calibrated && config->cali_handle != NULL)
    {
        int voltage = 0;
        ret = adc_cali_raw_to_voltage(config->cali_handle, raw, &voltage);
        if (ret == ESP_OK)
        {
            // 電圧 (mV) を ADC 値に逆変換 (0-4095)
            // 減衰率に応じた最大電圧で正規化
            uint32_t max_voltage =
                (config->attenuation == ADC_ATTEN_DB_0) ? 1100 : (config->attenuation == ADC_ATTEN_DB_2_5) ? 1500
                                                             : (config->attenuation == ADC_ATTEN_DB_6)     ? 2200
                                                                                                           : 3300;

            uint16_t normalized = (voltage * 4095) / max_voltage;
            return (normalized > 4095) ? 4095 : normalized;
        }
    }

    return (uint16_t)raw;
}

static esp_err_t gpio_set_mode(uint8_t pin, uint8_t command, uint8_t latch_mode)
{
    if (!is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // PWM が有効な場合は停止
    stop_pwm_if_active(pin);

    // ADC が有効な場合は停止
    stop_adc_if_active(pin);

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    bleio_gpio_state_t *state = &gpio_states[pin];

    switch (command)
    {
    case CMD_SET_INPUT_FLOATING:
        io_conf.mode = GPIO_MODE_INPUT;
        gpio_config(&io_conf);
        portENTER_CRITICAL(&gpio_states_mux);
        state->mode = BLEIO_MODE_INPUT_FLOATING;
        state->latch_mode = latch_mode;
        state->is_latched = false;
        state->stable_counter = 0;
        state->last_level = 0;
        portEXIT_CRITICAL(&gpio_states_mux);
        ESP_LOGI(TAG, "Set GPIO%d to INPUT_FLOATING (latch_mode=%d)", pin, latch_mode);
        break;
    case CMD_SET_INPUT_PULLUP:
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);
        portENTER_CRITICAL(&gpio_states_mux);
        state->mode = BLEIO_MODE_INPUT_PULLUP;
        state->latch_mode = latch_mode;
        state->is_latched = false;
        state->stable_counter = 0;
        state->last_level = 0;
        portEXIT_CRITICAL(&gpio_states_mux);
        ESP_LOGI(TAG, "Set GPIO%d to INPUT_PULLUP (latch_mode=%d)", pin, latch_mode);
        break;
    case CMD_SET_INPUT_PULLDOWN:
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        gpio_config(&io_conf);
        portENTER_CRITICAL(&gpio_states_mux);
        state->mode = BLEIO_MODE_INPUT_PULLDOWN;
        state->latch_mode = latch_mode;
        state->is_latched = false;
        state->stable_counter = 0;
        state->last_level = 0;
        portEXIT_CRITICAL(&gpio_states_mux);
        ESP_LOGI(TAG, "Set GPIO%d to INPUT_PULLDOWN (latch_mode=%d)", pin, latch_mode);
        break;
    default:
        ESP_LOGE(TAG, "Invalid mode command: %d", command);
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static esp_err_t gpio_write_level(uint8_t pin, uint8_t command)
{
    if (!is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // PWM が有効な場合は停止
    stop_pwm_if_active(pin);

    // ADC が有効な場合は停止
    stop_adc_if_active(pin);

    // GPIO を出力モードに設定
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_2);

    bleio_gpio_state_t *state = &gpio_states[pin];
    uint32_t level;
    bleio_mode_state_t new_mode;

    switch (command)
    {
    case CMD_SET_OUTPUT_LOW:
        level = 0;
        new_mode = BLEIO_MODE_OUTPUT_LOW;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_LOW", pin);
        break;
    case CMD_SET_OUTPUT_HIGH:
        level = 1;
        new_mode = BLEIO_MODE_OUTPUT_HIGH;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_HIGH", pin);
        break;
    default:
        ESP_LOGE(TAG, "Invalid write command: %d", command);
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&gpio_states_mux);
    state->mode = new_mode;
    portEXIT_CRITICAL(&gpio_states_mux);

    return gpio_set_level(pin, level);
}

static esp_err_t gpio_start_blink(uint8_t pin, uint8_t command)
{
    if (!is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // PWM が有効な場合は停止
    stop_pwm_if_active(pin);

    // ADC が有効な場合は停止
    stop_adc_if_active(pin);

    // GPIO を出力モードに設定
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_drive_capability(pin, GPIO_DRIVE_CAP_2);

    bleio_gpio_state_t *state = &gpio_states[pin];
    bleio_mode_state_t new_mode;

    switch (command)
    {
    case CMD_SET_OUTPUT_BLINK_250MS:
        new_mode = BLEIO_MODE_BLINK_250MS;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_BLINK_250MS", pin);
        break;
    case CMD_SET_OUTPUT_BLINK_500MS:
        new_mode = BLEIO_MODE_BLINK_500MS;
        ESP_LOGI(TAG, "Set GPIO%d to OUTPUT_BLINK_500MS", pin);
        break;
    default:
        ESP_LOGE(TAG, "Invalid blink command: %d", command);
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&gpio_states_mux);
    state->mode = new_mode;
    state->current_level = 0;
    portEXIT_CRITICAL(&gpio_states_mux);

    gpio_set_level(pin, 0);
    return ESP_OK;
}

static esp_err_t gpio_set_disconnect_behavior(uint8_t pin, uint8_t behavior)
{
    if (!is_valid_gpio(pin))
    {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    if (behavior > 2)
    {
        ESP_LOGE(TAG, "Invalid disconnect behavior: %d (0-2)", behavior);
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&gpio_states_mux);
    gpio_states[pin].disconnect_behavior = behavior;
    portEXIT_CRITICAL(&gpio_states_mux);

    const char *behavior_str = (behavior == 0) ? "維持" : (behavior == 1) ? "LOW" : "HIGH";
    ESP_LOGI(TAG, "GPIO%d の切断時の振る舞いを設定: %s", pin, behavior_str);

    return ESP_OK;
}

// BLE キャラクタリスティック コールバック
static int gatt_svr_chr_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        return BLE_ATT_ERR_UNLIKELY;
    }

    struct os_mbuf *om = ctxt->om;
    uint16_t len = OS_MBUF_PKTLEN(om);

    // 最小長チェック: 1 (コマンド個数) + 4 (最低1コマンド)
    if (len < 5)
    {
        ESP_LOGE(TAG, "Invalid write length: %d (minimum 5)", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t cmd_count;
    os_mbuf_copydata(om, 0, 1, &cmd_count);

    // パケット長チェック
    uint16_t expected_len = 1 + (cmd_count * 4);
    if (len != expected_len)
    {
        ESP_LOGE(TAG, "Invalid packet length: %d (expected %d for %d commands)",
                 len, expected_len, cmd_count);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    ESP_LOGI(TAG, "Received %d commands", cmd_count);

    // 各コマンドを処理
    for (int i = 0; i < cmd_count; i++)
    {
        uint8_t cmd_data[4];
        os_mbuf_copydata(om, 1 + (i * 4), 4, cmd_data);

        uint8_t pin = cmd_data[0];
        uint8_t command = cmd_data[1];
        uint8_t param1 = cmd_data[2];
        uint8_t param2 = cmd_data[3];

        ESP_LOGI(TAG, "Command %d: pin=%d, command=%d, param1=%d, param2=%d",
                 i + 1, pin, command, param1, param2);

        esp_err_t ret;
        if (command == CMD_SET_OUTPUT_LOW || command == CMD_SET_OUTPUT_HIGH)
        {
            ret = gpio_write_level(pin, command);
        }
        else if (command == CMD_SET_OUTPUT_BLINK_250MS || command == CMD_SET_OUTPUT_BLINK_500MS)
        {
            ret = gpio_start_blink(pin, command);
        }
        else if (command == CMD_SET_OUTPUT_PWM)
        {
            ret = gpio_set_pwm(pin, param1, param2); // param1 = duty_cycle, param2 = freq_preset
        }
        else if (command == CMD_SET_OUTPUT_ON_DISCONNECT)
        {
            ret = gpio_set_disconnect_behavior(pin, param1); // param1 = disconnect_behavior
        }
        else if (command >= CMD_SET_INPUT_FLOATING && command <= CMD_SET_INPUT_PULLDOWN)
        {
            ret = gpio_set_mode(pin, command, param1); // param1 = latch_mode
        }
        else if (command == CMD_SET_ADC_ENABLE)
        {
            ret = gpio_enable_adc(pin, param1); // param1 = attenuation
        }
        else if (command == CMD_SET_ADC_DISABLE)
        {
            ret = gpio_disable_adc(pin);
        }
        else
        {
            ESP_LOGE(TAG, "Unknown command: %d", command);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Command %d failed", i + 1);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return 0;
}

static int gatt_svr_chr_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        // READ 操作: すべての入力モード設定済みピンの状態を返す
        uint8_t buffer[1 + MAX_USABLE_GPIO * 2]; // 1 (カウント) + 24 * 2 (ピン番号と状態) = 49 バイト
        uint8_t count = 0;

        // すべての GPIO をスキャンして、入力モードのピンを収集
        for (int pin = 0; pin < 40; pin++)
        {
            if (!is_valid_gpio(pin))
            {
                continue;
            }

            portENTER_CRITICAL(&gpio_states_mux);
            bleio_gpio_state_t *state = &gpio_states[pin];
            bleio_mode_state_t mode = state->mode;
            uint8_t latch_mode = state->latch_mode;
            bool is_latched = state->is_latched;
            portEXIT_CRITICAL(&gpio_states_mux);

            // 入力モードかチェック
            if (mode == BLEIO_MODE_INPUT_FLOATING ||
                mode == BLEIO_MODE_INPUT_PULLUP ||
                mode == BLEIO_MODE_INPUT_PULLDOWN)
            {

                uint8_t level;

                // ラッチモードの処理
                if (latch_mode == LATCH_MODE_NONE)
                {
                    // ラッチなし: 現在の GPIO レベルをそのまま返す
                    level = gpio_get_level(pin);
                }
                else
                {
                    // ラッチあり
                    if (is_latched)
                    {
                        // ラッチ済み: ターゲット値を返す
                        level = (latch_mode == LATCH_MODE_HIGH) ? 1 : 0;
                    }
                    else
                    {
                        // 未ラッチ: ターゲット値の逆を返す (過渡状態での誤検出を避ける)
                        level = (latch_mode == LATCH_MODE_HIGH) ? 0 : 1;
                    }
                }

                buffer[1 + count * 2] = pin;
                buffer[1 + count * 2 + 1] = level;
                count++;

                ESP_LOGI(TAG, "Input GPIO%d: %s (latch_mode=%d, is_latched=%d)",
                         pin, level ? "HIGH" : "LOW", latch_mode, is_latched);
            }
        }

        buffer[0] = count;
        uint16_t data_len = 1 + count * 2;

        ESP_LOGI(TAG, "Sending %d input states (%d bytes)", count, data_len);
        int rc = os_mbuf_append(ctxt->om, buffer, data_len);
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    // WRITE 操作は無効
    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_chr_adc_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        // READ 操作: すべての ADC モード設定済みピンの値を返す
        uint8_t buffer[1 + MAX_USABLE_GPIO * 3]; // 1 (カウント) + 24 * 3 (ピン番号と ADC 値) = 73 バイト
        uint8_t count = 0;

        // すべての GPIO をスキャンして、ADC モードのピンを収集
        for (int pin = 0; pin < 40; pin++)
        {
            if (!is_valid_gpio(pin))
            {
                continue;
            }

            portENTER_CRITICAL(&gpio_states_mux);
            bleio_mode_state_t mode = gpio_states[pin].mode;
            portEXIT_CRITICAL(&gpio_states_mux);

            // ADC モードかチェック
            if (mode == BLEIO_MODE_ADC)
            {
                uint16_t adc_value = read_adc_value(pin);

                buffer[1 + count * 3] = pin;
                buffer[1 + count * 3 + 1] = adc_value & 0xFF;        // 下位バイト
                buffer[1 + count * 3 + 2] = (adc_value >> 8) & 0xFF; // 上位バイト
                count++;

                ESP_LOGI(TAG, "ADC GPIO%d: %d (0x%03X)", pin, adc_value, adc_value);
            }
        }

        buffer[0] = count;
        uint16_t data_len = 1 + count * 3;

        ESP_LOGI(TAG, "Sending %d ADC values (%d bytes)", count, data_len);
        int rc = os_mbuf_append(ctxt->om, buffer, data_len);
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    // WRITE 操作は無効
    return BLE_ATT_ERR_UNLIKELY;
}

// GATT サービス定義
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                // GPIO 書き込みキャラクタリスティック
                .uuid = &gatt_svr_chr_write_uuid.u,
                .access_cb = gatt_svr_chr_write_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                // GPIO 読み取りキャラクタリスティック
                .uuid = &gatt_svr_chr_read_uuid.u,
                .access_cb = gatt_svr_chr_read_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                // ADC 読み取りキャラクタリスティック
                .uuid = &gatt_svr_chr_adc_read_uuid.u,
                .access_cb = gatt_svr_chr_adc_read_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                0, // 終端
            }},
    },
    {
        0, // 終端
    },
};

// BLE イベントハンドラ
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0)
        {
            conn_handle = event->connect.conn_handle;

            // 接続パラメータを更新して電力変動を抑制
            // より長い Connection Interval を使用することで電力ピークを分散
            struct ble_gap_upd_params params = {
                .itvl_min = BLE_GAP_INITIAL_CONN_ITVL_MAX,  // 30ms
                .itvl_max = BLE_GAP_INITIAL_CONN_ITVL_MAX,  // 30ms
                .latency = 0,
                .supervision_timeout = 500,  // 5000ms
                .min_ce_len = BLE_GAP_INITIAL_CONN_MIN_CE_LEN,
                .max_ce_len = BLE_GAP_INITIAL_CONN_MAX_CE_LEN,
            };
            ble_gap_update_params(conn_handle, &params);
            ESP_LOGI(TAG, "Updated connection parameters for stable power consumption");
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnect; reason=%d", event->disconnect.reason);
        conn_handle = 0;

        // 切断時の振る舞いに従って GPIO を設定
        for (int pin = 0; pin < 40; pin++)
        {
            if (!is_valid_gpio(pin))
            {
                continue;
            }

            portENTER_CRITICAL(&gpio_states_mux);
            bleio_mode_state_t mode = gpio_states[pin].mode;
            uint8_t disconnect_behavior = gpio_states[pin].disconnect_behavior;
            portEXIT_CRITICAL(&gpio_states_mux);

            // 出力モードかつ切断時の振る舞いが設定されている場合
            if ((mode == BLEIO_MODE_OUTPUT_LOW || mode == BLEIO_MODE_OUTPUT_HIGH ||
                 mode == BLEIO_MODE_BLINK_250MS || mode == BLEIO_MODE_BLINK_500MS ||
                 mode == BLEIO_MODE_PWM) &&
                disconnect_behavior != 0)
            {
                if (disconnect_behavior == 1)
                {
                    // LOW に設定
                    gpio_write_level(pin, CMD_SET_OUTPUT_LOW);
                    ESP_LOGI(TAG, "切断したため GPIO%d を LOW に設定しました", pin);
                }
                else if (disconnect_behavior == 2)
                {
                    // HIGH に設定
                    gpio_write_level(pin, CMD_SET_OUTPUT_HIGH);
                    ESP_LOGI(TAG, "切断したため GPIO%d を HIGH に設定しました", pin);
                }
            }
        }

        // 再度アドバタイズを開始
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE advertise complete");
        break;
    }
    return 0;
}

// BLE セキュリティ設定
static void ble_app_set_security(void)
{
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;  // ボンディング有効
    ble_hs_cfg.sm_mitm = 0;     // MITM 保護なし (NoInputNoOutput のため)
    ble_hs_cfg.sm_sc = 1;       // Secure Connections 有効
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC;
}

// BLE アドバタイズ開始
static void ble_app_advertise(void)
{
    // Advertising data: Flags + 128-bit Service UUID(s)
    struct ble_hs_adv_fields adv;
    memset(&adv, 0, sizeof(adv));

    // 一般発見 + BR/EDR 非対応
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // 128-bit サービス UUID を広告に載せる
    adv.uuids128 = &gatt_svr_svc_uuid;
    adv.num_uuids128 = 1; // 複数必要な場合は配列を用意して num_uuids128 を増やす。
    adv.uuids128_is_complete = 1; // 完全リストとして扱う (必要に応じて 0 でも可)

    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        return;
    }

    // Scan Response: デバイス名を応答する
    struct ble_hs_adv_fields rsp;
    memset(&rsp, 0, sizeof(rsp));
    rsp.name = (uint8_t *)ble_svc_gap_device_name();
    rsp.name_len = strlen((char *)rsp.name);
    rsp.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields failed: rc=%d", rc);
        return;
    }

    // アドバタイズ・パラメータ
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;         // 接続可
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;         // 一般発見可能
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN; // 30ms
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX; // 60ms

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "Advertising with 128-bit Service UUID");
    }
}

// BLE 同期コールバック
static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "BLE host synchronized");

    // MAC アドレスを取得
    uint8_t own_addr_type;
    uint8_t addr[6];
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to infer address type: rc=%d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to copy address: rc=%d", rc);
        return;
    }

    // MAC アドレスの下位3バイトを使用してデバイス名を生成
    // addr[0] が最下位バイト、addr[5] が最上位バイト
    // 下位3バイトを表示するには addr[2], addr[1], addr[0] の順
    char device_name[32];
    if (auth_mode_enabled)
    {
        snprintf(device_name, sizeof(device_name), "BLEIO %02x%02x%02x [SEC]",
                 addr[2], addr[1], addr[0]);
    }
    else
    {
        snprintf(device_name, sizeof(device_name), "BLEIO %02x%02x%02x",
                 addr[2], addr[1], addr[0]);
    }

    // デバイス名を設定
    ble_svc_gap_device_name_set(device_name);
    ESP_LOGI(TAG, "Device name set to: %s", device_name);

    ble_hs_util_ensure_addr(0);
    ble_app_advertise();
}

// BLE ホストタスク
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting BLEIO-ESP32 Service");

    // NVS 初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // PWM 初期化
    pwm_init();

    // ADC 初期化
    adc_module_init();

    // 点滅タイマの初期化 (250ms 周期)
    const esp_timer_create_args_t blink_timer_args = {
        .callback = &blink_timer_callback,
        .name = "blink_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &blink_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(blink_timer, 250000)); // 250ms = 250000us
    ESP_LOGI(TAG, "Blink timer started (250ms interval)");

    // 入力ポーリングタイマの初期化 (10ms 周期)
    const esp_timer_create_args_t input_poll_timer_args = {
        .callback = &input_poll_timer_callback,
        .name = "input_poll_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&input_poll_timer_args, &input_poll_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(input_poll_timer, INPUT_POLL_INTERVAL_MS * 1000)); // 10ms = 10000us
    ESP_LOGI(TAG, "Input poll timer started (%dms interval)", INPUT_POLL_INTERVAL_MS);

    // 認証機能の有効/無効をチェック
    auth_mode_enabled = is_auth_enabled();

    if (auth_mode_enabled)
    {
        ESP_LOGI(TAG, "認証機能が有効です (GPIO%d = LOW)", GPIO_AUTH_ENABLE);

        // ペアリングモードのチェック (認証有効時のみ)
        bool pairing_mode = is_pairing_mode_requested();

        if (pairing_mode)
        {
            ESP_LOGI(TAG, "ペアリングモードで起動します (GPIO%d = LOW) - ボンディング情報をクリアします", GPIO_PAIRING_MODE);
        }
        else
        {
            ESP_LOGI(TAG, "認証有効モードで起動します (GPIO%d = HIGH)", GPIO_PAIRING_MODE);
        }
    }
    else
    {
        ESP_LOGI(TAG, "認証機能が無効です (GPIO%d = HIGH)", GPIO_AUTH_ENABLE);
    }

    // NimBLE 初期化
    ESP_ERROR_CHECK(nimble_port_init());

    // BLE 送信電力を設定 (電力変動を抑制)
    // ESP_PWR_LVL_N12 ~ ESP_PWR_LVL_P9 の範囲で設定可能
    // ESP_PWR_LVL_P3 (3dBm) 程度が安定性と到達距離のバランスが良い
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
    ESP_LOGI(TAG, "BLE TX power set to +3dBm for stable power consumption");

    // GATT サービス初期化
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // 認証有効時のみセキュリティ設定とボンディング情報のクリア
    if (auth_mode_enabled)
    {
        ble_app_set_security();

        // ペアリングモードの場合、ボンディング情報をクリア
        if (is_pairing_mode_requested())
        {
            clear_bonding_info();
        }
    }

    // ATT MTU を設定 (最大 22 コマンドまで送信可能)
    // 必要 MTU = ATT ヘッダ (3) + コマンド個数 (1) + コマンド (22 * 4) = 92 バイト
    ble_att_set_preferred_mtu(REQUIRED_MTU);
    ESP_LOGI(TAG, "Set preferred MTU to %d bytes (max %d commands, payload %d bytes)",
             REQUIRED_MTU, MAX_USABLE_GPIO, PAYLOAD_SIZE);

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    // BLE ホストタスク起動
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE initialization complete");
}
