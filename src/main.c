#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

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

// GPIO コマンド定義
#define CMD_SET_OUTPUT              0
#define CMD_SET_INPUT_FLOATING      1
#define CMD_SET_INPUT_PULLUP        2
#define CMD_SET_INPUT_PULLDOWN      3
#define CMD_WRITE_LOW               10
#define CMD_WRITE_HIGH              11
#define CMD_BLINK_500MS             12
#define CMD_BLINK_250MS             13

// GPIO 最大数
#define MAX_USABLE_GPIO             24  // 使用可能な GPIO の総数

// BLE ATT MTU 計算
// WRITE データ構造: 1 (コマンド個数) + MAX_USABLE_GPIO * 4 (各コマンド 4 バイト)
// ATT ヘッダ: 3 バイト (Opcode 1 + Attribute Handle 2)
// 必要な MTU = ATT ヘッダ (3) + ペイロード (1 + 24 * 4) = 3 + 97 = 100 バイト
#define ATT_HEADER_SIZE             3
#define COMMAND_HEADER_SIZE         1   // コマンド個数フィールド
#define COMMAND_SIZE                4   // 各コマンドのサイズ (Pin + Command + Param1 + Param2)
#define PAYLOAD_SIZE                (COMMAND_HEADER_SIZE + (MAX_USABLE_GPIO * COMMAND_SIZE))
#define REQUIRED_MTU                (ATT_HEADER_SIZE + PAYLOAD_SIZE)

// 入力ラッチモード定義
#define LATCH_MODE_NONE             0   // ラッチなし
#define LATCH_MODE_LOW              1   // LOWラッチ
#define LATCH_MODE_HIGH             2   // HIGHラッチ

// ポーリング設定
#define INPUT_POLL_INTERVAL_MS      10  // 入力ポーリング間隔 (ms)
#define LATCH_STABLE_COUNT          2   // ラッチ判定に必要な連続安定回数

// GPIO モード状態の定義
typedef enum {
    BLEIO_MODE_UNSET = 0,           // モード未設定 (初期状態)
    BLEIO_MODE_INPUT_FLOATING,      // ハイインピーダンス入力モード
    BLEIO_MODE_INPUT_PULLUP,        // 内部プルアップ付き入力モード
    BLEIO_MODE_INPUT_PULLDOWN,      // 内部プルダウン付き入力モード
    BLEIO_MODE_OUTPUT_LOW,          // LOW (0V) 出力モード
    BLEIO_MODE_OUTPUT_HIGH,         // HIGH (3.3V) 出力モード
    BLEIO_MODE_BLINK_250MS,         // 250ms 点滅出力モード
    BLEIO_MODE_BLINK_500MS          // 500ms 点滅出力モード
} bleio_mode_state_t;

// GPIO ごとの状態管理
typedef struct {
    bleio_mode_state_t mode;       // 現在のモード
    uint8_t current_level;         // 現在の出力レベル (点滅時に使用)
    uint8_t blink_counter;         // 点滅用カウンタ (500ms 用)
    uint8_t latch_mode;            // 入力ラッチモード (0-2)
    bool is_latched;               // ラッチ済みフラグ
    uint8_t stable_counter;        // 安定カウンタ
    uint8_t last_level;            // 前回の読み取り値
} bleio_gpio_state_t;

// グローバル変数
static uint16_t conn_handle = 0;
static bleio_gpio_state_t gpio_states[40] = {0};  // 全 GPIO の状態
static esp_timer_handle_t blink_timer = NULL;
static esp_timer_handle_t input_poll_timer = NULL;
static portMUX_TYPE gpio_states_mux = portMUX_INITIALIZER_UNLOCKED;  // gpio_states 保護用スピンロック

// 関数の前方宣言
static void ble_app_advertise(void);
static bool is_valid_gpio(uint8_t pin);

// 点滅タイマコールバック (250ms 周期)
static void blink_timer_callback(void* arg) {
    for (int pin = 0; pin < 40; pin++) {
        if (!is_valid_gpio(pin)) {
            continue;
        }

        portENTER_CRITICAL(&gpio_states_mux);
        bleio_gpio_state_t *state = &gpio_states[pin];
        bleio_mode_state_t mode = state->mode;
        uint8_t current_level = state->current_level;
        uint8_t blink_counter = state->blink_counter;
        portEXIT_CRITICAL(&gpio_states_mux);

        bool should_toggle = false;
        uint8_t new_level = current_level;
        uint8_t new_counter = blink_counter;

        if (mode == BLEIO_MODE_BLINK_250MS) {
            // 250ms ごとにトグル
            should_toggle = true;
            new_level = !current_level;
        }
        else if (mode == BLEIO_MODE_BLINK_500MS) {
            // カウンタで 2 回に 1 回トグル
            new_counter++;
            if (new_counter >= 2) {
                new_counter = 0;
                should_toggle = true;
                new_level = !current_level;
            }
        }

        if (should_toggle || mode == BLEIO_MODE_BLINK_500MS) {
            portENTER_CRITICAL(&gpio_states_mux);
            state->current_level = new_level;
            state->blink_counter = new_counter;
            portEXIT_CRITICAL(&gpio_states_mux);

            if (should_toggle) {
                gpio_set_level(pin, new_level);
            }
        }
    }
}

// 入力ポーリングタイマコールバック (10ms 周期)
static void input_poll_timer_callback(void* arg) {
    for (int pin = 0; pin < 40; pin++) {
        if (!is_valid_gpio(pin)) {
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
        if (latch_mode == LATCH_MODE_NONE || is_latched) {
            continue;
        }

        // 入力モードかチェック
        if (mode != BLEIO_MODE_INPUT_FLOATING &&
            mode != BLEIO_MODE_INPUT_PULLUP &&
            mode != BLEIO_MODE_INPUT_PULLDOWN) {
            continue;
        }

        uint8_t level = gpio_get_level(pin);
        uint8_t target = (latch_mode == LATCH_MODE_HIGH) ? 1 : 0;

        bool new_is_latched = is_latched;
        uint8_t new_stable_counter = stable_counter;

        if (level == target) {
            if (last_level == target) {
                new_stable_counter++;
                if (new_stable_counter >= LATCH_STABLE_COUNT) {
                    new_is_latched = true;
                    ESP_LOGI(TAG, "GPIO%d latched to %s", pin, target ? "HIGH" : "LOW");
                }
            } else {
                new_stable_counter = 1;
            }
        } else {
            new_stable_counter = 0;
        }

        portENTER_CRITICAL(&gpio_states_mux);
        state->is_latched = new_is_latched;
        state->stable_counter = new_stable_counter;
        state->last_level = level;
        portEXIT_CRITICAL(&gpio_states_mux);
    }
}

// GPIO 制御関数
static bool is_valid_gpio(uint8_t pin) {
    // 使用可能な GPIO ピン
    if (pin == 2 || (pin >= 4 && pin <= 5) || (pin >= 12 && pin <= 19) ||
        (pin >= 21 && pin <= 27) || (pin >= 32 && pin <= 36) || pin == 39) {
        return true;
    }
    return false;
}

static esp_err_t gpio_set_mode(uint8_t pin, uint8_t command, uint8_t latch_mode) {
    if (!is_valid_gpio(pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    bleio_gpio_state_t *state = &gpio_states[pin];

    switch (command) {
        case CMD_SET_OUTPUT:
            io_conf.mode = GPIO_MODE_OUTPUT;
            gpio_config(&io_conf);
            // 前回が HIGH なら HIGH を維持、それ以外は LOW にする
            portENTER_CRITICAL(&gpio_states_mux);
            bleio_mode_state_t prev_mode = state->mode;
            portEXIT_CRITICAL(&gpio_states_mux);

            if (prev_mode == BLEIO_MODE_OUTPUT_HIGH) {
                gpio_set_level(pin, 1);
                ESP_LOGI(TAG, "Set GPIO%d to OUTPUT (maintain HIGH)", pin);
            } else {
                gpio_set_level(pin, 0);
                portENTER_CRITICAL(&gpio_states_mux);
                state->mode = BLEIO_MODE_OUTPUT_LOW;
                portEXIT_CRITICAL(&gpio_states_mux);
                ESP_LOGI(TAG, "Set GPIO%d to OUTPUT (set to LOW)", pin);
            }
            break;
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

static esp_err_t gpio_write_level(uint8_t pin, uint8_t command) {
    if (!is_valid_gpio(pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // GPIO を出力モードに設定
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    bleio_gpio_state_t *state = &gpio_states[pin];
    uint32_t level;
    bleio_mode_state_t new_mode;

    switch (command) {
        case CMD_WRITE_LOW:
            level = 0;
            new_mode = BLEIO_MODE_OUTPUT_LOW;
            ESP_LOGI(TAG, "Write GPIO%d to LOW", pin);
            break;
        case CMD_WRITE_HIGH:
            level = 1;
            new_mode = BLEIO_MODE_OUTPUT_HIGH;
            ESP_LOGI(TAG, "Write GPIO%d to HIGH", pin);
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

static esp_err_t gpio_start_blink(uint8_t pin, uint8_t command) {
    if (!is_valid_gpio(pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // GPIO を出力モードに設定
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    bleio_gpio_state_t *state = &gpio_states[pin];
    bleio_mode_state_t new_mode;

    switch (command) {
        case CMD_BLINK_250MS:
            new_mode = BLEIO_MODE_BLINK_250MS;
            ESP_LOGI(TAG, "Start GPIO%d blinking at 250ms", pin);
            break;
        case CMD_BLINK_500MS:
            new_mode = BLEIO_MODE_BLINK_500MS;
            ESP_LOGI(TAG, "Start GPIO%d blinking at 500ms", pin);
            break;
        default:
            ESP_LOGE(TAG, "Invalid blink command: %d", command);
            return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&gpio_states_mux);
    state->mode = new_mode;
    state->current_level = 0;
    state->blink_counter = 0;
    portEXIT_CRITICAL(&gpio_states_mux);

    gpio_set_level(pin, 0);
    return ESP_OK;
}

// BLE キャラクタリスティック コールバック
static int gatt_svr_chr_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    struct os_mbuf *om = ctxt->om;
    uint16_t len = OS_MBUF_PKTLEN(om);

    // 最小長チェック: 1 (コマンド個数) + 4 (最低1コマンド)
    if (len < 5) {
        ESP_LOGE(TAG, "Invalid write length: %d (minimum 5)", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t cmd_count;
    os_mbuf_copydata(om, 0, 1, &cmd_count);

    // パケット長チェック
    uint16_t expected_len = 1 + (cmd_count * 4);
    if (len != expected_len) {
        ESP_LOGE(TAG, "Invalid packet length: %d (expected %d for %d commands)",
                 len, expected_len, cmd_count);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    ESP_LOGI(TAG, "Received %d commands", cmd_count);

    // 各コマンドを処理
    for (int i = 0; i < cmd_count; i++) {
        uint8_t cmd_data[4];
        os_mbuf_copydata(om, 1 + (i * 4), 4, cmd_data);

        uint8_t pin = cmd_data[0];
        uint8_t command = cmd_data[1];
        uint8_t param1 = cmd_data[2];
        uint8_t param2 = cmd_data[3];

        ESP_LOGI(TAG, "Command %d: pin=%d, command=%d, param1=%d, param2=%d",
                 i + 1, pin, command, param1, param2);

        esp_err_t ret;
        if (command <= CMD_SET_INPUT_PULLDOWN) {
            ret = gpio_set_mode(pin, command, param1);  // param1 = latch_mode
        } else if (command == CMD_WRITE_LOW || command == CMD_WRITE_HIGH) {
            ret = gpio_write_level(pin, command);
        } else if (command == CMD_BLINK_500MS || command == CMD_BLINK_250MS) {
            ret = gpio_start_blink(pin, command);
        } else {
            ESP_LOGE(TAG, "Unknown command: %d", command);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Command %d failed", i + 1);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return 0;
}

static int gatt_svr_chr_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // READ 操作: すべての入力モード設定済みピンの状態を返す
        uint8_t buffer[1 + MAX_USABLE_GPIO * 2];  // 1 (カウント) + 24 * 2 (ピン番号と状態) = 49 バイト
        uint8_t count = 0;

        // すべての GPIO をスキャンして、入力モードのピンを収集
        for (int pin = 0; pin < 40; pin++) {
            if (!is_valid_gpio(pin)) {
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
                mode == BLEIO_MODE_INPUT_PULLDOWN) {

                uint8_t level;

                // ラッチモードの処理
                if (latch_mode == LATCH_MODE_NONE) {
                    // ラッチなし: 現在の GPIO レベルをそのまま返す
                    level = gpio_get_level(pin);
                } else {
                    // ラッチあり
                    if (is_latched) {
                        // ラッチ済み: ターゲット値を返す
                        level = (latch_mode == LATCH_MODE_HIGH) ? 1 : 0;
                    } else {
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

// GATT サービス定義
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
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
                0, // 終端
            }
        },
    },
    {
        0, // 終端
    },
};

// BLE イベントハンドラ
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE connection %s; status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
            }
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE disconnect; reason=%d", event->disconnect.reason);
            conn_handle = 0;
            // 再度アドバタイズを開始
            ble_app_advertise();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "BLE advertise complete");
            break;
    }
    return 0;
}

// BLE アドバタイズ開始
static void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;  // 30ms
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;  // 60ms

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, ble_gap_event, NULL);
}

// BLE 同期コールバック
static void ble_app_on_sync(void) {
    ESP_LOGI(TAG, "BLE host synchronized");
    ble_hs_util_ensure_addr(0);
    ble_app_advertise();
}

// BLE ホストタスク
static void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting BLEIO-ESP32 Service");

    // NVS 初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 点滅タイマの初期化 (250ms 周期)
    const esp_timer_create_args_t blink_timer_args = {
        .callback = &blink_timer_callback,
        .name = "blink_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &blink_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(blink_timer, 250000));  // 250ms = 250000us
    ESP_LOGI(TAG, "Blink timer started (250ms interval)");

    // 入力ポーリングタイマの初期化 (10ms 周期)
    const esp_timer_create_args_t input_poll_timer_args = {
        .callback = &input_poll_timer_callback,
        .name = "input_poll_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&input_poll_timer_args, &input_poll_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(input_poll_timer, INPUT_POLL_INTERVAL_MS * 1000));  // 10ms = 10000us
    ESP_LOGI(TAG, "Input poll timer started (%dms interval)", INPUT_POLL_INTERVAL_MS);

    // NimBLE 初期化
    ESP_ERROR_CHECK(nimble_port_init());

    // GATT サービス初期化
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // ATT MTU を設定 (最大 24 コマンドまで送信可能)
    // 必要 MTU = ATT ヘッダ (3) + コマンド個数 (1) + コマンド (24 * 4) = 100 バイト
    ble_att_set_preferred_mtu(REQUIRED_MTU);
    ESP_LOGI(TAG, "Set preferred MTU to %d bytes (max %d commands, payload %d bytes)",
             REQUIRED_MTU, MAX_USABLE_GPIO, PAYLOAD_SIZE);

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    // デバイス名設定
    ble_svc_gap_device_name_set("BLEIO");

    // BLE ホストタスク起動
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE initialization complete. Device name: BLEIO");
}
