#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ESP32-GPIO";

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
#define CMD_SET_INPUT           0
#define CMD_SET_OUTPUT          1
#define CMD_SET_INPUT_PULLUP    2
#define CMD_WRITE_LOW           10
#define CMD_WRITE_HIGH          11

// グローバル変数
static uint8_t gpio_read_pin = 0;
static uint8_t gpio_read_state = 0;
static uint16_t conn_handle = 0;

// 関数の前方宣言
static void ble_app_advertise(void);

// GPIO 制御関数
static bool is_valid_gpio(uint8_t pin) {
    // 使用可能な GPIO ピン
    if (pin == 2 || (pin >= 4 && pin <= 5) || (pin >= 12 && pin <= 19) ||
        (pin >= 21 && pin <= 27) || (pin >= 32 && pin <= 36) || pin == 39) {
        return true;
    }
    return false;
}

static esp_err_t gpio_set_mode(uint8_t pin, uint8_t command) {
    if (!is_valid_gpio(pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    switch (command) {
        case CMD_SET_INPUT:
            io_conf.mode = GPIO_MODE_INPUT;
            ESP_LOGI(TAG, "Set GPIO%d to INPUT", pin);
            break;
        case CMD_SET_OUTPUT:
            io_conf.mode = GPIO_MODE_OUTPUT;
            ESP_LOGI(TAG, "Set GPIO%d to OUTPUT", pin);
            break;
        case CMD_SET_INPUT_PULLUP:
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            ESP_LOGI(TAG, "Set GPIO%d to INPUT_PULLUP", pin);
            break;
        default:
            ESP_LOGE(TAG, "Invalid mode command: %d", command);
            return ESP_ERR_INVALID_ARG;
    }

    return gpio_config(&io_conf);
}

static esp_err_t gpio_write_level(uint8_t pin, uint8_t command) {
    if (!is_valid_gpio(pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t level;
    switch (command) {
        case CMD_WRITE_LOW:
            level = 0;
            ESP_LOGI(TAG, "Write GPIO%d to LOW", pin);
            break;
        case CMD_WRITE_HIGH:
            level = 1;
            ESP_LOGI(TAG, "Write GPIO%d to HIGH", pin);
            break;
        default:
            ESP_LOGE(TAG, "Invalid write command: %d", command);
            return ESP_ERR_INVALID_ARG;
    }

    return gpio_set_level(pin, level);
}

static esp_err_t gpio_read_level(uint8_t pin, uint8_t *state) {
    if (!is_valid_gpio(pin)) {
        ESP_LOGE(TAG, "Invalid GPIO pin: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    *state = gpio_get_level(pin);
    ESP_LOGI(TAG, "Read GPIO%d: %d", pin, *state);
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

    if (len != 2) {
        ESP_LOGE(TAG, "Invalid write length: %d (expected 2)", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t data[2];
    os_mbuf_copydata(om, 0, 2, data);

    uint8_t pin = data[0];
    uint8_t command = data[1];

    ESP_LOGI(TAG, "Write characteristic: pin=%d, command=%d", pin, command);

    esp_err_t ret;
    if (command <= CMD_SET_INPUT_PULLUP) {
        ret = gpio_set_mode(pin, command);
    } else if (command == CMD_WRITE_LOW || command == CMD_WRITE_HIGH) {
        ret = gpio_write_level(pin, command);
    } else {
        ESP_LOGE(TAG, "Unknown command: %d", command);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    return (ret == ESP_OK) ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_chr_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // WRITE 操作: ピン番号を設定
        struct os_mbuf *om = ctxt->om;
        uint16_t len = OS_MBUF_PKTLEN(om);

        if (len != 1) {
            ESP_LOGE(TAG, "Invalid write length: %d (expected 1)", len);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        os_mbuf_copydata(om, 0, 1, &gpio_read_pin);
        ESP_LOGI(TAG, "Set read target pin: GPIO%d", gpio_read_pin);
        return 0;
    } else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // READ 操作: ピン番号と状態を返す
        // ピンの状態を読み取る
        if (gpio_read_level(gpio_read_pin, &gpio_read_state) != ESP_OK) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        uint8_t data[2] = {gpio_read_pin, gpio_read_state};
        ESP_LOGI(TAG, "Sending data: pin=%d, state=%d, size=%d", data[0], data[1], sizeof(data));
        int rc = os_mbuf_append(ctxt->om, data, sizeof(data));
        ESP_LOGI(TAG, "os_mbuf_append result: %d", rc);
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

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
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
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
    ESP_LOGI(TAG, "Starting ESP32 GPIO Control Service");

    // NVS 初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // NimBLE 初期化
    ESP_ERROR_CHECK(nimble_port_init());

    // GATT サービス初期化
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);

    // デバイス名設定
    ble_svc_gap_device_name_set("ESP32-GPIO");

    // BLE ホストタスク起動
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE initialization complete. Device name: ESP32-GPIO");
}
