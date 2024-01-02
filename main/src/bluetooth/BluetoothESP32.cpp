/*
 *  BluetoothESP32.cpp
 *
 *  Date: 28 Dec 2023
 *  Author: Rubén Garrido
 *  Mail: rgarrido.rbn@gmail.com
 *
 */
#include "bluetooth/BluetoothESP32.h"
#include "bluetooth/esp_hid_gap.h"

extern "C"
{
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <cstring>
    extern xQueueHandle mButtonEventQueue;
    static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
}

#define HID_CC_IN_RPT_LEN 2
#define HID_RPT_ID_CC_IN 3 // Gamepad Control input report ID
#define HID_CC_RPT_BUTTON_BITS 0xF0
#define HID_CC_RPT_SET_BUTTON(s, x)                                                                                    \
    (s)[1] &= HID_CC_RPT_BUTTON_BITS;                                                                                  \
    (s)[1] |= (x)

static const char *TAG = "HID_DEV_BLE";

const unsigned char buttonBoxReportMap[] = {
    0x05, 0x01, // Usage Page (Generic Desktop Controls)
    0x09, 0x05, // Usage (Gamepad)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x01, // Report ID (1)
    0x09, 0x01, // Usage (Button 1)
    0x09, 0x02, // Usage (Button 2)
    0x09, 0x39, // Usage (Hat Switch)
    0x15, 0x01, // Logical Minimum (0)
    0x25, 0x08, // Logical Maximum (8)
    0x35, 0x00, // Physical Minimum (0)
    0x45, 0x08, // Physical Maximum (8)
    0x95, 0x01, // Report Count (1)
    0x75, 0x08, // Report Size (8)
    0x81, 0x02, // Input (Data, Variable, Absolute)
    0xC0,       // End Collection
};

static esp_hid_raw_report_map_t ble_report_maps[] = {{.data = buttonBoxReportMap, .len = sizeof(buttonBoxReportMap)}};

static esp_hid_device_config_t ble_hid_config = {.vendor_id = 0x16C0,
                                                 .product_id = 0x05DF,
                                                 .version = 0x0100,
                                                 .device_name = "Prudi button box",
                                                 .manufacturer_name = "RBN SIM",
                                                 .serial_number = "1234567890",
                                                 .report_maps = ble_report_maps,
                                                 .report_maps_len = 1};

void buttonBoxBluetoothTask(void *pvParameters);

typedef struct
{
    xTaskHandle task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_ble_hid_param = {0};

static bool mRunning;

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch(event)
    {
    case ESP_HIDD_START_EVENT: {
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT: {
        ESP_LOGI(TAG, "CONNECT");
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT: {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index,
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT: {
        ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT: {
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index,
                 esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT: {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index,
                 esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT: {
        ESP_LOGI(
            TAG, "DISCONNECT: %s",
            esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_STOP_EVENT: {
        ESP_LOGI(TAG, "STOP");
        break;
    }
    default:
        break;
    }
    return;
}

BluetoothESP32::BluetoothESP32(esp_hid_device_config_t *config) : mConfig(config)
{
    xTaskCreate(buttonBoxBluetoothTask, "buttonBoxBluetoothTask", 2 * 1024, NULL, configMAX_PRIORITIES - 3, NULL);
}

int8_t BluetoothESP32::init()
{

    esp_err_t ret;
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK(ret);

    ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, mConfig->device_name);
    ESP_ERROR_CHECK(ret);

    if((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK)
    {
        ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
        return -1;
    }
    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_ble_hid_param.hid_dev));
    mRunning = true;
    return 0;
}

int8_t BluetoothESP32::stop()
{
    return 0;
}

void buttonBoxBluetoothTask(void *pvParameters)
{
    uint8_t buttonID = 0;
    uint8_t bufferToSend[12] = {0};
    while(1)
    {
        while(mRunning)
        {
            if(mButtonEventQueue != NULL)
            {
                if(xQueueReceive(mButtonEventQueue, &buttonID, portMAX_DELAY))
                {
                    HID_CC_RPT_SET_BUTTON(bufferToSend, 1);
                    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 1, HID_RPT_ID_CC_IN, bufferToSend,
                                           HID_CC_IN_RPT_LEN);
                    memset(bufferToSend, 0, sizeof(bufferToSend));
                }
            }
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}