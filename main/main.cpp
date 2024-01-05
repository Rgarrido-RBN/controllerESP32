/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "bluetooth/BluetoothESP32.h"
#include "button/ButtonManager.h"
#include "gpio/gpioESP32.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#if CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatts_api.h"
#endif
#include "esp_bt_device.h"
#include "esp_bt_main.h"

#include "bluetooth/esp_hid_gap.h"
#include "esp_hidd.h"

extern "C"
{
#include "gpio/gpioDefinition.h"
    void app_main(void);
}

static const char *TAG = "HID_DEV_DEMO";

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

void app_main(void)
{
    esp_err_t ret;
#if HID_DEV_MODE == HIDD_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID device or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    auto buttonPresetDownPin =
        std::make_shared<gpioESP32>(BUTTON_PRESET_DOWN_PIN, PAL_GPIO_INTR_POSEDGE, PAL_GPIO_MODE_INPUT, PULLDOWN_MODE);
    auto buttonTest = std::make_shared<Button>(buttonPresetDownPin, 1); // MODE 1
    auto buttonMan = std::make_shared<ButtonManager>();
    buttonMan->registerNewButton(buttonTest);

    auto bluetooth = std::make_shared<BluetoothESP32>(&ble_hid_config);

    bluetooth->init();
}
