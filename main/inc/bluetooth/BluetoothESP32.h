/*
 *  BluetoothESP32.h
 *
 *  Date: 28 Dec 2023
 *  Author: Rubén Garrido
 *  Mail: rgarrido.rbn@gmail.com
 *
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

extern "C"
{
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_hidd.h"
}

#include <stdint.h>
#include <string>

class BluetoothESP32
{
  public:
    BluetoothESP32(esp_hid_device_config_t *config);
    virtual ~BluetoothESP32() = default;

    int8_t init();
    int8_t stop();

  private:
    esp_hid_device_config_t *mConfig;
};

#endif /* BLUETOOTH_H_ */