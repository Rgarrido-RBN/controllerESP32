/*
 *  Button.h
 *
 *  Date: 26 Dec 2023
 *  Author: Rubén Garrido
 *  Mail: rgarrido.rbn@gmail.com
 *
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include "gpio/gpioESP32.h"
#include <memory>

class Button
{
  public:
    Button(std::shared_ptr<gpioESP32> pin, uint8_t mode);
    virtual ~Button() = 0;
    bool getValue();
    uint8_t getId();

  private:
    std::shared_ptr<gpioESP32> mPin;
    int mMode; // Auto release or not
    int mButtonId{0};
};
#endif /* BUTTON_H_ */
