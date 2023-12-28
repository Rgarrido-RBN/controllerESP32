/*
 *  Button.cpp
 *
 *  Date: 26 Dec 2023
 *  Author: Rubén Garrido
 *  Mail: rgarrido.rbn@gmail.com
 *
 */

#include "button/Button.h"

extern "C"
{
#include "freertos/FreeRTOS.h"
}

static uint8_t mPintIdCounter{0};

Button::Button(std::shared_ptr<gpioESP32> pin, uint8_t mode) : mMode(mode), mPin(pin), mButtonId(mPintIdCounter++)
{
}

bool Button::getValue()
{
    return mPin->getValue();
}

uint8_t Button::getId()
{
    return mButtonId;
}