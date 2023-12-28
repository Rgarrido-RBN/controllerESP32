/*
 * gpioESP32.h
 *
 *  Created on: Nov 12, 2021
 *      Author: rbn
 */

#ifndef MAIN_INC_GPIO_GPIOESP32_H_
#define MAIN_INC_GPIO_GPIOESP32_H_

class gpioESP32
{
  public:
    gpioESP32(int pinName, int interruptMode, int pinMode, int pullMode);
    virtual ~gpioESP32();

    void enableInterruptPin(int pin);
    bool getValue();
    void setValue(int valueToSet);
    bool togglePin();
    int getPin();

  private:
    int mPinName;
    int mInterruptMode;
    int mPinMode;
    int mPullMode;
};

#endif /* MAIN_INC_GPIO_GPIOESP32_H_ */
