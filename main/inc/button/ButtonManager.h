/*
 *  ButtonManager.h
 *
 *  Date: 26 Dec 2023
 *  Author: Rubén Garrido
 *  Mail: rgarrido.rbn@gmail.com
 *
 */

#include "button/Button.h"

#define MAX_BUTTON_ALLOWED 12

class ButtonManager
{
  public:
    ButtonManager();
    virtual ~ButtonManager() = default;
    int8_t registerNewButton(std::shared_ptr<Button> newButton);
};