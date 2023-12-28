/*
 *  ButtonManager.cpp
 *
 *  Date: 26 Dec 2023
 *  Author: Rubén Garrido
 *  Mail: rgarrido.rbn@gmail.com
 *
 */

#include "button/ButtonManager.h"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
    // Semaphore for avoinding race conditions on Button array
    static xSemaphoreHandle mButtonArraySem = NULL;
}

static bool mRunning{false};
static std::shared_ptr<Button> mButton[MAX_BUTTON_ALLOWED] = {0};

void checkButtons(void *pvParameters); // Main task

ButtonManager::ButtonManager()
{
    // create the task
    xTaskCreate(checkButtons, "checkButtons task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, this);

    // Init Mutex
    mButtonArraySem = xSemaphoreCreateBinary();
    if(mButtonArraySem == NULL)
    {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed!");
    }
    mRunning = true;
    // create the queue
}

int8_t ButtonManager::registerNewButton(std::shared_ptr<Button> newButton)
{
    if(nullptr != newButton)
    {
        // TODO: use a semaphore
        mButton[newButton->getId()] = newButton;
        return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
}

void checkButtons(void *pvParameters)
{
    ButtonManager *instance = (ButtonManager *)pvParameters;
    while(1)
    {
        while(mRunning)
        {
            for(int i = 0; i <= MAX_BUTTON_ALLOWED; i++)
            {
                // TODO: use semaphore
                if(mButton[i]->getValue())
                {
                    // Send id to the queue, that means that button is pressed
                }
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}