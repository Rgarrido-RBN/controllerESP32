/*
 *  ButtonManager.cpp
 *
 *  Date: 26 Dec 2023
 *  Author: Rubén Garrido
 *  Mail: rgarrido.rbn@gmail.com
 *
 */

#include "button/ButtonManager.h"
#include "esp_log.h"

const char TAG[] = "ButtonManager";

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
    // Semaphore for avoinding race conditions on Button array
}
xSemaphoreHandle mButtonArraySem;
xQueueHandle mButtonEventQueue;

static bool mRunning{false};
static std::shared_ptr<Button> mButton[MAX_BUTTON_ALLOWED] = {0};
static uint64_t debounceTime = 0;

void checkButtons(void *pvParameters); // Main task

ButtonManager::ButtonManager()
{
    // create the task
    xTaskCreate(checkButtons, "checkButtons task", 2 * 1024, this, configMAX_PRIORITIES - 3, NULL);

    // Init Mutex
    mButtonArraySem = xSemaphoreCreateBinary();

    if(mButtonArraySem == NULL)
    {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed!");
    }
    mRunning = true;

    // create the queue
    mButtonEventQueue = xQueueCreate(MAX_BUTTON_ALLOWED, sizeof(uint8_t));
}

int8_t ButtonManager::registerNewButton(std::shared_ptr<Button> newButton)
{
    if(nullptr != newButton)
    {
        ESP_LOGI(TAG, "BEFORE SEM");
        if(xSemaphoreTake(mButtonArraySem, NULL) == pdTRUE)
        {
            ESP_LOGI(TAG, "AFTER SEM");
            mButton[newButton->getId()] = newButton;
            xSemaphoreGive(mButtonArraySem);
            return EXIT_SUCCESS;
        }
    }
    return EXIT_FAILURE;
}

void checkButtons(void *pvParameters)
{
    while(1)
    {
        while(mRunning)
        {
            for(int i = 0; i <= MAX_BUTTON_ALLOWED; i++)
            {
                if(xSemaphoreTake(mButtonArraySem, NULL) == pdTRUE)
                {
                    // //if(mButton[i]->getValue())
                    // {
                    //     vTaskDelay(5 / portTICK_PERIOD_MS);
                    //     if(mButton[i]->getValue())
                    //     {
                    //         // Send id to the queue, that means that button is pressed
                    //         uint8_t id = mButton[i]->getId();
                    //         xQueueSend(mButtonEventQueue, (void *)id, portMAX_DELAY);
                    //     }
                    // }
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
                xSemaphoreGive(mButtonArraySem);
            }
            vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}