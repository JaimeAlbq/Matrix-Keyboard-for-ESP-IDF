/**
 * @file main.c
 * @author Jaime Albuquerque (jaime.albq@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Libraries
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "keypad_espidf.h"

// Keypad Pins
#define NUM_ROW 4
static gpio_num_t pins_row[] = {GPIO_NUM_35, GPIO_NUM_32, GPIO_NUM_25, GPIO_NUM_33};
#define NUM_COL 4
static gpio_num_t pins_col[] = {GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14, GPIO_NUM_12};

// Keypad Layout
static uint32_t kp_layout[NUM_ROW][NUM_COL] = {
    {0x07, 0x08, 0x09, 0x0F},
    {0x04, 0x05, 0x06, 0x0E},
    {0x01, 0x02, 0x03, 0x0D},
    {0x00, 0x0A, 0x0B, 0x0C}
};

// Keypad queue
static QueueHandle_t keypad_queue = NULL;

// Keypad task
static void task_get_key(void *pdParameters)
{
        keypad_settings_t *keypad = (keypad_settings_t *) pdParameters;

        while (true) {
                if (xQueueReceive(keypad->queue, keypad, portMAX_DELAY)) {
                        printf("Key pressed: 0x%02X\n\r", kp_layout[keypad->last_input][keypad->last_output]);
                }
        }
}

// Main
void app_main(void)
{
        static keypad_settings_t keypad_4x4 = {
                .num_row_layout = NUM_ROW,
                .num_col_layout = NUM_COL,
                .num_gpio_out = NUM_COL,
                .num_gpio_in = NUM_ROW,
                .gpio_output = pins_col,
                .gpio_input = pins_row,
                .frequency = 20,
                .last_output = -1,
                .last_input = -1,
                .queue = &keypad_queue,
        };

        keypad_install(&keypad_4x4);

        xTaskCreate(task_get_key, "task_get_key", 2048, (void *) &keypad_4x4, 10, NULL);
}