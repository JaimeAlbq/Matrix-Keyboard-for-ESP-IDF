/**
 * @file keypad_espidf.h
 * @author Jaime Albuquerque (jaime.albq@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef _KEYPAD_ESPIDF_H_
#define _KEYPAD_ESPIDF_H_

/* Libraries */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

/* Default values */
#define KEYPAD_DEFAULT_NUM_LAYOUT_ROW   4UL
#define KEYPAD_DEFAULT_NUM_LAYOUT_COL   4UL
#define KEYPAD_DEFAULT_NUM_GPIO_OUT     4UL
#define KEYPAD_DEFAULT_NUM_GPIO_IN      4UL
#define KEYPAD_DEFAULT_FREQUENCY        50UL
#define KEYPAD_DEFAULT_LAYOUT                   \
        {                                       \
                {0x01, 0x02, 0x03, 0x0A},       \
                {0x04, 0x05, 0x06, 0x0B},       \
                {0x07, 0x08, 0x09, 0x0C},       \
                {0x0F, 0x00, 0x0E, 0x0D}        \
        }
#define KEYPAD_DEFAULT_GPIO_OUT {26, 27, 14, 12}
#define KEYPAD_DEFAULT_GPIO_IN  {35, 32, 33, 25}

#define ESP_INTR_FLAG_DEFAULT 0

#define KEYPAD_DEFAULT_SET                                       \
        {                                                        \
                .num_row_layout = KEYPAD_DEFAULT_NUM_LAYOUT_ROW, \
                .num_col_layout = KEYPAD_DEFAULT_NUM_LAYOUT_COL, \
                .layout = NULL,                                  \
                .num_gpio_out = KEYPAD_DEFAULT_NUM_GPIO_OUT,     \
                .num_gpio_in = KEYPAD_DEFAULT_NUM_GPIO_IN,       \
                .gpio_output = NULL,                             \
                .gpio_input = NULL,                              \
                .frequency = KEYPAD_DEFAULT_FREQUENCY,           \
                .last_output = -1,                               \
                .last_input = -1,                                \
                .queue = NULL,                                   \
        }

/* Structures */
/**
 * @brief Structure with all necessaries settings to work with one keypad.
 * 
 */
typedef struct keypad_settings {
        uint32_t      num_row_layout;   // Number of rows of the layout
        uint32_t      num_col_layout;   // Number of columns of the layout
        uint32_t      layout[4][4];     // Keypad layout in 2 dimensions
        uint32_t      num_gpio_out;     // Number of output GPIOs
        uint32_t      num_gpio_in;      // Number of input GPIOs
        gpio_num_t    *gpio_output;     // Pointer for the output GPIOs array
        gpio_num_t    *gpio_input;      // Pointer for the input GPIOs array
        uint32_t      frequency;        // Scan frequency in Hz
        uint32_t      last_output;      // Last order in output dimension
        uint32_t      last_input;       // Last order in input dimension
        QueueHandle_t queue;            // Queue to send this structure
} keypad_settings_t;

/**
 * @brief Structure with information of input pin.
 * 
 */
typedef struct keypad_input_info {
        uint32_t          id;
        keypad_settings_t *keypad;
} keypad_input_info_t;

uint64_t        pins_mask(const gpio_num_t *pins_array, const uint32_t num_pins);
void IRAM_ATTR  keypad_isr_handler(void *pvParameters);
void            keypad_sequency_task(void *pvParameters);
void            keypad_install(keypad_settings_t *keypad);

#endif // _KEYPAD_ESPIDF_H_