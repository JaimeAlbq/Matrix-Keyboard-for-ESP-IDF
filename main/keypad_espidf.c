/**
 * @file keypad_espidf.c
 * @author Jaime Albuquerque (jaime.albq@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "keypad_espidf.h"

/**
 * @brief Return a number with all selected pins from an array
 *        as a bit mask
 * 
 * @param pins_array    Array of pins
 * @param num_pins      Number of pins
 * @return uint64_t 
 */
uint64_t pins_mask(const gpio_num_t *pins_array, const uint32_t num_pins)
{
    uint32_t buf32_0 = 0;
    uint32_t buf32_1 = 0;
    uint64_t result = 0;
    for (uint32_t i = 0; i < num_pins; i++) {
        // result |= 1 << pins_array[i];
        if (pins_array[i] >= 32)
            buf32_1 |= 1 << (pins_array[i] - 32);
        else
            buf32_0 |= 1 << pins_array[i];
        
    }

    result = ((uint64_t)buf32_1 << 32) | ((uint64_t)buf32_0 << 0);

    return result;
}

/**
 * @brief   This task task does a sequency of outputs, pulling
 *          the active pin down and all the others up.
 * 
 * @param pvParameters  A keypad_settings_t pointer, whitch store
 *                      all the settings of the keypad, also needed
 *                      to set the last output dimansion.
 */
void keypad_sequency_task(void *pvParameters)
{
    keypad_settings_t *keypad = (keypad_settings_t *) pvParameters; // Getting the keypad settings
    uint32_t dim_turn = 0;                                          // Started to count the sequency
    uint32_t time_delay = 1000 / keypad->frequency;                 // 1000ms / frequency
    keypad->last_output = -1;

    while (true) {
        vTaskDelay(time_delay / portTICK_RATE_MS);

        for (uint32_t i = 0; i < keypad->num_gpio_out; i++) {
            if (dim_turn == i) {
                keypad->last_output = i;
                gpio_set_level(keypad->gpio_output[i], 0);
            } else {
                gpio_set_level(keypad->gpio_output[i], 1);
            }

            for (uint32_t j = 0; j < keypad->num_gpio_in; j++) {
                if (!gpio_get_level(keypad->gpio_input[j])) {
                    keypad->last_input = j;
                    xQueueSendToBack(keypad->queue, keypad, portMAX_DELAY);
                }
                
            }
            
        }

        if (++dim_turn > keypad->num_gpio_out) dim_turn = 0;
    }
}

/**
 * @brief   Its set the output and input pins, create the keypad queue,
 *          create the sequency task to scan each output, install the
 *          ISR and set the interrupt handler for each input pin.
 * 
 * @param keypad Pointer to created keypad settings.
 */
void keypad_install(keypad_settings_t *keypad)
{
    // keypad_settings_t *keypad = (keypad_settings_t *) pvParameters;

    gpio_config_t *io_conf = malloc(sizeof(gpio_config_t));
    //disable interrupt
    io_conf->intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf->mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf->pin_bit_mask = pins_mask(keypad->gpio_output, keypad->num_gpio_out);
    //disable pull-down mode
    io_conf->pull_down_en = 0;
    //disable pull-up mode
    io_conf->pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(io_conf);

    //interrupt of rising edge
    // io_conf->intr_type = GPIO_INTR_NEGEDGE;
    io_conf->intr_type = GPIO_INTR_DISABLE;
    //set as input mode
    io_conf->mode = GPIO_MODE_INPUT;
    //bit mask of the pins, use GPIO4/5 here
    io_conf->pin_bit_mask = pins_mask(keypad->gpio_input, keypad->num_gpio_in);
    //disable pull-down mode
    io_conf->pull_down_en = 0;
    //enable pull-up mode
    io_conf->pull_up_en = 0; // Bad internal pull-up
    //configure GPIO with the given settings
    gpio_config(io_conf);

    free(io_conf);

    //create a queue to handle gpio event from isr
    keypad->queue = xQueueCreate(10, sizeof(keypad_settings_t));
    //start gpio task
    xTaskCreate(keypad_sequency_task, "keypad_sequency_task", 2048, (void *) keypad, 10, NULL);
}
