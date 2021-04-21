# Matrix Keyboard for ESP-IDF
Library for matrix keyboard using ESP-IDF framework

## How to use it?
It needed just 4 simple steps to use this library:
1. Copy the `keypad_espidf.h` and `keypad_espidf.c` to your project;
1. Set your keypad settings by using the `keypad_settings_t` structure;
1. Install it with `keypad_install(keypad_settings_t *)`;
1. Use the income information with the `.queue` in the settings and FreeRTOS Queue.

## How is it work?
Before start using your keyboard or even install it, a structure needs to be set with all configuration of your keypad.

* `keypad_settings_t`
  * `num_row_layout` = Number of rows of the layout
  * `num_col_layout` = Number of columns of the layout
  * `layout` = Keypad layout in 2 dimensions
  * `num_gpio_out` = Number of output GPIOs
  * `num_gpio_in` = Number of input GPIOs
  * `gpio_output` = Pointer for the output GPIOs array
  * `gpio_input` = Pointer for the input GPIOs array
  * `frequency` = Scan frequency in Hz
  * `last_output` = Last order in output dimension
  * `last_input` = Last order in input dimension
  * `queue` = Queue to send this structure

After that, the installation is needed. To do so, the functions `keypad_install(keypad_settings_t *)` is called and the keypad configuration passed to the library. Here are setted the output and input pins, create the keypad queue, create the sequency task to scan each output, install the ISR and set the interrupt handler for each input pin.

The `keypad_sequency_task(void *)` makes a sequency for each output pin, by switching every pin in high level and just the select one in low. During the task is working, the ISR handler is active to store the last input pin and than send it to the keypad queue.

## TODO
* [ ] Write a macro to set default setting to the commum [membrane keypad 4x4](https://potentiallabs.com/cart/image/cache/catalog/Arduino/4x4%20keypad-700x700-800x800.jpg)