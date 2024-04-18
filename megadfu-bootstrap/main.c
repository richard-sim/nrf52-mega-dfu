#include "bootloader_util.h"

#include "nrf_gpio.h"

#if !defined(MEGADFU_START)
#error MEGADFU_START must be defined!
#endif

#define GPIO_OUTPUT_PIN_NUMBER 22

int main(void) {

  nrf_gpio_cfg_output(GPIO_OUTPUT_PIN_NUMBER);
  while(1) nrf_gpio_pin_toggle(GPIO_OUTPUT_PIN_NUMBER);
  nrf_gpio_pin_toggle(GPIO_OUTPUT_PIN_NUMBER);

	// Just forward on to MegaDFU, which is located at an offset that won't be obliterated by the incoming SoftDevice update
	bootloader_util_app_start(MEGADFU_START);

	return 0;
}
