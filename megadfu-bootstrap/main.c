#include "nrf.h"
#include <nrf52.h>

#include "bootloader_util.h"

#if !defined(MEGADFU_START)
#error MEGADFU_START must be defined!
#endif

static void prx_nvmc_page_erase(uint32_t address)
{
    // Enable erase.
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    __ISB();
    __DSB();

    // Erase the page
    NRF_NVMC->ERASEPAGE = address;
    wait_for_flash_ready();

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    __ISB();
    __DSB();
}



int main(void) {
  // S132 version 2.0 occupies 0000_0000-0001_b6b0
  // S132 version 7.2 occupies 0000_0000-0002_596c
  // SDK11 will place this code at 0001_c000 which will be overwritten when the softdevice is updated
  // Here we use the booloader to start the updater application at
  // MEGADFU_START (0x0002_6000) which is above S132 version 7.2
  if (NRF_UICR->APPROTECT != 0xffffffff) {
	  prx_nvmc_page_erase((uint32_t)NRF_UICR);
	  NVIC_SystemReset();
  }
  bootloader_util_app_start(MEGADFU_START);

  return 0;
}
