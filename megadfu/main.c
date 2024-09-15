#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_wdt.h"
#include "bootloader_util.h"

#include "nrf_drv_uart.h"

#include "prx_nvmc.h"

#include <nrf52.h>
#include <nrf52_bitfields.h>

#include "itm_messages.h"


#define DEBUG 1

#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)


// these externs are all resolved when this application is linked with megadfu-finalise
extern const uint32_t _binary__build_obj_payload_descriptor_bin_start;
extern const uint32_t _binary__build_obj_payload_descriptor_bin_end;
extern const uint32_t _binary__build_obj_payload_finalize_bin_start;
extern const uint32_t _binary__build_obj_payload_finalize_bin_end;
extern const uint32_t _binary__build_obj_payload_softdevice_lz4_start;
extern const uint32_t _binary__build_obj_payload_softdevice_lz4_end;
extern const uint32_t _binary__build_obj_payload_bootloader_lz4_start;
extern const uint32_t _binary__build_obj_payload_bootloader_lz4_end;
extern const uint32_t _binary__build_obj_payload_settings_lz4_start;
extern const uint32_t _binary__build_obj_payload_settings_lz4_end;

typedef struct {
	unsigned char* finalize_start;
	unsigned char* sd_start;
	unsigned char* bl_start;
	unsigned char* settings_start;
	unsigned char* app_start;
} PayloadDescriptor_t;

static void printbyte(uint8_t val) {
  uint8_t buffer[] = "11";
  uint8_t upper = (val >> 4) & 0xf;
  uint8_t lower = (val >> 0) & 0xf;

  buffer[0] = (upper > 9) ? (upper-10 + 'A') : (upper + '0');
  buffer[1] = (lower > 9) ? (lower-10 + 'A') : (lower + '0');

  nrf_drv_uart_tx(buffer, sizeof(buffer)-1);
}

static void printhex(uint32_t val) {
  const uint8_t buffer1[] = "0x";
  nrf_drv_uart_tx(buffer1, sizeof(buffer1)-1);
  
  printbyte((val >> 24) & 0xff);
  printbyte((val >> 16) & 0xff);
  printbyte((val >>  8) & 0xff);
  printbyte((val >>  0) & 0xff);

  const uint8_t buffer2[] = "\r\n";
  nrf_drv_uart_tx(buffer2, sizeof(buffer2)-1);
}

void WDT_IRQHandler() {
    if (nrf_wdt_int_enable_check(NRF_WDT_INT_TIMEOUT_MASK) == true) {
        nrf_wdt_event_clear(NRF_WDT_EVENT_TIMEOUT);
    }
}

static void trace_init(void) {
  // enableNRF52TRACE 4 3 3
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  NRF_CLOCK->TRACECONFIG |= (CLOCK_TRACECONFIG_TRACEMUX_Parallel << CLOCK_TRACECONFIG_TRACEMUX_Pos) | (CLOCK_TRACECONFIG_TRACEPORTSPEED_4MHz << CLOCK_TRACECONFIG_TRACEPORTSPEED_Pos);

  NRF_P0->PIN_CNF[18] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[20] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[14] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[15] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[16] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

  ITM->LAR = 0xc5acce55;
  *(uint32_t*)(TPI_BASE+0x1000+0xfb0) = 0xc5acce55;
  *(uint32_t*)(TPI_BASE+0xfb0) = 0xc5acce55;

  // Set port size (TPIU_CSPSR)
  TPI->CSPSR = 1 << 3;
  // Set pin protocol to Sync Trace Port (TPIU_SPPR)
  TPI->SPPR = 0;

  TPI->FFCR = 0x102;

  ITM->LAR = 0xc5acce55;

  ITM->TCR &= ~(0x7f << 16);
  ITM->TCR |= (1 << 16);

  // dwtSamplePC 1
  DWT->CTRL |= (1 << 12);

  // dwtSyncTap 3
  DWT->CTRL |= (3 << 10);

  // dwtPostTap 0
  DWT->CTRL &= ~(1 << 9);
  
  // dwtPostInit 1
  CoreDebug->DCRDR |= 0x1000000;
  DWT->CTRL |= ~(0x0f << 5);
  DWT->CTRL &= ~(1 << 5);
  
  // dwtPostReset 10
  DWT->CTRL &= ~(0x0f << 1);
  DWT->CTRL |= (0x0a << 1);

  // dwtCycEna 1
  DWT->CTRL |= (1 << 0);

  // ITMTXEna 1
  ITM->LAR = 0xc5acce55;
  ITM->TCR |= (1 << 3);

  // ITMEna 1
  ITM->TCR |= (1 << 0);
}


int main(void) {
  trace_init();
      ITM_SendString(0, __DATE__);
      ITM_Send8(0,'\r');
      ITM_Send8(0,'\n');
      ITM_SendString(0, __TIME__);
      ITM_Send8(0,'\r');
      ITM_Send8(0,'\n');
	if (nrf_wdt_started()) {
		// WDT is already running; feed it
		for(uint32_t i = 0; i < NRF_WDT_CHANNEL_NUMBER; i++) {
			nrf_wdt_rr_register_t reload_reg = (nrf_wdt_rr_register_t)(NRF_WDT_RR0 + i);
			if (nrf_wdt_reload_request_is_enabled(reload_reg)) {
				nrf_wdt_reload_request_set(reload_reg);
			}
		}
	} else {
		// WDT isn't running yet, so start it. This is done so that we can rely on it
		// always running (we expect to come in here with it running, so this is only
		// to be safe)

		// nrf_drv_wdt_init(NULL, on_wdt_timeout);
		nrf_wdt_behaviour_set(NRF_WDT_BEHAVIOUR_RUN_SLEEP);
		nrf_wdt_reload_value_set(0xfffffffe);

		NVIC_SetPriority(WDT_IRQn, APP_IRQ_PRIORITY_HIGH);
		NVIC_ClearPendingIRQ(WDT_IRQn);
		NVIC_EnableIRQ(WDT_IRQn);

		// nrf_drv_wdt_channel_id wdt_channel;
		// nrf_drv_wdt_channel_alloc(&wdt_channel);
		nrf_wdt_reload_request_enable(NRF_WDT_RR0);

		// nrf_drv_wdt_enable();
		nrf_wdt_int_enable(NRF_WDT_INT_TIMEOUT_MASK);
		nrf_wdt_task_trigger(NRF_WDT_TASK_START);

		// nrf_drv_wdt_feed();
		nrf_wdt_reload_request_set(NRF_WDT_RR0);
	}
#if DEBUG
	nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
	nrf_drv_uart_init(&uart_config, NULL);
	{
	  const uint8_t data[] = STRINGIZE(__LINE__) "\r\n";
	  nrf_drv_uart_tx(data, sizeof(data)-1);
	}
#endif
	PayloadDescriptor_t* pPayloadDescriptor = (PayloadDescriptor_t*)&_binary__build_obj_payload_descriptor_bin_start;

	// Erase region and copy the megadfu-finalise to immediately before the bootloader (pstorage pages??)
	// Needs to be cleaned up by the real application on first-run before pstorage or BLE are initialized.
	for (uint32_t eraseAddress=(uint32_t)pPayloadDescriptor->finalize_start; eraseAddress<(uint32_t)pPayloadDescriptor->bl_start; eraseAddress+=0x1000) {
		prx_nvmc_page_erase(eraseAddress);
	}
	unsigned char* pFinalizeStart = (unsigned char*)&_binary__build_obj_payload_finalize_bin_start;
	unsigned char* pFinalizeEnd = (unsigned char*)&_binary__build_obj_payload_finalize_bin_end;
	unsigned int finalizeSize = (uint32_t)(pFinalizeEnd - pFinalizeStart);

	prx_nvmc_write_words((uint32_t)pPayloadDescriptor->finalize_start, (uint32_t*)pFinalizeStart, finalizeSize / sizeof(uint32_t));
#if DEBUG
	printhex((uint32_t)pPayloadDescriptor->finalize_start); // 0x00075000
	printhex((uint32_t) pFinalizeStart);                    // 0x0002713C (will vary based on code size)
	printhex(finalizeSize);                                 // 0x00000EFC (will vary based on code size)
	printhex((uint32_t)pPayloadDescriptor->bl_start);       // 0x00076000
#endif
	// Erase the UICR so that we can be sure that storing the payload addresses in UICR->Customer is safe
	// NOTE: This will obliterate the NRFFW[0], NRFFW[1], PSELRESET[0], PSELRESET[1], and NFCPINS values
	prx_nvmc_page_erase((uint32_t)NRF_UICR);

	// Restore the appropriate settings
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->NRFFW[0]), (uint32_t)pPayloadDescriptor->bl_start);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->NRFFW[1]), 		0x7E000ul);	// Always FLASH_SIZE-2*CODE_PAGE_SIZE
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->NFCPINS), 		0xFFFFFFFEul);	// NFC pins disabled
	
	// Store all payload addresses in UICR->Customer
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[24]), (uint32_t)&_binary__build_obj_payload_descriptor_bin_start);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[25]), (uint32_t)&_binary__build_obj_payload_descriptor_bin_end);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[26]), (uint32_t)&_binary__build_obj_payload_softdevice_lz4_start);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[27]), (uint32_t)&_binary__build_obj_payload_softdevice_lz4_end);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[28]), (uint32_t)&_binary__build_obj_payload_bootloader_lz4_start);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[29]), (uint32_t)&_binary__build_obj_payload_bootloader_lz4_end);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[30]), (uint32_t)&_binary__build_obj_payload_settings_lz4_start);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[31]), (uint32_t)&_binary__build_obj_payload_settings_lz4_end);

	{
	  const uint8_t data[] = STRINGIZE(__LINE__) "\r\n";
	  nrf_drv_uart_tx(data, sizeof(data)-1);
	}

	// Jump to the finalize application
	bootloader_util_app_start((uint32_t)pPayloadDescriptor->finalize_start);

	return 0;
}
