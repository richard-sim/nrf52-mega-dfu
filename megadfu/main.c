#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_util_platform.h"
#include "nrf_wdt.h"
#include "bootloader_util.h"

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

void WDT_IRQHandler() {
    if (nrf_wdt_int_enable_check(NRF_WDT_INT_TIMEOUT_MASK) == true) {
        nrf_wdt_event_clear(NRF_WDT_EVENT_TIMEOUT);
    }
}

void start_etm(void) {
  const unsigned ETMBASE = 0xE0041000;
  // Allow access to device
  *(unsigned*)(ETMBASE+0xfb0) = 0xc5acce55;

  // Enter configuration mode (write twice to be sure we reached it)
  *(unsigned*)(ETMBASE) = (1<<10);
  *(unsigned*)(ETMBASE) = (1<<10);

  // Set busID 2
  *(unsigned*)(ETMBASE+0x200) = 2;

  // Set trigger event
  *(unsigned*)(ETMBASE+8) = 0x406f;

  // Set to always enable in ETM Trace Enable Event
  *(unsigned*)(ETMBASE+0x20) = 0x6f;

  // Trace and stall always enabled
  *(unsigned*)(ETMBASE+0x24) = 0x020000001;

  // Stall when < 8 byes free in fifo
  *(unsigned*)(ETMBASE+0x2c) = 8;

  // Enable trace
  *(unsigned*)(ETMBASE) = 0x0800 | (0 << 7) | (0 << 8);

  // Essential that this bit is only cleared after everything else is done
  *(unsigned*)(ETMBASE) &= ~(1<<10);
}

void trace_init(void) {
  // enableNRF52TRACE 4 3 3

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  NRF_CLOCK->TRACECONFIG |= (CLOCK_TRACECONFIG_TRACEMUX_Parallel << CLOCK_TRACECONFIG_TRACEMUX_Pos) | (CLOCK_TRACECONFIG_TRACEPORTSPEED_32MHz << CLOCK_TRACECONFIG_TRACEPORTSPEED_Pos);

  NRF_P0->PIN_CNF[18] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[20] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[14] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[15] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[16] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);

  // unload ITM
  ITM->LAR = 0xc5acce55;

  // Set port size (TPIU_CSPSR)
  TPI->CSPSR = 1 << 3;
  // Set pin protocol to Sync Trace Port (TPIU_SPPR)
  TPI->SPPR = 0;

  TPI->FFCR = 0x102;

  ITM->TCR &= ~(0x7f << 16);
  ITM->TCR |= (1 << 16);

  // https://developer.arm.com/documentation/ddi0403/d/Debug-Architecture/ARMv7-M-Debug/The-Data-Watchpoint-and-Trace-unit/Control-register--DWT-CTRL?lang=en
  // dwtSamplePC 1
  DWT->CTRL |= (1 << 12);

  // dwtTraceException 1
  DWT->CTRL |= (1 << 16);
  
  // dwtSyncTap 1 (was 3)
  DWT->CTRL &= ~(3 << 10);
  DWT->CTRL |= (1 << 10);

  // dwtPostTap 0
  DWT->CTRL &= ~(1 << 9);
  
  // dwtPostInit 0 (was 1)
  CoreDebug->DCRDR |= 0x1000000;
  DWT->CTRL |= ~(0x0f << 5);
  //  DWT->CTRL &= ~(1 << 5);
  
  // dwtPostReset 0 (was 10)
  DWT->CTRL &= ~(0x0f << 1);
  //  DWT->CTRL |= (0x00 << 1);

  // dwtCycEna 1
  DWT->CTRL |= (1 << 0);

  // ITMTXEna 1
  ITM->TCR |= (1 << 3);

  // ITMEna 1
  ITM->TCR |= (1 << 0);



  start_etm();
}

int main(void) {
  trace_init();
      ITM_SendString(0, "\r\n" __DATE__ "\r\n" __TIME__ "\r\n");
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

	trace_init();
	char a = 'a';
	while(1) {
          nrf_wdt_reload_request_set(NRF_WDT_RR0);
	  ITM_Send8(0, a++);
	  if (a > 'z') a = 'a';
	}

	// Jump to the finalize application
	bootloader_util_app_start((uint32_t)pPayloadDescriptor->finalize_start);

	return 0;
}
