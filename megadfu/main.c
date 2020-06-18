#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "bootloader_util.h"

#include "prx_nvmc.h"


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
// extern const uint32_t _binary__build_obj_payload_application_lz4_start;
// extern const uint32_t _binary__build_obj_payload_application_lz4_end;


typedef struct {
	unsigned char* finalize_start;
	unsigned char* sd_start;
	unsigned char* bl_start;
	unsigned char* settings_start;
	unsigned char* app_start;
} PayloadDescriptor_t;


//static volatile uint32_t sMegaDFUActivate = 0;


int main(void) {
//	while (sMegaDFUActivate == 0) {
//		// Wait
//	}
	
	PayloadDescriptor_t* pPayloadDescriptor = (PayloadDescriptor_t*)&_binary__build_obj_payload_descriptor_bin_start;

	// Copy the finalizer application to immediately before the bootloader (pstorage pages?? Needs to be cleaned up by the real application on first-run before pstorage or BLE are initialized!)
	for (uint32_t eraseAddress=(uint32_t)pPayloadDescriptor->finalize_start; eraseAddress<(uint32_t)pPayloadDescriptor->bl_start; eraseAddress+=0x1000) {
		prx_nvmc_page_erase(eraseAddress);
	}
	unsigned char* pFinalizeStart = (unsigned char*)&_binary__build_obj_payload_finalize_bin_start;
	unsigned char* pFinalizeEnd = (unsigned char*)&_binary__build_obj_payload_finalize_bin_end;
	unsigned int finalizeSize = (uint32_t)(pFinalizeEnd - pFinalizeStart);
	prx_nvmc_write_words((uint32_t)pPayloadDescriptor->finalize_start, (uint32_t*)pFinalizeStart, finalizeSize / sizeof(uint32_t));
	
	// Erase the UICR so that we can be sure that storing the payload addresses in UICR->Customer is safe
	// NOTE: This will obliterate the NRFFW[0], NRFFW[1], PSELRESET[0], PSELRESET[1], APPROTECT, and NFCPINS values
	prx_nvmc_page_erase((uint32_t)NRF_UICR);
	// Restore the appropriate settings
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->NRFFW[0]), (uint32_t)pPayloadDescriptor->bl_start);
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->NRFFW[1]), 		0x7E000ul);		// Always FLASH_SIZE-2*CODE_PAGE_SIZE
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->PSELRESET[0]), 	0x15ul);		// Enabled on Pin 21
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->PSELRESET[1]), 	0x15ul);		// copy of [0]
	prx_nvmc_write_word((uint32_t)&(NRF_UICR->APPROTECT), 		0xFFFFFF00ul);	// APPROTECT enabled
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
	// prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[30]), (uint32_t)&_binary__build_obj_payload_application_lz4_start);
	// prx_nvmc_write_word((uint32_t)&(NRF_UICR->CUSTOMER[31]), (uint32_t)&_binary__build_obj_payload_application_lz4_end);
	
	// Jump to the finalize application
	bootloader_util_app_start((uint32_t)pPayloadDescriptor->finalize_start);

	return 0;
}
