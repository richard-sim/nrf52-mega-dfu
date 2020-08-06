/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_mbr.h"
#include "nrf_nvmc.h"
#include "nrf_wdt.h"
#include "nrf_drv_wdt.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_util_platform.h"

#include "bsp.h"
#include "pstorage_platform.h"

#include "LZ4.h"


extern const uint32_t _binary_bootloader_package_desc_start;
extern const uint32_t _binary_bootloader_package_desc_end;

extern const uint32_t _binary__build_bootloader_package_lz4_start;
extern const uint32_t _binary__build_bootloader_package_lz4_end;


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "DFU-Shotgun"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define BOOTLOADER_DFU_START			0xB1

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */


typedef enum {
	STATE_NONE,
	STATE_TRANSFER,
	STATE_TRANSFER_WAIT,
	STATE_UPDATE_BOOTLOADER,
	STATE_UNUSED_PAGE_SCAN,
	STATE_UNUSED_PAGE_SCAN_WAIT,
	STATE_ERASE_PAGE
} State_e;

static State_e sCurrentState = STATE_NONE;

static uint32_t* sTransferAddress = NULL;
static uint32_t* sTransferEnd = NULL;
static uint32_t sTransfersCompleted = 0;
static uint32_t sScanPage = 0;
static uint32_t sErasePageAddress = 0;


static void watchdog_feed() {
	if (!nrf_wdt_started()) {
		return;
	}
	
	for (int i=0; i<8; i++) {
		nrf_wdt_rr_register_t r = (nrf_wdt_rr_register_t)(NRF_WDT_RR0 + i);
		if (nrf_wdt_reload_request_is_enabled(r)) {
			nrf_wdt_reload_request_set(r);
		}
	}
}


static void uint32_to_uint8arr(const uint32_t ui, uint8_t* pUC) {
	pUC[0] = (uint8_t)((ui >> 24) & 0xFF);
	pUC[1] = (uint8_t)((ui >> 16) & 0xFF);
	pUC[2] = (uint8_t)((ui >>  8) & 0xFF);
	pUC[3] = (uint8_t)((ui >>  0) & 0xFF);
	pUC[4] = 0;
}

static void continue_transfer() {
	static uint8_t tx_data[BLE_NUS_MAX_DATA_LEN];

	sTransferAddress += sTransfersCompleted * 4;
	if (sTransferAddress == sTransferEnd) {
		sTransferAddress = sTransferEnd = NULL;
		sCurrentState = STATE_NONE;
		return;
	}

	unsigned int w0 = sTransferAddress[0];
	unsigned int w1 = sTransferAddress[1];
	unsigned int w2 = sTransferAddress[2];
	unsigned int w3 = sTransferAddress[3];

	//sprintf((char*)tx_data, "0x%08X\n", w);
	uint32_to_uint8arr(w0, tx_data+0);
	uint32_to_uint8arr(w1, tx_data+4);
	uint32_to_uint8arr(w2, tx_data+8);
	uint32_to_uint8arr(w3, tx_data+12);
	uint32_t err_code = ble_nus_string_send(&m_nus, tx_data, 16);
	if (err_code == NRF_ERROR_INVALID_STATE) {
		// Either disconnected or the RX hasn't enabled notifications
		return;
	}
	APP_ERROR_CHECK(err_code);
	
	sCurrentState = STATE_TRANSFER_WAIT;
	sTransfersCompleted = 0;
}

static uint32_t isPageUsed(uint32_t page) {
	const uint32_t* start = (uint32_t*)(page * 0x1000);
	const uint32_t* end = (uint32_t*)((page + 1) * 0x1000);
	for (const uint32_t* pAddr=start; pAddr!=end; pAddr++) {
		if (*pAddr != 0xFFFFFFFF) {
			return 1;
		}
	}

	return 0;
}

static uint32_t usedPages(uint32_t startPage) {
	uint32_t res = 0;
	for (uint32_t i=0; i<32; i++) {
		uint32_t page = startPage + i;
		uint32_t inUse = isPageUsed(page);
		
		res |= inUse << i;
	}
	
	return res;
}

static void continue_page_scan() {
	static uint8_t tx_data[BLE_NUS_MAX_DATA_LEN];

	sScanPage += sTransfersCompleted * 4 * 32;	// Each event contains 4x 32b fields (1 page = 1 bit)
	if (sScanPage >= 128) {
		sScanPage = 0;
		sCurrentState = STATE_NONE;
		return;
	}

	unsigned int pageState0 = usedPages(sScanPage);		// Pages [sScanPage..sScanPage+32)
	unsigned int pageState1 = usedPages(sScanPage+32);	// Pages [sScanPage+32..sScanPage+64)
	unsigned int pageState2 = usedPages(sScanPage+64);	// Pages [sScanPage+64..sScanPage+96)
	unsigned int pageState3 = usedPages(sScanPage+96);	// Pages [sScanPage+96..sScanPage+128)

	//sprintf((char*)tx_data, "0x%08X\n", pageState);
	uint32_to_uint8arr(pageState0, tx_data+0);
	uint32_to_uint8arr(pageState1, tx_data+4);
	uint32_to_uint8arr(pageState2, tx_data+8);
	uint32_to_uint8arr(pageState3, tx_data+12);
	uint32_t err_code = ble_nus_string_send(&m_nus, tx_data, 16);
	if (err_code == NRF_ERROR_INVALID_STATE) {
		// Either disconnected or the RX hasn't enabled notifications
		return;
	}
	APP_ERROR_CHECK(err_code);
	
	sCurrentState = STATE_UNUSED_PAGE_SCAN_WAIT;
}

typedef enum
{
	STREAM_OK,
	STREAM_ERROR_READ_PAST_END,
	STREAM_ERROR_DATA_REMAINING
} StreamDecompress_e;

typedef struct
{
	unsigned int* pStart;
	unsigned int* pCurr;

	unsigned char write_buf[3];
	unsigned char write_buf_size;
} StreamContext_t;

void stream_write(void* ctx, unsigned char val)
{
	StreamContext_t* p_stream_context = (StreamContext_t*)ctx;

	if (p_stream_context->write_buf_size < sizeof(p_stream_context->write_buf))
	{
		p_stream_context->write_buf[p_stream_context->write_buf_size++] = val;
		return;
	}

	unsigned char* pWB = p_stream_context->write_buf;
	uint32_t word_val = (val << 24) | (pWB[2] << 16) | (pWB[1] << 8) | pWB[0];

	nrf_nvmc_write_word((uint32_t)p_stream_context->pCurr, word_val);
	p_stream_context->pCurr++;

	p_stream_context->write_buf_size = 0;
}

unsigned char stream_read(void* ctx, unsigned int offset_from_start)
{
	StreamContext_t* p_stream_context = (StreamContext_t*)ctx;

	unsigned char val;

	uint32_t word = (offset_from_start & ~(sizeof(uint32_t) - 1)) / sizeof(uint32_t);
	uint32_t word_byte = offset_from_start & (sizeof(uint32_t) - 1);
	if ((int)word == (p_stream_context->pCurr - p_stream_context->pStart))
	{
		// Still in the write buffer
		val = p_stream_context->write_buf[word_byte];
	}
	else
	{
		uint32_t word_val = p_stream_context->pStart[word];
		val = (unsigned char)((word_val >> (word_byte * 8)) & 0xFF);
	}

	return val;
}

StreamDecompress_e stream_decompress(
	unsigned char* compressedData, int compressedSize,
	unsigned char* decompressedData, unsigned int* decompressedSize)
{
	StreamContext_t stream_context;
	memset(&stream_context, 0, sizeof(stream_context));
	stream_context.pStart = (unsigned int*)decompressedData;
	stream_context.pCurr = (unsigned int*)decompressedData;
	
	*decompressedSize = 0;

	unsigned char* pStart = compressedData;
	unsigned char* pEnd = compressedData + compressedSize;
	int compressed_size_remaining = compressedSize;
	while (pStart < pEnd) {
		unsigned int decompressed_chunk_size = 0;
		LZ4_stream_decode(
			&stream_context, stream_write, stream_read,
			pStart, &compressed_size_remaining,
			*decompressedSize, &decompressed_chunk_size);
		*decompressedSize += decompressed_chunk_size;

		pStart = compressedData + (compressedSize - compressed_size_remaining);
		
		watchdog_feed();
	}

	if (stream_context.write_buf_size != 0)
	{
		return STREAM_ERROR_DATA_REMAINING;
	}

	if (pStart != pEnd)
	{
		return STREAM_ERROR_READ_PAST_END;
	}

	return STREAM_OK;
}

static inline void wait_for_flash_ready(void)
{
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {;}
}

// The I/D cache flushes in this at the least seem to help the debugger update its memory views
static void sdk153_nrf_nvmc_write_words(uint32_t address, const uint32_t * src, uint32_t num_words)
{
    uint32_t i;

    // Enable write.
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    __ISB();
    __DSB();

    for (i = 0; i < num_words; i++)
    {
        ((uint32_t*)address)[i] = src[i];
        wait_for_flash_ready();
		
		if ((i % 128) == 0) {
			watchdog_feed();
		}
    }

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    __ISB();
    __DSB();
}

static void update_bootloader() {
	uint32_t bl_address = NRF_UICR->NRFFW[0];
	uint32_t mbr_settings = NRF_UICR->NRFFW[1];
	uint32_t currBootloaderSize = mbr_settings - bl_address;

	// Find the matching bootloader from the descriptor
	uint32_t newBootloaderOffset = 0;
	uint32_t* pDescStart = (uint32_t*)&_binary_bootloader_package_desc_start;
	uint32_t* pDescEnd = (uint32_t*)&_binary_bootloader_package_desc_end;
	uint32_t* pDesc = pDescStart;
	for (; pDesc!=pDescEnd; pDesc++) {
		uint32_t blSize = *pDesc;

		if (blSize == currBootloaderSize) {
			break;
		}

		newBootloaderOffset += blSize;
	}
	if (pDesc == pDescEnd) {
		// No matching bootloader found :(
		return;
	}
	
	// Disable the SoftDevice as we are going to be doing FLASH operations that don't play nicely with BLE (i.e. will HardFault)
	sd_softdevice_disable();
	
	watchdog_feed();
	
	//
	unsigned char* pCompressedStart = (unsigned char*)&_binary__build_bootloader_package_lz4_start;
	unsigned char* pCompressedEnd = (unsigned char*)&_binary__build_bootloader_package_lz4_end;
	unsigned int compressedSize = (uint32_t)(pCompressedEnd - pCompressedStart);
	// Use the empty space between the end of this application and the pstorage start to decompress
	// the bootloader package into. The package is placed at the end of the application by the
	// linker script.
	uint32_t decompressedDataAddress = ((unsigned int)pCompressedEnd + (0x1000ul-1ul)) & ~(0x1000ul-1ul);
	uint32_t pstorageStartAddress = PSTORAGE_DATA_START_ADDR;
	
	// Erase the temporary-storage pages
	for (uint32_t eraseAddress=decompressedDataAddress; eraseAddress<pstorageStartAddress; eraseAddress+=0x1000) {
		nrf_nvmc_page_erase(eraseAddress);
		
		watchdog_feed();
	}
	
	// Decompress the bootloaders (all variants)
	unsigned int decompressedSize = 0;
	unsigned char* pDecompressedData = (unsigned char*)decompressedDataAddress;
	StreamDecompress_e stream_result = stream_decompress(
		pCompressedStart, compressedSize, 
		pDecompressedData, &decompressedSize);
	if (stream_result != STREAM_OK) {
		return;
	}

	// Erase the bootloader so we can copy the new one in its place
	for (uint32_t eraseAddress=bl_address; eraseAddress<mbr_settings; eraseAddress+=0x1000) {
		nrf_nvmc_page_erase(eraseAddress);
		
		watchdog_feed();
	}
	// Copy the appropriate bootloader into place
	sdk153_nrf_nvmc_write_words(bl_address, (uint32_t*)(pDecompressedData + newBootloaderOffset), currBootloaderSize / sizeof(uint32_t));
	
	// Cleanup the temporary pages
	//for (uint32_t eraseAddress=decompressedDataAddress; eraseAddress<pstorageStartAddress; eraseAddress+=0x1000) {
	//	nrf_nvmc_page_erase(eraseAddress);
	//}
	
	watchdog_feed();
	
	// Erase the bootloader settings page to ensure the application isn't re-executed
	nrf_nvmc_page_erase(0x7F000);
	
	watchdog_feed();

	// Reset to start the new bootloader and enter DFU mode
	NRF_POWER->GPREGRET = BOOTLOADER_DFU_START;
	NVIC_SystemReset();
}

static void stateMachineUpdate() {
	// Force exit any state if not connected
	if ((m_conn_handle == BLE_CONN_HANDLE_INVALID) &&
		(sCurrentState != STATE_NONE)) {
		sCurrentState = STATE_NONE;
	}
	
	switch (sCurrentState) {
	case STATE_NONE:
		break;
	
	case STATE_TRANSFER:
		continue_transfer();
		break;
	case STATE_TRANSFER_WAIT:
		break;
		
	case STATE_UPDATE_BOOTLOADER:
		update_bootloader();
		break;
	
	case STATE_UNUSED_PAGE_SCAN: {
		continue_page_scan();
		break;
	}
	case STATE_UNUSED_PAGE_SCAN_WAIT:
		break;
	
	case STATE_ERASE_PAGE:
		nrf_nvmc_page_erase(sErasePageAddress);
		sErasePageAddress = 0;
		sCurrentState = STATE_NONE;
		break;
	}
}


static void handle_command(uint8_t* pCommand, uint32_t len) {
	static uint8_t tx_data[BLE_NUS_MAX_DATA_LEN];
    uint32_t       err_code;
	
	if (len > BLE_NUS_MAX_DATA_LEN) {
		return;
	}

	tx_data[0] = '\0';
	if (pCommand[0] == 'd') {
		// Reset for DFU mode
		//NRF_POWER->GPREGRET = BOOTLOADER_DFU_START;
		//NVIC_SystemReset();
		//sd_power_gpregret_clr(0, 0xffffffff);
		watchdog_feed();
		sd_power_gpregret_set(BOOTLOADER_DFU_START);
		sd_nvic_SystemReset();
	} else if (pCommand[0] == 'r') {
		// Read basic info (Bootloader address, MBR settings page address, if the bootloader matches the packaged one or not, etc)
		unsigned int bl_address = NRF_UICR->NRFFW[0];
		unsigned int mbr_settings = NRF_UICR->NRFFW[1];
		sprintf((char*)tx_data, "0x%05X 0x%05X\n", bl_address, mbr_settings);

		err_code = ble_nus_string_send(&m_nus, tx_data, strlen((char*)tx_data));
		if (err_code != NRF_ERROR_INVALID_STATE) {
			APP_ERROR_CHECK(err_code);
		}
	} else if (pCommand[0] == 's') {
		// Get SoftDevice information (from SD Info struct)
		unsigned int sd_size = SD_SIZE_GET(MBR_SIZE);
		unsigned int sd_fwid = SD_FWID_GET(MBR_SIZE);
		sprintf((char*)tx_data, "0x%05X 0x%04X\n", sd_size, sd_fwid);

		err_code = ble_nus_string_send(&m_nus, tx_data, strlen((char*)tx_data));
		if (err_code != NRF_ERROR_INVALID_STATE) {
			APP_ERROR_CHECK(err_code);
		}
	} else if (pCommand[0] == 'u') {
		// Dump full UICR
		sCurrentState = STATE_TRANSFER;
		uint32_t* pStart = (uint32_t*)(NRF_UICR_BASE);
		uint32_t* pEnd = (uint32_t*)(NRF_UICR_BASE + 0x1000);
		sTransferAddress = pStart;
		sTransferEnd = pEnd;
		sTransfersCompleted = 0;
	} else if (pCommand[0] == 'f') {
		// Dump full FICR
		sCurrentState = STATE_TRANSFER;
		uint32_t* pStart = (uint32_t*)(NRF_FICR_BASE);
		uint32_t* pEnd = (uint32_t*)(NRF_FICR_BASE + 0x1000);
		sTransferAddress = pStart;
		sTransferEnd = pEnd;
		sTransfersCompleted = 0;
	} else if (pCommand[0] == 'b') {
		// Update bootloader with the appropriate packaged one
		sCurrentState = STATE_UPDATE_BOOTLOADER;
	} else if (pCommand[0] == 'e') {
		// Erase the bootloader settings page and reset (will result in this application not executing)
		sCurrentState = STATE_ERASE_PAGE;
		sErasePageAddress = 0x7F000;
	} else if (pCommand[0] == '1') {
		// Dump the bootloader
		sCurrentState = STATE_TRANSFER;
		uint32_t bl_address = NRF_UICR->NRFFW[0];
		uint32_t mbr_settings = NRF_UICR->NRFFW[1];
		uint32_t* pStart = (uint32_t*)(bl_address);
		uint32_t* pEnd = (uint32_t*)(mbr_settings);
		sTransferAddress = pStart;
		sTransferEnd = pEnd;
		sTransfersCompleted = 0;
	} else if (pCommand[0] == '2') {
		// Dump the MBR settings page
		sCurrentState = STATE_TRANSFER;
		uint32_t mbr_settings = NRF_UICR->NRFFW[1];
		uint32_t* pStart = (uint32_t*)(mbr_settings);
		uint32_t* pEnd = (uint32_t*)(mbr_settings + 0x1000);
		sTransferAddress = pStart;
		sTransferEnd = pEnd;
		sTransfersCompleted = 0;
	} else if (pCommand[0] == '3') {
		// Dump the bootloader settings page
		sCurrentState = STATE_TRANSFER;
		uint32_t* pStart = (uint32_t*)(0x7F000);
		uint32_t* pEnd = (uint32_t*)(0x7F000 + 0x1000);
		sTransferAddress = pStart;
		sTransferEnd = pEnd;
		sTransfersCompleted = 0;
	} else if (pCommand[0] == 'p') {
		// Scan for used vs unused flash pages
		sCurrentState = STATE_UNUSED_PAGE_SCAN;
		sScanPage = 0;
		sTransfersCompleted = 0;
	} else if (pCommand[0] == 'w') {
		// Dump the watchdog state
		uint32_to_uint8arr((uint32_t)nrf_wdt_started(), tx_data+0);
		uint32_to_uint8arr(NRF_WDT->CONFIG, tx_data+4);
		uint32_to_uint8arr(NRF_WDT->RREN, tx_data+8);
		uint32_to_uint8arr(NRF_WDT->CRV, tx_data+12);

		uint32_t err_code = ble_nus_string_send(&m_nus, tx_data, 16);
		if (err_code != NRF_ERROR_INVALID_STATE) {
			APP_ERROR_CHECK(err_code);
		}
	}
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	handle_command(p_data, length);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE: {
			//uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
			//APP_ERROR_CHECK(err_code);
            break;
		}
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
#if 0
        // Upon terminated advertising (time-out), the next advertising mode is started.
        case BLE_GAP_EVT_ADV_SET_TERMINATED:
			err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
			APP_ERROR_CHECK(err_code);
            break;
#endif
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
		
		case BLE_EVT_TX_COMPLETE: {
			uint8_t evt_count = p_ble_evt->evt.common_evt.params.tx_complete.count;
			if (sCurrentState == STATE_TRANSFER_WAIT) {
				sTransfersCompleted = evt_count;
				sCurrentState = STATE_TRANSFER;
			} else if (sCurrentState == STATE_UNUSED_PAGE_SCAN_WAIT) {
				sTransfersCompleted = evt_count;
				sCurrentState = STATE_UNUSED_PAGE_SCAN;
			}
			break;
		}

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;

    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	
	watchdog_feed();
    
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
	
	watchdog_feed();

    printf("\r\ndfu-shotgun!\r\n");
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    // Enter main loop.
    for (;;)
    {
		watchdog_feed();
		
		stateMachineUpdate();
		
		watchdog_feed();

		if ((sCurrentState == STATE_NONE) ||
			(sCurrentState == STATE_TRANSFER_WAIT) ||
			(sCurrentState == STATE_UNUSED_PAGE_SCAN_WAIT)) {
			power_manage();
		}
    }
}


/** 
 * @}
 */
