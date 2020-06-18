/* Linker script to configure memory regions. */
/* Based on SDK11 ble_app_hrs_gcc_nrf52.ld */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x26000, LENGTH = 0x4F000 /* 0x4F000 = 0x75000 (megadfu-finalize location) - 0x26000 (origin) */
  RAM (rwx) :  ORIGIN = 0x20002080, LENGTH = 0xdf80
}

SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;

SECTIONS
{
  .payloads :
  {
    . = ALIGN(4);
    *(.payload_finalize)
    . = ALIGN(4);
    *(.payload_descriptor)
    . = ALIGN(4);
    *(.payload_softdevice)
    . = ALIGN(4);
    *(.payload_bootloader)
    . = ALIGN(4);
    *(.payload_settings)
    . = ALIGN(4);
    *(.payload_application)
  } > FLASH
} INSERT AFTER .text

/* TODO: Place payload sections */

INCLUDE "nrf5x_common.ld"