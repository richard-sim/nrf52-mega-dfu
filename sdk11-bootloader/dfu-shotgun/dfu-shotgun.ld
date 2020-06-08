/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x1c000, LENGTH = 0x2e000
  RAM (rwx) :  ORIGIN = 0x20001FE8, LENGTH = 0xE018
/*  RAM (rwx) :  ORIGIN = 0x20002080, LENGTH = 0xdf80*/
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
    *(.payload_descriptor)
    . = ALIGN(4);
    *(.payload_package)
  } > FLASH
} INSERT AFTER .text;

INCLUDE "nrf5x_common.ld"