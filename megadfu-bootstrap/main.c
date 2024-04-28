#include "bootloader_util.h"

#if !defined(MEGADFU_START)
#error MEGADFU_START must be defined!
#endif

int main(void) {
	// Just forward on to MegaDFU, which is located at an offset that won't be obliterated by the incoming SoftDevice update
	bootloader_util_app_start(MEGADFU_START);

	return 0;
}
