/*
   LZ4 - Fast LZ compression algorithm
   Header File
   Copyright (C) Yann Collet 2011,

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/


#if defined (__cplusplus)
extern "C" {
#endif

//****************************
// Simple Functions
//****************************

int LZ4_compress (unsigned char* source, unsigned char* dest, int isize);
int LZ4_decode   (unsigned char* source, unsigned char* dest, int isize);

typedef void (*LZ4_stream_write)(void* ctx, unsigned char val);
typedef unsigned char (*LZ4_stream_read)(void* ctx, unsigned int offset_from_start);

int LZ4_stream_decode(
	void* ctx,
	LZ4_stream_write stream_write,
	LZ4_stream_read stream_read,
	unsigned char* source,
	int* isize,
	unsigned int dest_offset,
	unsigned int* osize);
	
/*
LZ4_compress :
	return : the number of bytes in compressed buffer dest
	note 1 : this function may sometimes read beyond source+isize, so provide some extra buffer room for that.
	note 2 : this simple function explicitly allocate/deallocate memory **at each call**

LZ4_decode :
	return : the number of bytes in decoded buffer dest
	note : this function may write up to 3 bytes more than decoded data length, so provide some extra buffer room for that.
*/


//****************************
// More Complex Functions
//****************************
int LZ4_compressCtx(void** ctx, unsigned char* source,  unsigned char* dest, int isize);

/*
LZ4_compressCtx :
	This function allows to handle explicitly the CTX memory structure.
	It avoids allocating/deallocating this memory at each call, for better performance.

	On first call : provide a *ctx=NULL; It will be automatically allocated.
	On next calls : reuse the same ctx pointer.
	Use different pointers for different threads when doing multi-threading.
*/

#if defined (__cplusplus)
}
#endif

