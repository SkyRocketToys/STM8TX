/*-----------------------------------------------------------------
 printfl.c - source file for reduced version of printf

 Copyright (C) 1999, Sandeep Dutta . sandeep.dutta@usa.net
 2001060401: Improved by was@icb.snz.chel.su

 This library is free software; you can redistribute it and/or modify it
 under the terms of the GNU General Public License as published by the
 Free Software Foundation; either version 2.1, or (at your option) any
 later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this library; see the file COPYING. If not, write to the
 Free Software Foundation, 51 Franklin Street, Fifth Floor, Boston,
 MA 02110-1301, USA.

 As a special exception, if you link this library with other files,
 some of which are compiled with SDCC, to produce an executable,
 this library does not by itself cause the resulting executable to
 be covered by the GNU General Public License. This exception does
 not however invalidate any other reasons why the executable file
 might be covered by the GNU General Public License.
 -------------------------------------------------------------------------*/

/* following formats are supported :-
 format     output type       argument-type
 %d        decimal             int
 %ld       decimal             long
 %hd       decimal             char
 %u        decimal             unsigned int
 %lu       decimal             unsigned long
 %hu       decimal             unsigned char
 %x        hexadecimal         int
 %lx       hexadecimal         long
 %hx       hexadecimal         char
 %o        octal               int
 %lo       octal               long
 %ho       octal               char
 %c        character           char
 %s        character           generic pointer
 */

#include "config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include "uart.h"

static char radix;
static bool long_flag = 0;
static bool string_flag = 0;
static bool char_flag = 0;
static bool unsigned_flag = 0;
static char *str;
static long val;

static void
output_char(register char c)
{
    uart2_putchar(c);
}

static char *ultoa_invert(uint32_t val, char *s, uint8_t base)
{
    uint32_t xval;
    if (base == 8) {
        do {
            *s++ = '0' + (val & 0x7);
            val >>= 3;
        } while(val);
        return s;
    }

    if (base == 16) {
        do {
            uint8_t digit = '0' + (val & 0xf);
            if (digit > '0' + 9)
                digit += ('A' - '0' - 10);
            *s++ = digit;
            val >>= 4;
        } while(val);
        return s;
    }

    // Every base which in not hex and not oct is considered decimal.

    // 33 bits would have been enough.
    xval = val;
    do {
        uint8_t saved = xval;
        xval &= ~1;
        xval += 2;
        xval += xval >> 1;		// *1.5
        xval += xval >> 4;		// *1.0625
        xval += xval >> 8;		// *1.00390625
        xval += xval >> 16;		// *1.000015259
        //xval += xval >> 32;		// it all amounts to *1.6
        xval >>= 4;				// /16 ... so *1.6/16 is /10, fraction truncated.
        *s++ = '0' + saved - 10 * (uint8_t)xval;
    } while (xval);
    return s;
}

static char *ltoa_invert(int32_t val, char *s, uint8_t base)
{
    if (val >= 0) {
        return ultoa_invert(val, s, base);
    }
    s = ultoa_invert(-val, s, base);
    *s++ = '-';
    return s;
}

void vprintfl(const char * fmt, va_list ap)
{
	for (; *fmt; fmt++) {
		if (*fmt == '%') {
			long_flag = string_flag = char_flag = unsigned_flag = 0;
			fmt++;
			switch (*fmt) {
			case 'l':
				long_flag = 1;
				fmt++;
				break;
			case 'h':
				char_flag = 1;
				fmt++;
			}

			switch (*fmt) {
			case 's':
				string_flag = 1;
				break;
			case 'd':
				radix = 10;
				break;
			case 'u':
				radix = 10;
				unsigned_flag = 1;
				break;
			case 'x':
				radix = 16;
				unsigned_flag = 1;
				break;
			case 'c':
				radix = 0;
				break;
			case 'o':
				radix = 8;
				unsigned_flag = 1;
				break;
			}

			if (string_flag) {
				str = va_arg(ap, char *);
				while (*str)
					output_char(*str++);
				continue;
			}

			if (unsigned_flag) {
				if (long_flag) {
					val = va_arg(ap,unsigned long);
				} else if (char_flag) {
					val = va_arg(ap,unsigned char);
				} else {
					val = va_arg(ap,unsigned int);
				}
			} else {
				if (long_flag) {
					val = va_arg(ap,long);
				} else if (char_flag) {
					val = va_arg(ap,char);
				} else {
					val = va_arg(ap,int);
				}
			}

			if (radix) {
				static char buffer[12]; /* 37777777777(oct) */
				char *stri;

				if (unsigned_flag) {
                                    stri = ultoa_invert(val, buffer, radix);
				} else {
                                    stri = ltoa_invert(val, buffer, radix);
				}
				while (stri > &buffer[0]) {
                                    output_char(*(--stri));
				}
			} else {
				output_char((char) val);
			}

		} else {
			output_char(*fmt);
		}
	}
}

void printf(const char *fmt, ...)
{
	va_list ap;

	va_start(ap,fmt);
	vprintfl(fmt, ap);
}

