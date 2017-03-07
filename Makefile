CC=sdcc
CFLAGS=-lstm8 -mstm8 -Iinclude -DSTM8S105=1
LD=sdld
CHIP=stm8s105c6

LIBSRC=lib/util.c lib/gpio.c lib/uart.c lib/printfl.c lib/adc.c lib/spi.c lib/cypress.c

LIBOBJ = $(LIBSRC:%.c=%.rel)

txtest: txtest.ihx

pintest: pintest.ihx

.PHONY: all clean

%.rel: %.c
	$(CC) -c $(CFLAGS) $^ -o $*.rel

%.ihx: %/main.rel $(LIBOBJ)
	$(CC) $(CFLAGS) -o $*.ihx --out-fmt-ihx $^

all: txtest pintest

clean:
	rm -f $(OBJ) $(HEX) *.map *.asm *.lst *.rst *.sym *.lk *.cdb *.ihx *.rel

txtest.flash: txtest.ihx
	stm8flash -cstlinkv2 -p$(CHIP) -w $^

pintest.flash: pintest.ihx
	stm8flash -cstlinkv2 -p$(CHIP) -w $^

$(HEX) : $(OBJ)
