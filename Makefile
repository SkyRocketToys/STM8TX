CC=sdcc
CFLAGS=-lstm8 -mstm8 -Iinclude -DSTM8S105=1
LD=sdld
CHIP=stm8s105c6

LIB=lib/util.c lib/gpio.c lib/uart.c lib/printfl.c lib/adc.c lib/spi.c lib/cypress.c
SRC=$(LIB) test.c 

OBJ = $(SRC:%.c=%.rel)
HEX = test.ihx

.PHONY: all clean

%.rel: %.c
	$(CC) -c $(CFLAGS) $^ -o $*.rel

all: $(OBJ) $(HEX)

clean:
	rm -f $(OBJ) $(HEX) *.map *.asm *.lst *.rst *.sym *.lk *.cdb *.ihx *.rel

flash: $(HEX)
	stm8flash -cstlinkv2 -p$(CHIP) -w $(HEX)

$(HEX) : $(OBJ)
	$(CC) $(CFLAGS) -o $(HEX) --out-fmt-ihx $(OBJ)

