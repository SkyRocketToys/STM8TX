CC=sdcc
CODELOC=0x8700
CFLAGS=-mstm8 -Iinclude -DSTM8S105=1 --opt-code-size -DCODELOC=$(CODELOC) -DBLBASE=$(BLBASE)
BLBASE=0x8100
LD=sdld
CHIP=stm8s105c6
#STLINK=stlink
STLINK=stlinkv2

LIBSRC=lib/util.c lib/gpio.c lib/uart.c lib/printfl.c lib/adc.c lib/spi.c lib/cypress.c lib/timer.c lib/eeprom.c lib/buzzer.c lib/crc.c

RELOBJ = $(LIBSRC:%.c=%.rel)

txtest: txtest.ihx

pintest: pintest.ihx

.PHONY: all clean

.PRECIOUS: lib/%.rel

%.rel: %.c
	@echo Building $^
	@$(CC) -c $(CFLAGS) $^ -o $*.rel

lib/%.rel: lib/%.c
	@echo Building lib source $^
	@$(CC) -c $(CFLAGS) $^ -o lib/$*.rel

%.ihx: %/main.c $(RELOBJ)
	@echo Building binary $* at $(CODELOC)
	@$(CC) $(CFLAGS) --code-loc $(CODELOC) -o $*.ihx --out-fmt-ihx $^

bootloader.ihx: bootloader/main.c lib/gpio.rel
	@echo Building bootloader binary $* at $(BLBASE)
	@$(CC) $(CFLAGS) -o bootloader.ihx --code-loc $(BLBASE) --out-fmt-ihx $^

all: txtest bootloader pintest

clean:
	@echo Cleaning
	@rm -f $(OBJ) $(HEX) *.map *.asm *.lst *.rst *.sym *.lk *.cdb *.ihx *.rel */*.rel

txtest.flash: txtest.ihx
	@echo Flashing $^ to $(STLINK)
	@stm8flash -c$(STLINK) -p$(CHIP) -s $(CODELOC) -w $^

pintest.flash: pintest.ihx
	@echo Flashing $^ to $(STLINK)
	@stm8flash -c$(STLINK) -p$(CHIP) -s $(CODELOC) -w $^

bootloader.flash: bootloader.ihx
	@echo Flashing bootloader to $(STLINK)
	@stm8flash -c$(STLINK) -p$(CHIP) -w $^

