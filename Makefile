CC=sdcc
CFLAGS=-lstm8 -mstm8
LD=sdld
CHIP=stm8s105c6

SRC=util.c uart.c test.c printfl.c

OBJ = $(SRC:%.c=%.rel)
HEX = test.ihx

.PHONY: all clean

%.rel: %.c
	$(CC) -c $(CFLAGS) $^

all: $(OBJ) $(HEX)

clean:
	rm -f $(OBJ) $(HEX)

flash: $(HEX)
	stm8flash -cstlinkv2 -p$(CHIP) -w $(HEX)

$(HEX) : $(OBJ)
	$(CC) $(CFLAGS) -o $(HEX) --out-fmt-ihx $(OBJ)

