CC=sdcc
CFLAGS=-lstm8 -mstm8
LD=sdld

SRC=util.c test.c
OBJ = $(SRC:%.c=%.rel)
HEX = test.ihx

.PHONY: all clean

%.rel: %.c
	$(CC) -c $(CFLAGS) $^

all: $(OBJ) $(HEX)

clean:
	rm -f $(OBJ) $(HEX)

flash: $(HEX)
	stm8flash -cstlinkv2 -pstm8s105c6 -w $(HEX)

$(HEX) : $(OBJ)
	$(CC) $(CFLAGS) -o $(HEX) --out-fmt-ihx $(OBJ)

