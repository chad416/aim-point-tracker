# aim_host: the tracker service (Linux). make -C host  ->  build/aim_host
CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -g -D_GNU_SOURCE -Wall -Wextra -Wshadow -Wconversion -Wdouble-promotion -Werror
ROOT    := ..
OUT     ?= $(ROOT)/build/aim_host
SRC     := aim_host.c canio.c frames.c calibfile.c $(wildcard $(ROOT)/aimtrk/src/*.c)
HDR     := $(wildcard *.h) $(wildcard $(ROOT)/aimtrk/include/aimtrk/*.h)

$(OUT): $(SRC) $(HDR) Makefile
	@mkdir -p $(dir $(OUT))
	$(CC) $(CFLAGS) -I$(ROOT)/aimtrk/include -o $@ $(SRC) -lm

.PHONY: clean
clean:
	rm -f $(OUT)
