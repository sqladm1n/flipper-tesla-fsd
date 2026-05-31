# Host unit tests for the Tesla FSD protocol core.
# Compiles fsd_logic/fsd_handler.c on the host — no Flipper SDK / SPI needed.
#
#   make -C test check

CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -O0 -g
ROOT    := ..
SRC     := test_fsd_core.c $(ROOT)/fsd_logic/fsd_handler.c

test_fsd_core: $(SRC) $(ROOT)/fsd_logic/fsd_handler.h $(ROOT)/fsd_logic/fsd_types.h $(ROOT)/fsd_logic/fsd_checksum.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(SRC) -o $@ -lm

check: test_fsd_core
	./test_fsd_core

clean:
	rm -rf test_fsd_core test_fsd_core.dSYM

.PHONY: check clean
