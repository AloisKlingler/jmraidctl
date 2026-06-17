CC = gcc
CFLAGS = -g -O2 -Wall -std=gnu99

all: jmraidctl

jmraidctl: src/jmraidctl.c src/jm_crc.c src/sata_xor.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f jmraidctl

.PHONY: all clean
