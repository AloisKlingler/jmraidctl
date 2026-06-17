CC = gcc
CFLAGS = -g -O2 -Wall -std=gnu99
SRCDIR = src

all: JMraidcon jmraidctl

JMraidcon: $(SRCDIR)/JMraidcon.c $(SRCDIR)/jm_crc.c $(SRCDIR)/sata_xor.c
	$(CC) $(CFLAGS) $^ -o $@

jmraidctl: $(SRCDIR)/jmraidctl.c $(SRCDIR)/jm_crc.c $(SRCDIR)/sata_xor.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	-rm -f JMraidcon jmraidctl
