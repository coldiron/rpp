CC = gcc
CFLAGS = -Wall -O

all: rpp

rpp: rpp.c
	$(CC) $(CFLAGS) rpp.c -o rpp

clean:
	rm -f rpp

.PHONY: clean
