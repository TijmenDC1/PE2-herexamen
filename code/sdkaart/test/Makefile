# PC-test van de vluchtplan-keten. Vereist een gewone gcc (MSYS2/MinGW, WSL,
# of Linux). Heb je die niet, dan test je gewoon op het bord zelf, zie README.md.
#
#   make run     bouwen en draaien
#   make clean    alles opruimen, ook de namaak-SD-kaart

CC      ?= gcc
CFLAGS  := -std=gnu11 -Wall -Wextra -O0 -g -Istub -I../Core/Inc
LDLIBS  := -lm

SRC := test_flightplan.c \
       stub/host_fatfs.c \
       ../Core/Src/sdcard.c \
       ../Core/Src/flightplan.c \
       ../Core/Src/flightplan_io.c

BIN := test_flightplan

all: $(BIN)

$(BIN): $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDLIBS)

run: $(BIN)
	./$(BIN)

clean:
	rm -rf $(BIN) $(BIN).exe sdtest

.PHONY: all run clean
