# Solar Powered Personal Cloud Power Controller
# (C) 2016 Stuart Longland
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation; either version 2 of the License, or (at your
# option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.

-include local.mk

CFLAGS = -Os -g -mmcu=attiny24a -Wall -Werror
LDFLAGS = -mmcu=attiny24a -Wall -Werror
CPPFLAGS = -DF_CPU=1000000UL
CROSS_COMPILE ?= avr-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size
PROG_ARGS ?=-c stk500v2 -P /dev/ttyACM0
PROG_DEV ?= t24

.PHONY: clean all

all: powerctl.ihex

clean:
	-rm *.ihex *.elf *.o

%.ihex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $^ $@

%.elf:
	$(CC) $(LDFLAGS) -o $@ $^
	$(SIZE) -d $@

powerctl.elf: powerctl.o uart.o
powerctl.o: board.h
uart.o: uartcfg.h uart.h

%.pgm: %.ihex
	avrdude $(PROG_ARGS) -p $(PROG_DEV) -U flash:w:$^:i