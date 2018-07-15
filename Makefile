# Makefile for programming the ATMega328
# modified the one generated by CrossPack

DEVICE     = atmega328p
CLOCK      = 16000000
PROGRAMMER = -c usbtiny 
OBJECTS    = src/main.o
# for ATMega328

# Tune the lines below only if you know what you are doing:
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

# symbolic targets:
all:	src/main.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:src/main.hex:i

# fuse:
# 	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID src/main.hex

clean:
	rm -f src/main.hex src/main.elf $(OBJECTS)

# file targets:
src/main.elf: $(OBJECTS)
	$(COMPILE) -o src/main.elf $(OBJECTS)

src/main.hex: src/main.elf
	rm -f src/main.hex
	avr-objcopy -j .text -j .data -O ihex src/main.elf src/main.hex
	avr-size --format=avr --mcu=$(DEVICE) src/main.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	src/main.elf
	avr-objdump -d src/main.elf

cpp:
	$(COMPILE) -E src/main.c