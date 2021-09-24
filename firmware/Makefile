#
# Top level Makefile
#


include Make.inc

## General Flags
PROJECT = fan
TARGET = fan.elf

## Linker flags
LDFLAGS +=  -Wl,-Map=fan.map
CFLAGS += -I. 

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Objects that must be built in order to link
OBJECTS = delay.o fan.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all:	$(TARGET) fan.hex fan.eep fan.lss size

## Compile
fan.o: fan.c delay.h delay.c
	@echo `pwd`
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
#       @avr-size -C --mcu=${MCU} ${TARGET}
	@avr-size --target=ihex ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) fan.elf dep/* fan.hex fan.eep fan.lss fan.map *.o.d

prog:
	avrdude -p t13 -c avrispmkII -P usb -e -U flash:w:fan.hex:i

fuses:
	#avrdude -p t13 -c avrispmkII -P usb -U lfuse:w:0x6a:m # CKDIV8=0: 1.2MHz
	avrdude -p t13 -c avrispmkII -P usb -U lfuse:w:0x7a:m # CKDIV8=1: 9.6MHz
	avrdude -p t13 -c avrispmkII -P usb -U hfuse:w:0xff:m



## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)
