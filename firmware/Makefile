CPU_FREQ = 16000000
DEVICEL = atmega16
DEVICES = m16
PORT = usb
PROGRAMMER = usbtiny

AVRDUDE = avrdude -c $(PROGRAMMER) -p $(DEVICES) -P $(PORT)
CC = avr-gcc
CFLAGS = -std=c99 -Wall -O3 -mmcu=$(DEVICEL) -DF_CPU=$(CPU_FREQ)

OBJECTS = main.o

all: main.hex

clean:
	rm -f *.hex *.lst *.o *.bin

dumpeeprom:
	$(AVRDUDE) -U eeprom:r:dump:h
	@cat dump

flash: all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) -U lfuse:w:0xef:m

main.bin: $(OBJECTS)
	$(CC) $(CFLAGS) -o main.bin $(OBJECTS)

main.hex: main.bin
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	avr-size --totals *.o
