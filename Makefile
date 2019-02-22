CHIP=m168
CHIP_GCC=atmega168
F_CPU=4000000UL
BAUD=9600

DS18B20=ds18x20/DS18X20_get_power_status.o ds18x20.o ds18x20/DS18X20_read_meas.o ds18x20/DS18X20_meas_to_cel.o ds18x20/DS18X20_find_sensor.o
OBJECTS=packet.o main.o my_uart.o onewire.o ${DS18B20} crc8.o
.SECONDARY: main.S my_uart.S

CFLAGS=-mmcu=${CHIP_GCC} -O2 -DF_CPU=${F_CPU} -DBAUD=${BAUD}
CC=avr-gcc

all: avr.bin

%.o: %.c
	avr-gcc -mmcu=${CHIP_GCC} -c $< -o $@ ${CFLAGS}

avr.bin: a.out
	avr-objcopy -j .text -j .data -O binary a.out avr.bin

a.out: ${OBJECTS}
	${CC} ${OBJECTS} -mmcu=${CHIP_GCC} -o a.out -Wl,-u,vfprintf -lprintf_min -Wl,-Map,output.map -Wl,-T,/usr/lib/binutils/avr/2.24/ldscripts/avr5.x -lc

size: a.out ${OBJECTS}
	avr-size -t ${OBJECTS}
	avr-size a.out
	@echo max text+data: 16384, max data+bss: 1024
	avr-nm a.out -S --size-sort|tail
	ls -l a.out ${OBJECTS}

%.asm: %.c *.h
	-mv $@ $@.old
	avr-gcc ${CFLAGS} -S $< -o $@

program: avr.bin
	beep -r 2
	avrdude -E noreset -p m168 -U flash:w:avr.bin -y -i 15
	beep -r 3
wifi_program: avr.bin
	wifi_program
set_fuses:
	avrdude -E reset -p m168 -u -U lfuse:w:0x62:m -U hfuse:w:0xd5:m -i 50
read: status
	mv prom.hex prom.old.hex
	avrdude -E noreset -p m168 -U eeprom:r:prom.hex:i
	cat prom.hex
	diff prom.hex prom.old.hex || exit 0
status:
	avrdude -E reset -p ${CHIP} -v -i 50
status_resume:
	avrdude -E noreset -p ${CHIP} -v -i 50
clean:
	-rm ${OBJECTS} a.out
	-mv -v avr.bin avr.bin.backup
fat_functions: a.out
	avr-objdump -t a.out|sort -rk4|head -n15
router_program: avr.bin
	scp avr.bin newrouter:/home/clever/avr/avr.bin
	ssh newrouter -t time /usr/local/bin/avrdude -p ${CHIP} -U flash:w:/home/clever/avr/avr.bin -y
main.o: main.c packet.h main.h
full_dump.lss: a.out
	avr-objdump -h -S a.out > full_dump.lss
