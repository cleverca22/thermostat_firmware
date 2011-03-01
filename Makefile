CHIP=m168
CHIP_GCC=atmega168
F_CPU=4000000UL
BAUD=9600

DS18B20=ds18x20/DS18X20_get_power_status.o ds18x20.o ds18x20/DS18X20_read_meas.o ds18x20/DS18X20_meas_to_cel.o ds18x20/DS18X20_find_sensor.o
OBJECTS=main.o my_uart.o onewire.o ${DS18B20} crc8.o
.SECONDARY: main.S my_uart.S

CFLAGS=-mmcu=${CHIP_GCC} -Os -DF_CPU=${F_CPU} -DBAUD=${BAUD}
CC=avr-gcc

all: avr.bin

%.o: %.S
	avr-gcc -mmcu=${CHIP_GCC} -Os -c $< -o $@

avr.bin: a.out
	avr-objcopy -j .text -j .data -O binary a.out avr.bin

a.out: ${OBJECTS}
	${CC} ${OBJECTS} -mmcu=${CHIP_GCC} -Os -o a.out -Wl,-u,vfprintf -lprintf_min

size: a.out ${OBJECTS}
	avr-size -t ${OBJECTS}
	avr-size a.out
	@echo max text+data: 16384, max data+bss: 1024
	ls -l a.out ${OBJECTS}

%.S: %.c *.h
	avr-gcc ${CFLAGS} -S $< -o $@

program: avr.bin
	beep -r 2
	avrdude -E noreset -p m168 -U flash:w:avr.bin -y -i 15 -P /dev/ttyS0
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
	mv -v avr.bin avr.bin.backup
fat_functions: a.out
	avr-objdump -t a.out|sort -rk4
router_program: avr.bin
	scp avr.bin newrouter:/home/clever/avr/avr.bin
	ssh newrouter -t /usr/local/bin/avrdude -p ${CHIP} -U flash:w:/home/clever/avr/avr.bin -y -i 15
