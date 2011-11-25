#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "my_uart.h"
#include "onewire.h"
#include "ds18x20.h"
#include "packet.h"
#include "main.h"

#define NO_PARASITE

void display_number(int number);
#define MAXSENSORS 12
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
int nSensors = 0;
#ifdef PARASITE
uint8_t parasite_mode;
#endif
static FILE mystdout = FDEV_SETUP_STREAM(USART_Transmit, getch,_FDEV_SETUP_RW);

#define XBEE_ON DDRC |= _BV(PC4)
#define XBEE_OFF DDRC &= ~_BV(PC4)

uint8_t search_sensors(void);
int scan_sensors();
int check_temps();
uint8_t check_count = 0;
// flags
enum my_flags { rescan=0x1, dump=0x2 };
volatile enum my_flags state = rescan; // 16 bit, fix it to be 8 bit
volatile uint8_t OK; // make this into a bit-field
xbee_packet isr_packet;


EMPTY_INTERRUPT(WDT_vect);
volatile char serial_buffer[20];
volatile uint8_t ser_buf_size;
volatile xbee_packet rx_packet;

void packet_recieved(xbee_packet *pkt);
void sensor_scanned(uint8_t addr[OW_ROMCODE_SIZE],uint8_t subzero,uint16_t raw_temp);
ISR(USART_RX_vect) {
	static uint8_t uart_state = 0;
	static uint8_t rx_state = 0,remaining_bytes,checksum;
	unsigned char status = UCSR0A;
	unsigned char tmp = getch();

	switch (tmp) {
		case '1':
			state |= rescan;
			break;
		case '2':
			state |= dump;
			break;
		default:
			printf("unknown byte %d\n",tmp);
	}
	return;
/*
	if ((tmp == 'O') && (uart_state == 0)) { uart_state = 1; }
	if ((tmp == 'K') && (uart_state == 1)) { uart_state = 2; }
	if ((tmp == '\r') && (uart_state == 2)) { OK=1; uart_state = 0; }
	switch (tmp) {
		case 0x7e:
			rx_packet.length_l = 0;
			rx_state = 1;
			checksum = 0;
			break;
		default:
			//start_tx_packet(&isr_packet,0,0);
			//packet_append_string_P(&isr_packet,PSTR("isr read in "));
			//snprintf_P(buffer,12,PSTR("%c(%d) %d\n"),tmp,tmp,status);
			//packet_append_string(&isr_packet,buffer);
			//packet_end_send(&isr_packet);
			switch (rx_state) {
			case 1:// length h
				if (tmp == 0) {
					rx_state++;
					break;
				}
				rx_state = 0;
				goto err;
			case 2:// length l
				remaining_bytes = tmp;
				rx_state++;
				break;
			case 3:// data
				checksum += tmp;
				rx_packet.data[rx_packet.length_l] = tmp;
				rx_packet.length_l++;
				remaining_bytes--;
				if (remaining_bytes == 0) rx_state++;
				break;
			case 4:// checksum
				checksum += tmp;
				//start_tx_packet(&isr_packet,0,0);
				//packet_append_string_P(&isr_packet,PSTR("isr read in "));
				//snprintf_P(buffer,12,PSTR("%d %c\n"),rx_packet.length_l,rx_packet.data[5]);
				//packet_append_string(&isr_packet,buffer);
				//packet_end_send(&isr_packet);
				if (checksum == 0xff) packet_recieved(&rx_packet);
			}
			return;
err:
			if (ser_buf_size < 20) {
				serial_buffer[ser_buf_size] = tmp;
				ser_buf_size++;
			}
	}
*/
}
void packet_recieved(xbee_packet *pkt) {
	switch ((uint8_t)pkt->data[0]) { // api type
	case 0x81: // 16bit tx
		if (pkt->length_l >= 6) {
			switch (pkt->data[5]) {
			case '1':
				state |= rescan;
				break;
			case '2':
				state |= dump;
				break;
			/*default:
				send_string("unknown byte\n");*/
			}
		} /*else {
			send_string("packet too short\n");
		}*/
		break;
	default:
		start_tx_packet(&isr_packet,0,0);
		packet_printf_P(&isr_packet,PSTR("%d %x\n"),pkt->length_l,pkt->data[0]);
		packet_end_send(&isr_packet);
	}
				
}

void adc_init() {
	ADMUX = _BV(REFS1) | _BV(REFS0);
	DIDR0 = _BV(ADC0D);
	DDRC &= ~_BV(PC0);
	PORTC &= ~_BV(PC0);
}
void set_zone1(unsigned char on) { // livingroom, kitchen
	static uint8_t last_state = 0;
	if (on) {
		PORTB |= _BV(PB1);
		if (last_state != 1) printf("turning livingroom on\n");
		last_state = 1;
	} else {
		PORTB &= ~_BV(PB1);
		if (last_state != 0) printf("turning livingroom off\n");
		last_state = 0;
	}
}
#define OK_WAIT while (OK == 0) {} OK=0
uint8_t startups;
void init() {
	PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI); // min
	PORTC &= ~_BV(PC4); // min
	XBEE_ON;
	USART_Init(); // min
	stdout = &mystdout;
	_delay_ms(5);
	send_string_P(PSTR("serial port online\n"));


	adc_init();
	startups = eeprom_read_byte(0);
	startups++;
	eeprom_write_byte(0,startups);
	start_tx_packet(&main_packet,0,0);
	packet_printf_P(&main_packet,PSTR("%d bootups recorded\n"),startups);
	__bss_end = startups;
	packet_end_send(&main_packet);
	//_delay_ms(1100);
	//printf("+++");
	sei();
	/*OK_WAIT;
	printf_P(PSTR("ATSM2\r"));
	OK_WAIT;
	printf_P(PSTR("ATAP1\r"));
	OK_WAIT;*/
	ser_buf_size = 0; // empty the rx error buffer
	//printf_P(PSTR("ATCN\r"));
	send_string_P(PSTR("entered api1 mode\n"));

	DDRB |= _BV(PB1); // zone 1 control is output
	set_zone1(0); // zone 1 off by default
#ifdef ignore_this_code
	SPI_MasterInit();
#endif
}
void wdt_delay() {
	//	DDRB &= ~_BV(PB1); // turn xbee off
		//PORTB &= ~_BV(PB1);
		cli();
        //__watchdog_reset();
        /* Start timed equence */
        WDTCSR |= (1<<WDCE) | (1<<WDE);
        /* Set new prescaler(time-out) value = 64K cycles (~0.5 s) */
		// 8 seconds is 1001
        WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);
        sei();

		//SMCR = (1<<SM1) | (1<<SE);
		SMCR = (1<<SE);
        __asm__ __volatile__ ("sleep"::);
		SMCR = 0;

        // turn WDT off
        cli();
        //__watchdog_reset();
        /* Clear WDRF in MCUSR */
        MCUSR &= ~(1<<WDRF);
        /* Write logical one to WDCE and WDE */
        /* Keep old prescaler setting to prevent unintentional time-out */
        WDTCSR |= (1<<WDCE) | (1<<WDE);
        /* Turn off WDT */
        WDTCSR = 0x00;
        sei();
		//DDRB |= _BV(PB1); // turn xbee on
		//PORTB &= ~_BV(PB1);
		//_delay_ms(20);
}
void do_dump_sensors() {
	_delay_ms(10);
	uint8_t x,y;
	printf("\n%d sensors\n",nSensors);
	for (y=0; y < nSensors;y++) {
		printf_P(PSTR("A %d "),(uint16_t)y);
		for (x=0; x < 8; x++) {
			printf_P(PSTR("%x "),gSensorIDs[y][x]);
		}
		printf("E\n");
		_delay_ms(30);
	}
	#ifdef PARASITE
	start_tx_packet(&main_packet,0,0);
	packet_append_string(&main_packet,"parasite mode ");
	snprintf(buffer,5,"%d\n",parasite_mode);
	packet_append_string(&main_packet,buffer);
	packet_end_send(&main_packet);
	#endif
	state &= ~dump;
}
void adc_on() {
	power_adc_enable();
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
}
void adc_off() {
	ADCSRA = _BV(ADPS2) | _BV(ADPS0);
	power_adc_disable();
}
int main(void) {
	char old_mcusr;

	old_mcusr = MCUSR;
	MCUSR = 0;
	//eeprom_write_byte(0x180,old_mcusr);
	clock_prescale_set(clock_div_2);
	init();
	if ((old_mcusr & (1<<BORF)) & ((old_mcusr & (1<<PORF)) == 0)) {
		start_tx_packet(&main_packet,0,0);
		packet_printf_P(&main_packet,PSTR("last reset caused by code %2d, halting\n"),old_mcusr);
		packet_end_send(&main_packet);
		while(1) {
			wdt_delay();
			_delay_ms(100);
		}
	}
	start_tx_packet(&main_packet,0,0);
	packet_printf(&main_packet,"last reset caused by code %2d\n",old_mcusr);
	packet_end_send(&main_packet);

	// repeat forever
	sei();
	int ret;
	while (1) {
		if (__bss_end != startups) {
			while (1) {
				send_string("stack overflow!!!\n");
				_delay_ms(1000);
			}
		}
		cli();
		if (ser_buf_size) {
			start_tx_packet(&main_packet,0,0);
			packet_append_string(&main_packet,"unknown bytes in serial buffer ");
			uint8_t x;
			for (x = 0; x < ser_buf_size; x++) {
				packet_printf(&main_packet,"%x ",(uint8_t)serial_buffer[x]);
			}
			packet_append_byte(&main_packet,'\n');
			ser_buf_size = 0;
			state |= dump;
			sei();
			packet_end_send(&main_packet);
		}
		sei();
		if (state & rescan) nSensors = scan_sensors();
		if (state & dump) do_dump_sensors();
		ret = check_temps();
		//if (ret != 1) XBEE_OFF;
		wdt_delay();
	}
}
int check_temps() {
	int main_return = 0;
	uint8_t x;
	uint8_t ret;
	uint8_t subzero,cel,cel_frac_bits;
	adc_on();
	#ifdef PARASITE
	ret = DS18X20_start_meas(parasite_mode, NULL );
	#else
	ret = DS18X20_start_meas(DS18X20_POWER_EXTERN,NULL);
	#endif
	if (ret != DS18X20_OK) {
		XBEE_ON;
		_delay_ms(10);
		send_string_P(PSTR("start fail\n"));
		state |= rescan;
		return;
	}
	ADCSRA |= _BV(ADSC);
	_delay_ms(DS18B20_TCONV_12BIT);
	uint16_t adc_value = ADC;
	if (adc_value == 1023) main_return = 1;
	adc_off();
	start_tx_packet(&main_packet,0,0);
	packet_append_string_P(&main_packet,PSTR("s r "));
	//xbee_packet bin_packet;
	//start_tx_packet(&bin_packet,0,0);
	//packet_append_byte(&bin_packet,0x00); // binary flag
	//packet_append_byte(&bin_packet,0x02); // temp dump
	//packet_append_byte(&bin_packet,nSensors);
	//packet_append_byte(&bin_packet,(adc_value >> 8) & 0xff);
	//packet_append_byte(&bin_packet,adc_value & 0xff);
	for (x=0; x < nSensors;x++) {
		#if 0
		ret = DS18X20_read_meas(gSensorIDs[x],&subzero, &cel, &cel_frac_bits);
		#endif
		uint8_t scratchpad[DS18X20_SP_SIZE];
		ret = DS18X20_read_scratchpad(gSensorIDs[x],scratchpad);
		if (ret != DS18X20_OK) { // short circuit
			packet_append_string_P(&main_packet,PSTR("U U "));
			continue;
		}
		if (crc8(&scratchpad[0], DS18X20_SP_SIZE)) { // CRC error
			packet_append_string_P(&main_packet,PSTR("U U "));
			state |= rescan;
			continue;
		}
//memcpy(&(bin_packet.data[bin_packet.length_l]), scratchpad,2);
//bin_packet.length_l = bin_packet.length_l + 2;
		DS18X20_meas_to_cel(gSensorIDs[x][0], scratchpad, &subzero, &cel, &cel_frac_bits);
		uint16_t raw_temp = (cel * 16) + cel_frac_bits;
		if (ret == DS18X20_OK) {
			sensor_scanned(gSensorIDs[x],subzero,raw_temp);
			if (subzero) packet_append_byte(&main_packet,'-');
			packet_printf(&main_packet,"%d %d ",cel,cel_frac_bits);
		} else {
			state |= rescan;
			packet_append_string_P(&main_packet,PSTR("U U "));
		}
	}
	packet_printf(&main_packet,"c %d %d %d e\n",adc_value,check_count++,PORTB);
	packet_end_send(&main_packet);
	_delay_ms(100);
	//packet_end_send(&bin_packet);
	return main_return;
}
uint8_t living_room[OW_ROMCODE_SIZE] = { 0x28, 0xcc, 0x7c, 0xcd, 0x02, 0x00, 0x00, 0x5b }; 
int8_t compare(uint8_t a[OW_ROMCODE_SIZE], uint8_t b[OW_ROMCODE_SIZE]) {
	unsigned int i;
	for (i = 0; i < OW_ROMCODE_SIZE; i++) {
		if (a[i] < b[i]) return -1;
		else if (a[i] > b[i]) return 1;
	}
	return 0;
}
#define CC(cel,frac) ((cel * 16) + frac)
uint16_t livingroom_min = CC(22,0);
uint16_t livingroom_max = CC(22,8);
void sensor_scanned(uint8_t addr[OW_ROMCODE_SIZE],uint8_t subzero,uint16_t raw_temp) {
	if (compare(living_room,addr) == 0) {
		if (livingroom_min > raw_temp) set_zone1(1);
		if (livingroom_max < raw_temp) set_zone1(0);
	}
}
uint8_t search_sensors(void) {
	uint8_t i,j,k;
        uint8_t id[OW_ROMCODE_SIZE];
        uint8_t diff, nSensors;
        nSensors = 0;
	uint8_t lSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
        
        for (diff = OW_SEARCH_FIRST; 
                diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; ) {
		// DS18X20_find_sensor is a wrapper to filter the list, but i have nothing to filter out
		DS18X20_find_sensor( &diff, &id[0] );
                
                if( diff == OW_PRESENCE_ERR ) {
			//printf("presence err %x\n",diff);
                        break;
                }
                
                if( diff == OW_DATA_ERR ) {
			//printf("data error\r\n");
                        break;
                }
                
#ifdef SORT
                for (i=0;i<OW_ROMCODE_SIZE;i++) lSensorIDs[nSensors][i]=id[i];
#else
                for (i=0;i<OW_ROMCODE_SIZE;i++) gSensorIDs[nSensors][i]=id[i];
#endif
                
                nSensors++;
        }
#ifdef SORT
		uint8_t lowest[OW_ROMCODE_SIZE];
		int lowest_id = -1;
		uint8_t used[MAXSENSORS];
		uint8_t reset_lowest = 1;
		for (i = 0; i < MAXSENSORS; i++) used[i] = 0;
		for (k = 0; k < nSensors; k++) {
			for (i = 0; i < nSensors; i++) {
				if (used[i] == 0) {
					if (reset_lowest) { for (j=0;j<OW_ROMCODE_SIZE;j++) lowest[j] = lSensorIDs[i][j]; reset_lowest = 0; lowest_id = i; }
					//printf("%d %d %x %x\n",k,i,lSensorIDs[i][1],lowest[1]);
					if (compare(lSensorIDs[i],lowest) == -1) {
						for (j = 0; j < OW_ROMCODE_SIZE; j++) lowest[j] = lSensorIDs[i][j];
						lowest_id = i;
					}
				}
			}
			for (i=0;i<OW_ROMCODE_SIZE;i++) gSensorIDs[k][i] = lowest[i];
			used[lowest_id] = 1;
			reset_lowest = 1;
		}
#endif
        
        return nSensors;
}
#ifdef PARASITE
void check_parasite() {
	int x;
	parasite_mode = DS18X20_POWER_EXTERN;
	start_tx_packet(&main_packet,0,0);
	packet_append_string(&main_packet,"p ");
	char buffer[4];
	for (x = 0; x < nSensors; x++) {
		int parasite = DS18X20_get_power_status(gSensorIDs[x]);
		snprintf(buffer,4,"%d ",parasite);
		packet_append_string(&main_packet,buffer);
		if (parasite == DS18X20_POWER_PARASITE) {
			parasite_mode = parasite;
			break;
		}
	}
	packet_append_string(&main_packet,"e\n");
	packet_end_send(&main_packet);
}
#endif
int scan_sensors() {
	XBEE_ON;
	_delay_ms(10);
	send_string_P(PSTR("rescanning for sensors\n"));
	int nSensors = 0;
	while (!nSensors) {
		nSensors = search_sensors();
		start_tx_packet(&main_packet,0,0);
		packet_printf_P(&main_packet,PSTR("found %d sensors\n"),nSensors);
		packet_end_send(&main_packet);
		if (!nSensors) _delay_ms(100);
	}
	#ifdef PARASITE
	check_parasite();
	#endif
	state = dump;
	return nSensors;
}
