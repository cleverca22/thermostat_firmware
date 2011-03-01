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
// flags
enum my_flags { rescan=0x1, dump=0x2 };
volatile enum my_flags state = rescan; // 16 bit, fix it to be 8 bit
volatile uint8_t OK; // make this into a bit-field
xbee_packet isr_packet;


EMPTY_INTERRUPT(WDT_vect);
volatile char serial_buffer[20];
volatile uint8_t ser_buf_size;

ISR(USART_RX_vect) {
	static uint8_t uart_state = 0;
	unsigned char status = UCSR0A;
	unsigned char tmp = getch();
	char buffer[12];
	if ((tmp == 'O') && (uart_state == 0)) { uart_state = 1; return; }
	if ((tmp == 'K') && (uart_state == 1)) { uart_state = 2; return; }
	if ((tmp == '\r') && (uart_state == 2)) { OK=1; uart_state = 0; return; }
	switch (tmp) {
		case '1':
			state |= rescan;
			break;
		case '2':
			state |= dump;
			break;
		default:
			//start_tx_packet(&isr_packet,0,0);
			//packet_append_string_P(&isr_packet,PSTR("isr read in "));
			//snprintf_P(buffer,12,PSTR("%c(%d) %d\n"),tmp,tmp,status);
			//packet_append_string(&isr_packet,buffer);
			//packet_end_send(&isr_packet);
			if (ser_buf_size < 20) {
				serial_buffer[ser_buf_size] = tmp;
				ser_buf_size++;
			}
	}
}

void adc_init() {
	ADMUX = _BV(REFS1) | _BV(REFS0);
	DIDR0 = _BV(ADC0D);
	DDRC &= ~_BV(PC0);
	PORTC &= ~_BV(PC0);
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
	char buffer[20];
	snprintf_P(buffer,20,PSTR("%d bootups recorded\n"),startups);
	stack_mark = startups;
	packet_append_string(&main_packet,buffer);
	packet_end_send(&main_packet);
	_delay_ms(1100);
	printf("+++");
	sei();
	OK_WAIT;
	printf_P(PSTR("ATSM2\r"));
	OK_WAIT;
	printf_P(PSTR("ATAP1\r"));
	OK_WAIT;
	printf_P(PSTR("ATCN\r"));
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
	XBEE_ON;
	_delay_ms(10);
	int x,y;
	start_tx_packet(&main_packet,0,0);
	char buffer[5];
	snprintf(buffer,5,"%d ",nSensors);
	packet_append_string(&main_packet,buffer);
	packet_append_string(&main_packet,"sensors\n");
	packet_end_send(&main_packet);
	for (y=0; y < nSensors;y++) {
		_delay_ms(5);
		start_tx_packet(&main_packet,0,0);
		packet_append_byte(&main_packet,0x00);
		packet_append_byte(&main_packet,0x01);
		packet_append_byte(&main_packet,y);
		for (x=0; x < 8; x++) {
			packet_append_byte(&main_packet,gSensorIDs[y][x]);
		}
		packet_end_send(&main_packet);
	}
	#ifdef PARASITE
	printf("parasite mode %d\n",parasite_mode);
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
		packet_append_string_P(&main_packet,PSTR("last reset caused by code %2d, halting\n")); //),old_mcusr); FIXME
		packet_end_send(&main_packet);
		while(1) {
			wdt_delay();
			_delay_ms(100);
		}
	}
	char buffer[5];
	start_tx_packet(&main_packet,0,0);
	packet_append_string(&main_packet,"last reset caused by code ");
	snprintf(buffer,5,"%2d\n",old_mcusr);
	packet_append_string(&main_packet,buffer);
	packet_end_send(&main_packet);

	// repeat forever
	sei();
	int ret;
	while (1) {
		if (stack_mark != startups) {
			while (1) {
				send_string("stack overflow!!!\n");
				_delay_ms(1000);
			}
		}
		cli();
		if (ser_buf_size) {
			start_tx_packet(&main_packet,0,0);
			packet_append_string(&main_packet,"unknown bytes in serial buffer ");
			int x;
			for (x = 0; x < ser_buf_size; x++) {
				packet_append_byte(&main_packet,serial_buffer[x]);
			}
			packet_append_byte(&main_packet,'\n');
			packet_end_send(&main_packet);
			ser_buf_size = 0;
		}
		sei();
		if (state & rescan) nSensors = scan_sensors();
		if (state & dump) do_dump_sensors();
		ret = check_temps();
		if (ret != 1) XBEE_OFF;
		wdt_delay();
	}
}
int check_temps() {
	int main_return = 0;
	unsigned int x;
	uint8_t ret;
	uint8_t subzero[MAXSENSORS],cel[MAXSENSORS],cel_frac_bits[MAXSENSORS];
	adc_on();
	#ifdef PARASITE
	ret = DS18X20_start_meas(parasite_mode, NULL );
	#else
	ret = DS18X20_start_meas(DS18X20_POWER_EXTERN,NULL);
	#endif
	ADCSRA |= _BV(ADSC);
	if (ret != DS18X20_OK) {
		XBEE_ON;
		_delay_ms(10);
		send_string("start fail\n");
		state |= rescan;
		return;
	}
	_delay_ms(DS18B20_TCONV_12BIT);
	uint16_t adc_value = ADC;
	if (adc_value == 1023) main_return = 1;
	adc_off();
	for (x=0; x < nSensors;x++) {
		ret = DS18X20_read_meas(gSensorIDs[x],&subzero[x], &cel[x], &cel_frac_bits[x]);
		if (ret != DS18X20_OK) {
			subzero[x] = cel[x] = cel_frac_bits[x] = 0;
			state |= rescan;
		}
	}
	XBEE_ON;
	_delay_ms(10);
	start_tx_packet(&main_packet,0,0);
	packet_append_string(&main_packet,"s r ");
	uint8_t check = 0;
	char buffer[15];
	for (x = 0; x < nSensors; x++) {
		if (subzero[x]) putchar('-'); //fputc('-',&mystdout);
		snprintf(buffer,15,"%d %d ",cel[x],cel_frac_bits[x]);
		packet_append_string(&main_packet,buffer);
		check += cel[x];
	}
	snprintf(buffer,15,"c %d %d e\n",check,adc_value);
	packet_append_string(&main_packet,buffer);
	packet_end_send(&main_packet);
	return main_return;
}
#ifdef SORT
int8_t compare(uint8_t a[OW_ROMCODE_SIZE], uint8_t b[OW_ROMCODE_SIZE]) {
	unsigned int i;
	for (i = 0; i < OW_ROMCODE_SIZE; i++) {
		if (a[i] < b[i]) return -1;
		else if (a[i] > b[i]) return 1;
	}
	return 0;
}
#endif
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
void check_parasite() { FIXME
	int x;
	parasite_mode = DS18X20_POWER_EXTERN;
	printf("p ");
	for (x = 0; x < nSensors; x++) {
		int parasite = DS18X20_get_power_status(gSensorIDs[x]);
		printf("%d ",parasite);
		if (parasite == DS18X20_POWER_PARASITE) {
			parasite_mode = parasite;
			return;
		}
	}
	printf(" e\n");
}
#endif
int scan_sensors() {
	XBEE_ON;
	_delay_ms(10);
	start_tx_packet(&main_packet,0,0);
	packet_append_string(&main_packet,"rescanning for sensors\n");
	packet_end_send(&main_packet);
	int nSensors = 0;
	while (!nSensors) {
		nSensors = search_sensors();
		start_tx_packet(&main_packet,0,0);
		packet_append_string(&main_packet,"found ");
		char buffer[4];
		snprintf(buffer,4,"%d",nSensors);
		packet_append_string(&main_packet,buffer);
		packet_append_string(&main_packet," sensors\n");
		packet_end_send(&main_packet);
		if (!nSensors) _delay_ms(100);
	}
	#ifdef PARASITE
	check_parasite();
	#endif
	state = dump;
	return nSensors;
}
