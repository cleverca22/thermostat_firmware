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

void display_number(int number);
#define MAXSENSORS 12
uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
int nSensors = 0;
uint8_t parasite_mode;
static FILE mystdout = FDEV_SETUP_STREAM(USART_Transmit, getch,_FDEV_SETUP_RW);

#define XBEE_ON DDRC |= _BV(PC4)
#define XBEE_OFF DDRC &= ~_BV(PC4)

uint8_t search_sensors(void);
int scan_sensors();
void check_temps();
// flags
volatile uint8_t rescan_sensors = 1;
volatile uint8_t check_temp;
volatile uint8_t dump_sensors;


EMPTY_INTERRUPT(WDT_vect);

ISR(USART_RX_vect) {
	unsigned char status = UCSR0A;
	unsigned char tmp = getch();
	switch (tmp) {
		case '1':
			rescan_sensors = 1;
			break;
		case '2':
			dump_sensors = 1;
			break;
		default:
			printf_P(PSTR("isr read in %c(%d) %d\n"),tmp,tmp,status);
	}
}

void adc_init() {
	ADMUX = _BV(REFS1) | _BV(REFS0);
	DIDR0 = _BV(ADC0D);
	DDRC &= ~_BV(PC0);
	PORTC &= ~_BV(PC0);
}
void init() {
	PRR = _BV(PRTWI) | _BV(PRTIM2) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI); // min
	uint8_t startups;
	PORTC &= ~_BV(PC4); // min
	XBEE_ON;
	USART_Init(); // min
	stdout = &mystdout;
	_delay_ms(5);
	puts_P(PSTR("serial port online"));


	adc_init();
	startups = eeprom_read_byte(0);
	startups++;
	eeprom_write_byte(0,startups);
	printf_P(PSTR("%d bootups recorded\n"),startups);
	_delay_ms(1100);
	printf("+++");
	_delay_ms(1100);
	printf("ATSM2\r");
	_delay_ms(10);
	printf("ATCN\r");
	// disable various things
	//power_adc_disable();
	//power_spi_disable();
	//power_timer0_disable();
	//power_timer1_disable();
	//power_timer2_disable();
	//power_twi_disable();
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
	printf("%d sensors\n",nSensors);
	for (y=0; y < nSensors;y++) {
		printf("sensor %d ",y);
		for (x=0; x < 8; x++) {
			printf("%02x",gSensorIDs[y][x]);
		}
		putchar('\n');
	}
	printf("parasite mode %d\n",parasite_mode);
	printf("done\n");
	dump_sensors = 0;
}
void adc_on() {
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS0);
}
void adc_off() {
	ADCSRA = _BV(ADPS2) | _BV(ADPS0);
}
int main(void) {
	char old_mcusr;

	old_mcusr = MCUSR;
	MCUSR = 0;
	//eeprom_write_byte(0x180,old_mcusr);
	clock_prescale_set(clock_div_2);
	init();
	if ((old_mcusr & (1<<BORF)) & ((old_mcusr & (1<<PORF)) == 0)) {
		printf("last reset caused by code %2d, halting\r\n",old_mcusr);
		while(1) {
			wdt_delay();
			_delay_ms(100);
		}
	}
	printf_P(PSTR("last reset caused by code %2d\n"),old_mcusr);

	//nSensors = scan_sensors();	
	// repeat forever
	sei();
	while (1) {
		adc_on();
		if (rescan_sensors) nSensors = scan_sensors();
		if (dump_sensors) do_dump_sensors();
		check_temps();
		adc_off();
		XBEE_OFF;
		wdt_delay();
	}
}
void check_temps() {
	int x;
	uint8_t ret;
	uint8_t subzero[MAXSENSORS],cel[MAXSENSORS],cel_frac_bits[MAXSENSORS];
	ret = DS18X20_start_meas(parasite_mode, NULL );
	ADCSRA |= _BV(ADSC);
	uint16_t adc_value = ADC;
	if (ret != DS18X20_OK) {
		XBEE_ON;
		_delay_ms(10);
		printf("s %d fail e\n",ret);
		rescan_sensors = 1;
		return;
	}
	_delay_ms(DS18B20_TCONV_12BIT);
	for (x=0; x < nSensors;x++) {
		ret = DS18X20_read_meas(gSensorIDs[x],&subzero[x], &cel[x], &cel_frac_bits[x]);
		if (ret != DS18X20_OK) {
			subzero[x] = cel[x] = cel_frac_bits[x] = 0;
			rescan_sensors = 1;
		}
	}
	XBEE_ON;
	_delay_ms(10);
	printf("s r ");
	uint8_t check = 0;
	for (x = 0; x < nSensors; x++) {
		if (subzero[x]) putchar('-'); //fputc('-',&mystdout);
		printf("%d %d ",cel[x],cel_frac_bits[x]);
		//printf("%d %d ",cel[x],cel_frac_bits[x]);
		check += cel[x];
		//check += cel_frac_bits[x];
	}
	printf("c %d %d e\n",check,adc_value);
}
int8_t compare(uint8_t a[OW_ROMCODE_SIZE], uint8_t b[OW_ROMCODE_SIZE]) {
	unsigned int i;
	for (i = 0; i < OW_ROMCODE_SIZE; i++) {
		if (a[i] < b[i]) return -1;
		else if (a[i] > b[i]) return 1;
	}
	return 0;
}
uint8_t search_sensors(void) {
	uint8_t i,j,k;
        uint8_t id[OW_ROMCODE_SIZE];
        uint8_t diff, nSensors;
        nSensors = 0;
	uint8_t lSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
        
        for (diff = OW_SEARCH_FIRST; 
                diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; ) {
                DS18X20_find_sensor( &diff, &id[0] );
                
                if( diff == OW_PRESENCE_ERR ) {
						printf("presence err\r\n");
                        break;
                }
                
                if( diff == OW_DATA_ERR ) {
						printf("data error\r\n");
                        break;
                }
                
                for (i=0;i<OW_ROMCODE_SIZE;i++) lSensorIDs[nSensors][i]=id[i];
                
                nSensors++;
        }
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
        
        return nSensors;
}
void check_parasite() {
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
int scan_sensors() {
	XBEE_ON;
	_delay_ms(10);
	printf("rescanning for sensors\n");
	int nSensors = 0;
	while (!nSensors) {
		nSensors = search_sensors();
		printf("found %d sensors\r\n",nSensors);
		if (!nSensors) _delay_ms(100);
	}
	check_parasite();
	rescan_sensors = 0;
	dump_sensors = 1;
	return nSensors;
}
