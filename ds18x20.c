/*********************************************************************************
Title:    DS18X20-Functions via One-Wire-Bus
Author:   Martin Thomas <eversmith@heizung-thomas.de>   
          http://www.siwawi.arubi.uni-kl.de/avr-projects
Software: avr-gcc 3.4.1 / avr-libc 1.0.4 
Hardware: any AVR - tested with ATmega16/ATmega32 and 3 DS18B20

Partly based on code from Peter Dannegger and others

changelog:
20041124 - Extended measurements for DS18(S)20 contributed by Carsten Foss (CFO)
200502xx - function DS18X20_read_meas_single
20050310 - DS18x20 EEPROM functions (can be disabled to save flash-memory)
           (DS18X20_EEPROMSUPPORT in ds18x20.h)

**********************************************************************************/

#include <avr/io.h>

#include "ds18x20.h"
#include "onewire.h"
#include "crc8.h"

#ifdef DS18X20_EEPROMSUPPORT
// for 10ms delay in copy scratchpad
#include <util/delay.h>
#endif

/*----------- start of "debug-functions" ---------------*/
#ifdef DS18X20_VERBOSE
/* functions for debugging-output - undef DS18X20_VERBOSE in .h
   if you run out of program-memory */
#include <string.h>
#include "uart.h"

void DS18X20_uart_put_temp(const uint8_t subzero, 
	const uint8_t cel, 	const uint8_t cel_frac_bits)
{
	char buffer[sizeof(int)*8+1];
	size_t i;
	
	uart_putc((subzero)?'-':'+');
	uart_puti((int)cel);
	uart_puts_P(".");
	itoa(cel_frac_bits*DS18X20_FRACCONV,buffer,10);
	for (i=0;i<4-strlen(buffer);i++) {
		uart_puts_P("0");
	}
	uart_puts(buffer);
	uart_puts_P("°C");
}

void DS18X20_show_id_uart( uint8_t *id, size_t n )
{
	size_t i;
	for( i = 0; i < n; i++ ) {
	    if ( i == 0 ) uart_puts_P( "FC:" );
		else if ( i == n-1 ) uart_puts_P( "CRC:" );
		if ( i == 1 ) uart_puts_P( "SN: " );
		uart_puthex_byte(id[i]);
		uart_puts_P(" ");
		if ( i == 0 ) {
			if ( id[0] == DS18S20_ID ) uart_puts_P ("(18S)");
			else if ( id[0] == DS18B20_ID ) uart_puts_P ("(18B)");
			else uart_puts_P ("( ? )");
		}
	}
	if ( crc8( id, OW_ROMCODE_SIZE) )
		uart_puts_P( " CRC FAIL " );
	else 
		uart_puts_P( " CRC O.K. " );
}

void show_sp_uart( uint8_t *sp, size_t n )
{
	size_t i;
	uart_puts_P( "SP:" );
	for( i = 0; i < n; i++ ) {
		if ( i == n-1 ) uart_puts_P( "CRC:" );
		uart_puthex_byte(sp[i]);
		uart_puts_P(" ");
	}
}

/* verbose output rom-search follows read-scratchpad in one loop */
uint8_t DS18X20_read_meas_all_verbose( void )
{
	uint8_t id[OW_ROMCODE_SIZE], sp[DS18X20_SP_SIZE], diff;
	
	uint8_t i;
	uint16_t meas;
	
	uint8_t subzero, cel, cel_frac_bits;
	
	for( diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE; )
	{
		diff = ow_rom_search( diff, &id[0] );

		if( diff == OW_PRESENCE_ERR ) {
		  uart_puts_P( "No Sensor found\r" );
		  return OW_PRESENCE_ERR;
		}
		
		if( diff == OW_DATA_ERR ) {
		  uart_puts_P( "Bus Error\r" );
		  return OW_DATA_ERR;
		}
		
		DS18X20_show_id_uart( id, OW_ROMCODE_SIZE );
		
		if( id[0] == DS18B20_ID || id[0] == DS18S20_ID ) {	 // temperature sensor
			
			uart_putc ('\r');
			
			ow_byte_wr( DS18X20_READ );			// read command
			
			for ( i=0 ; i< DS18X20_SP_SIZE; i++ )
				sp[i]=ow_byte_rd();
			
			show_sp_uart( sp, DS18X20_SP_SIZE );

			if ( crc8( &sp[0], DS18X20_SP_SIZE ) )
				uart_puts_P( " CRC FAIL " );
			else 
				uart_puts_P( " CRC O.K. " );
			uart_putc ('\r');
		
			meas = sp[0]; // LSB Temp. from Scrachpad-Data
			meas |= (uint16_t) (sp[1] << 8); // MSB
			
			uart_puts_P(" T_raw=");
			uart_puthex_byte((uint8_t)(meas>>8));
			uart_puthex_byte((uint8_t)meas);
			uart_puts_P(" ");

			if( id[0] == DS18S20_ID ) { // 18S20
				uart_puts_P( "S20/09" );
			}
			else if ( id[0] == DS18B20_ID ) { // 18B20
				i=sp[DS18B20_CONF_REG];
				if ( (i & DS18B20_12_BIT) == DS18B20_12_BIT ) {
					uart_puts_P( "B20/12" );
				}
				else if ( (i & DS18B20_11_BIT) == DS18B20_11_BIT ) {
					uart_puts_P( "B20/11" );
				}
				else if ( (i & DS18B20_10_BIT) == DS18B20_10_BIT ) {
					uart_puts_P( " B20/10 " );
				}
				else { // if ( (i & DS18B20_9_BIT) == DS18B20_9_BIT ) { 
					uart_puts_P( "B20/09" );
				}
			}			
			uart_puts_P(" ");
			
			DS18X20_meas_to_cel(id[0], sp, &subzero, &cel, &cel_frac_bits);
			
			DS18X20_uart_put_temp(subzero, cel, cel_frac_bits);
			
			uart_puts("\r");
			
		} // if meas-sensor
		
	} // loop all sensors
	
	uart_puts_P( "\r" );
	
	return DS18X20_OK;
}
#endif

/*----------- end of "debug-functions" ---------------*/





/* start measurement (CONVERT_T) for all sensors if input id==NULL 
   or for single sensor. then id is the rom-code */
uint8_t DS18X20_start_meas( uint8_t with_power_extern, uint8_t id[])
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_CONVERT_T, id );
		if (with_power_extern != DS18X20_POWER_EXTERN)
			ow_parasite_enable();
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		uart_puts_P( "DS18X20_start_meas: Short Circuit !\r" );
		#endif
		return DS18X20_START_FAIL;
	}
}



#ifdef DS18X20_EEPROMSUPPORT

uint8_t DS18X20_write_scratchpad( uint8_t id[], 
	uint8_t th, uint8_t tl, uint8_t conf)
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_WRITE_SCRATCHPAD, id );
		ow_byte_wr(th);
		ow_byte_wr(tl);
		if (id[0] == DS18B20_ID) ow_byte_wr(conf); // config avail. on B20 only
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		uart_puts_P( "DS18X20_write_scratchpad: Short Circuit !\r" );
		#endif
		return DS18X20_ERROR;
	}
}

uint8_t DS18X20_read_scratchpad( uint8_t id[], uint8_t sp[] )
{
	uint8_t i;
	
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_READ, id );
		for ( i=0 ; i< DS18X20_SP_SIZE; i++ )	sp[i]=ow_byte_rd();
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		uart_puts_P( "DS18X20_read_scratchpad: Short Circuit !\r" );
		#endif
		return DS18X20_ERROR;
	}
}

uint8_t DS18X20_copy_scratchpad( uint8_t with_power_extern, 
	uint8_t id[] )
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_COPY_SCRATCHPAD, id );
		if (with_power_extern != DS18X20_POWER_EXTERN)
			ow_parasite_enable();
		_delay_ms(DS18X20_COPYSP_DELAY); // wait for 10 ms 
		if (with_power_extern != DS18X20_POWER_EXTERN)
			ow_parasite_disable();
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		uart_puts_P( "DS18X20_copy_scratchpad: Short Circuit !\r" );
		#endif
		return DS18X20_START_FAIL;
	}
}

uint8_t DS18X20_recall_E2( uint8_t id[] )
{
	ow_reset(); //**
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command( DS18X20_RECALL_E2, id );
		// TODO: wait until status is "1" (then eeprom values
		// have been copied). here simple delay to avoid timeout 
		// handling
		_delay_ms(DS18X20_COPYSP_DELAY);
		return DS18X20_OK;
	} 
	else { 
		#ifdef DS18X20_VERBOSE
		uart_puts_P( "DS18X20_recall_E2: Short Circuit !\r" );
		#endif
		return DS18X20_ERROR;
	}
}
#endif
