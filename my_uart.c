#include <util/delay.h>
#include <avr/io.h>
#include "my_uart.h"

uint8_t retransmit_flag = FALSE;

void USART_Transmit( unsigned char data ) {
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) ) ;
    /* Put data into buffer, sends the data */
    UDR0 = data;
}
/* Send some data to the serial port */
void USART_tx_string( char *data ) {
	while ((*data != '\0')) {
		while (!(UCSR0A & (1 <<UDRE0)));
		UDR0 = *data;
		data++;
	}
}
void USART_Init() {
	/*Set baud rate */
	UBRR0H = (unsigned char)(25>>8); // FIXME, use MYUBRR, not 25 (9600 @ 4mhz)
	UBRR0L = (unsigned char)25;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	return;
	while(1) {
			USART_Transmit('U');
			_delay_ms(10);
			USART_Transmit('\n');
			_delay_ms(10);
	}
}
char getch(void)
{
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}
