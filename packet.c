#include <string.h>
#include <stdarg.h>
#include "packet.h"
#include "my_uart.h"

void start_packet(xbee_packet *pkt,uint8_t type) {
	pkt->length_l = 0;
	//packet_append_byte(pkt,type);
}
int packet_append_string(xbee_packet *pkt,char * string) {
	int size = strlen(string);
	memcpy(&(pkt->data[pkt->length_l]), string, size);
	pkt->length_l = pkt->length_l + size;
	return size;
}
int packet_append_string_P(xbee_packet *pkt,char * string) {
	int size = strlen_P(string);
	strcpy_P(&(pkt->data[pkt->length_l]),string);
	pkt->length_l = pkt->length_l + size;
}
void packet_end(xbee_packet *pkt) {
	uint8_t x;
	uint8_t checksum = 0;
	for (x = 0; x < pkt->length_l; x++) {
		checksum += pkt->data[x];
	}
	//checksum = 0xff - checksum;
	//pkt->data[pkt->length_l] = checksum;
}
void inline packet_tx_byte(uint8_t byte) { // size 22(19)
	/*switch (byte) {
	case 0x7e:
	case 0x7d:
	case 0x11:
	case 0x13:
		USART_Transmit(0x7d);
		byte = 0x20 ^ byte;
	}*/
	USART_Transmit(byte);
}
void packet_send(xbee_packet *pkt) {
	//USART_Transmit(0x7e);
	uint8_t *hack;
	hack = (uint8_t*) &(pkt->data);
	int x;
	//USART_Transmit(0); // length high byte
	for (x = 0; x <= (pkt->length_l + 1); x++) {
		packet_tx_byte(hack[x]);
	}
}
void packet_end_send(xbee_packet *pkt) {
	packet_end(pkt);
	packet_send(pkt);
}
void packet_append_byte(xbee_packet *pkt,uint8_t byte) {
	pkt->data[pkt->length_l] = byte;
	pkt->length_l++;
}
void start_tx_packet(xbee_packet *pkt,uint16_t addr,uint8_t options) { // size 21 (20)
	pkt->length_l = 0;
	//pkt->data[0] = 0x01;
	//pkt->data[1] = 0x00;// frameid
	//pkt->data[2] = (addr >> 8) & 0xff;// dest h
	//pkt->data[3] = addr & 0xff;// dest l
	//pkt->data[4] = options;
}
void send_string(char* string) {
	xbee_packet pkt;
	start_tx_packet(&pkt,0,0);
	packet_append_string(&pkt,string);
	packet_end_send(&pkt);
}
void send_string_P(char* string) {
	xbee_packet pkt;
	start_tx_packet(&pkt,0,0);
	packet_append_string_P(&pkt,string);
	packet_end_send(&pkt);
}
void packet_printf(xbee_packet *pkt, char *format, ...) {
	va_list ap;
	va_start(ap,format);
	int size = vsnprintf(&(pkt->data[pkt->length_l]), 100 - pkt->length_l, format,ap);
	va_end(ap);
	pkt->length_l = pkt->length_l + size;
}
void packet_printf_P(xbee_packet *pkt, char *format, ...) {
	va_list ap;
	va_start(ap,format);
	int size = vsnprintf_P(&(pkt->data[pkt->length_l]), 100 - pkt->length_l, format,ap);
	va_end(ap);
	pkt->length_l = pkt->length_l + size;
}
xbee_packet main_packet;
