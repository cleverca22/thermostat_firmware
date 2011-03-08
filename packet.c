#include <string.h>
#include "packet.h"
#include "my_uart.h"

void start_packet(xbee_packet *pkt,uint8_t type) {
	pkt->start_byte = 0x7e;
	pkt->length_h = 0;
	pkt->length_l = 0;
	packet_append_byte(pkt,type);
}
int packet_append_string(xbee_packet *pkt,char * string) {
	int size = strlen(string);
	int x,y;
	y = 0;
	for (x = pkt->length_l; x < (pkt->length_l + size); x++) {
		pkt->data[x] = string[y];
		y++;
	}
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
	checksum = 0xff - checksum;
	pkt->data[pkt->length_l] = checksum;
}
void inline packet_tx_byte(uint8_t byte) { // size 22(19)
	switch (byte) {
	case 0x7e:
	case 0x7d:
	case 0x11:
	case 0x13:
		USART_Transmit(0x7d);
		byte = 0x20 ^ byte;
	}
	USART_Transmit(byte);
}
void packet_send(xbee_packet *pkt) {
	USART_Transmit(pkt->start_byte);
	uint8_t *hack;
	hack = &(pkt->length_h);
	int x;
	for (x = 0; x <= (pkt->length_l + 2); x++) {
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
void start_tx_packet(xbee_packet *pkt,uint16_t addr,uint8_t options) {
	start_packet(pkt,0x01);
	packet_append_byte(pkt,0x00);// frameid
	packet_append_byte(pkt,(addr << 8) && 0xff);// dest h
	packet_append_byte(pkt,addr && 0xff);// dest l
	packet_append_byte(pkt,options);
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

xbee_packet main_packet;
