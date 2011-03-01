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
void packet_end(xbee_packet *pkt) {
	int x;
	uint8_t checksum = 0;
	for (x = 0; x < pkt->length_l; x++) {
		checksum += pkt->data[x];
	}
	checksum = 0xff - checksum;
	pkt->data[pkt->length_l] = checksum;
}
void packet_send(xbee_packet *pkt) {
	USART_Transmit(pkt->start_byte);
	USART_Transmit(pkt->length_h);
	USART_Transmit(pkt->length_l);
	int x;
	for (x = 0; x <= pkt->length_l; x++) {
		USART_Transmit(pkt->data[x]);
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
