#include <avr/io.h>

typedef struct xbee_packet {
	uint8_t length_l;
	char data[101];
} xbee_packet;

void start_packet(xbee_packet *pkt,uint8_t type);
int packet_append_string(xbee_packet *pkt,char * string);
int packet_append_string_P(xbee_packet *pkt,char * string);
void packet_end_send(xbee_packet *pkt);
void packet_append_byte(xbee_packet *pkt,uint8_t byte);
void start_tx_packet(xbee_packet *pkt,uint16_t addr,uint8_t options);
void send_string(char* string);
void send_string_P(char* string);

extern xbee_packet main_packet;
