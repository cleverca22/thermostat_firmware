#define TRUE    0
#define FALSE   1
#define MAX_CHARACTER_WAIT      15 //10 works. 20 works. 5 throws all sorts of retries, but will work.
#define MAX_WAIT_IN_CYCLES ( ((MAX_CHARACTER_WAIT * 8) * F_CPU) / BAUD )
#define MYUBRR F_CPU/16/BAUD-1

void USART_Transmit( unsigned char data );
void USART_Init();
