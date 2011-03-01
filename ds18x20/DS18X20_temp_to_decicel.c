#include "ds18x20.h"

/* converts to decicelsius
   input is ouput from meas_to_cel
   returns absolute value of temperatur in decicelsius
	i.e.: sz=0, c=28, frac=15 returns 289 (=28.9°C)
0	0	0	
1	625	625	1
2	1250	250	
3	1875	875	3
4	2500	500	4
5	3125	125	
6	3750	750	6
7	4375	375	
8	5000	0	
9	5625	625	9
10	6250	250	
11	6875	875	11
12	7500	500	12
13	8125	125	
14	8750	750	14
15	9375	375	*/
uint16_t DS18X20_temp_to_decicel(uint8_t subzero, uint8_t cel, 
	uint8_t cel_frac_bits)
{
	uint16_t h;
	uint8_t  i;
	uint8_t need_rounding[] = { 1, 3, 4, 6, 9, 11, 12, 14 };
	
	h = cel_frac_bits*DS18X20_FRACCONV/1000;
	h += cel*10;
	if (!subzero) {
		for (i=0; i<sizeof(need_rounding); i++) {
			if ( cel_frac_bits == need_rounding[i] ) {
				h++;
				break;
			}
		}
	}
	return h;
}
