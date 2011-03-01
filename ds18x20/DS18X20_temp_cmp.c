
/* compare temperature values (full celsius only)
   returns -1 if param-pair1 < param-pair2 
            0 if == 
			1 if >    */
int8_t DS18X20_temp_cmp(uint8_t subzero1, uint16_t cel1, 
	uint8_t subzero2, uint16_t cel2)
{
	int16_t t1 = (subzero1) ? (cel1*(-1)) : (cel1);
	int16_t t2 = (subzero2) ? (cel2*(-1)) : (cel2);
	
	if (t1<t2) return -1;
	if (t1>t2) return 1;
	return 0;
}
