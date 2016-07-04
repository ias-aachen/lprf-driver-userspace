// Test bench for reverse_bit_order()

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

const char * print_binary(uint8_t data)
{
	static char mychar[9];
	mychar[8] = '\0';
	int i;
	for(i=0;i<8;i++) {
		if(data & (1<<i))
			mychar[7-i] = '1';
		else
			mychar[7-i] = '0';
	}
	return mychar;	
}


uint8_t reverse_bit_order(uint8_t source)
{
	int i;
	uint8_t dest = 0;
	for(i=0; i<8; i++) 
		if(source & (1<<i)) 
			dest += (1<<(7-i));
	return dest;
}

int main(void)
{
	int i;
	uint8_t payload[] = {0x18, 0x80, 0x81, 0x01};
	uint8_t * payload_new = (uint8_t *)malloc(sizeof(uint8_t) * 4);
	for(i=0;i<4;i++) {
		payload_new[i] = reverse_bit_order(payload[i]);
	}

	for(i=0;i<4;i++) {
		printf("%d: old=0x%02X (%s)", i, payload[i],     print_binary(payload[i]));
		printf(" -> new=0x%02X (%s)\n",  payload_new[i], print_binary(payload_new[i]));
	}
}
