// Test bench for reverse_bit_order()

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

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
	uint8_t myint = 15;
	uint8_t newint = reverse_bit_order(myint);
	printf("old: %d, new: %d\n", myint, newint);
	printf("%d\n", 128+64+32+16);
}
