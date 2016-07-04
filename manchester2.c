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


upsample(uint8_t * data_in, uint8_t * data_out, uint8_t len)
{
	int i, j, data_ups_pointer;
	data_ups_pointer = 0;
	uint16_t tempout;
	for(i=0; i<len;i++) {
		tempout = 0;
		for(j=0; j<8;j++) {
			if(data_in[i] & (1<<j)) {
				tempout += (3<<(j*2));
			}
		}
		//write out tempout to data_out and correct endianness 
 		data_out[data_ups_pointer] = ((tempout & 0xFF00) >> 8); 
    	data_out[data_ups_pointer+1] = (tempout & 0x00FF);
		data_ups_pointer += 2;
	}
}

manchester_encode(uint8_t * data_in, uint8_t * data_out, uint8_t len)
{
	int i;
	for(i=0; i<len; i++) 
		data_out[i] = data_in[i] ^ (0xAA);
}

int main(void)
{
	//uint8_t payload[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA};
    uint8_t payload[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                         0x00, 0x15, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 
                         0xAA, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA};    
	uint8_t payload_len = 36;
	int i;
	uint8_t *payload_ups = (uint8_t*) malloc(sizeof(uint8_t) * payload_len * 2);
	uint8_t *payload_mod = (uint8_t*) malloc(sizeof(uint8_t) * payload_len * 2);

	upsample(payload, payload_ups, payload_len);
	manchester_encode(payload_ups, payload_mod, payload_len*2);

	for(i=0;i<payload_len;i++) {
		printf("0x%02X: %s ", payload[i], print_binary(payload[i]));
		printf("-> %s", print_binary(payload_ups[2*i]));
		printf(" %s\n", print_binary(payload_ups[2*i+1]));
	}
	printf("\n");




	for(i=0;i<payload_len*2;i++) 
		printf("%02X", payload_mod[i]);
	printf("\n");
}

