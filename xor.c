#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

void byte_to_binary(char *target, int x)
{
    int index = 1;

    int z;
    for (z = 7; z >= 0; z--)
    {
        if (x&index)
            target[z] = '1';
        else
            target[z] = '0';
            
        index <<= 1;
    }
}


int main(void)
{
    uint8_t byte1 = 0b11001100;
    uint8_t byte2 = 0b00111100;
    uint8_t clk = 0b01010101;
    
    uint8_t byte_out1 = byte1 ^ clk;
    uint8_t byte_out2 = byte2 ^ clk;
    
    char myout1[] = "iiiiiiii";
    char myout2[] = "iiiiiiii";        
        
    byte_to_binary(myout1, byte1);
    byte_to_binary(myout2, byte2);    
    printf("byte in : %s %s\n", myout1, myout2);
    byte_to_binary(myout1, clk);
    printf("clk     : %s %s\n", myout1, myout1);
    byte_to_binary(myout1, byte_out1);
    byte_to_binary(myout2, byte_out2);        
    printf("byte out: %s %s\n", myout1, myout2);

    uint8_t byte_back1 = byte_out1 ^ clk;
    uint8_t byte_back2 = byte_out2 ^ clk;
    byte_to_binary(myout1, byte_back1);
    byte_to_binary(myout2, byte_back2);        
    printf("byteback: %s %s\n", myout1, myout2);
    byte_to_binary(myout1, byte1);
    byte_to_binary(myout2, byte2);    
    printf("byte in : %s %s\n", myout1, myout2);
    
}
