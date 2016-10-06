/*
 * Abstraction layer for SPI hardware
 * this mostly defines the struct spi_hw
 * 
 * transfer_byte is only used to send&receive single bytes
 *
 *
 *
 */
 
#include "spi_hw_abstraction.h"
 
 /*
struct spi_hw {
	int fd;      //file descriptor
	int MAX_FRAME_LEN;
	uint8_t (*transfer_byte)(int fd, unsigned int data);
	void    (*write_databyte)(int fd, unsigned int addr, unsigned int data);
	uint8_t (*read_databyte)(int fd, unsigned int addr);
	void    (*frame_write)(int fd, uint8_t *txbuf, uint8_t len);
	uint8_t (*frame_read)(int fd, uint8_t *rxbuf);
};
*/

int register_hw(struct spi_hw *hw)
{

}

/*
 * read_reg performs a read operation 
 */

uint8_t read_reg(struct spi_hw *hw, unsigned int addr)
{
	uint8_t data = hw->read_databyte(hw->fd, addr);
	return data;
}

/*
 * write_reg performs a write operation
 */

void write_reg(struct spi_hw *hw, unsigned int addr, unsigned int data)
{
	hw->write_databyte(hw->fd, addr, data);
}


/*
 * read_subreg performs a read operation and returns the specified 
 * bitfield only, after performing shift and mask correction
 */
uint8_t read_subreg(struct spi_hw *hw, unsigned int addr, unsigned int mask, unsigned int shift)
{
	uint8_t data = hw->read_databyte(hw->fd, addr);
	return (data & mask) >> shift;
}


/*
 * write_subreg performs a read-modify-write operation
 * to manipulate individual bitfields within registers
 */
void write_subreg(struct spi_hw *hw, unsigned int addr, unsigned int mask, unsigned int shift, unsigned int value)
{
	uint8_t data = hw->read_databyte(hw->fd, addr);	
	data &= ~mask;               //clear bits
	data |= value << shift;	     //set bits
	hw->write_databyte(hw->fd, addr, data);	
	//check contents
	uint8_t datacheck = hw->read_databyte(hw->fd, addr);
	if(data != datacheck)
		printf("ERROR: write confirming failed\n");
}

uint8_t read_frame(struct spi_hw *hw, uint8_t *rxbuf)
{
	uint8_t len = hw->frame_read(hw->fd, rxbuf);
	return len;
}


void write_frame(struct spi_hw *hw, uint8_t *txbuf, uint8_t len)
{
	hw->frame_write(hw->fd, txbuf, len);
}


void print_register_content(struct spi_hw *hw, unsigned int addr_start, unsigned int len)
{
	int i;
	for (i=0; i<len; i++) {
		printf("%.2X ", hw->read_databyte(hw->fd, addr_start+i));
	}
	printf("\n");
}

