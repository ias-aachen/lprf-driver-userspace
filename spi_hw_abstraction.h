#ifndef SPI_HW_ABSTRACTION
#define SPI_HW_ABSTRACTION

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>   //for memcpy
#include <sys/ioctl.h>
#include <stdbool.h>  //for false&true
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "../lprf-driver/lprf_registers.h"
#include <byteswap.h>

struct spi_hw {
	int fd;      //file descriptor
	int MAX_FRAME_LEN;
	uint8_t (*transfer_byte)(int fd, unsigned int data);
	void    (*write_databyte)(int fd, unsigned int addr, unsigned int data);
	uint8_t (*read_databyte)(int fd, unsigned int addr);
	void    (*frame_write)(int fd, uint8_t *txbuf, uint8_t len);
	uint8_t (*frame_read)(int fd, uint8_t *rxbuf);
};


int register_hw(struct spi_hw *hw);
uint8_t read_reg(struct spi_hw *hw, unsigned int addr);
void write_reg(struct spi_hw *hw, unsigned int addr, unsigned int data);
uint8_t read_subreg(struct spi_hw *hw, unsigned int addr, unsigned int mask, unsigned int shift);
void write_subreg(struct spi_hw *hw, unsigned int addr, unsigned int mask, unsigned int shift, unsigned int value);
uint8_t read_frame(struct spi_hw *hw, uint8_t *rxbuf);
void write_frame(struct spi_hw *hw, uint8_t *txbuf, uint8_t len);
void print_register_content(struct spi_hw *hw, unsigned int addr_start, unsigned int len);

#endif // SPI_HW_ABSTRACTION
