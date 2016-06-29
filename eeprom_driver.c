/*
 * SPI EEPROM driver (Microchip specifically 25LC640) (using spidev driver)
 *
 * Copyright (c) 2016  Moritz Schrey
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>   //for memcpy
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include </home/pi/spidev/lprf_registers.h>
#include <byteswap.h>
#include </home/pi/spidev/spi_hw_abstraction.h>


#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define CMD_READ     0x03   
#define CMD_WRITE    0x02
#define CMD_WRDI     0x04
#define CMD_WREN     0x06
#define CMD_RDSR     0x05
#define CMD_WRSR     0x01
#define cmd_len      1
#define addr_len     2
#define SR_WIP       0x01  //Status register bits   
#define SR_WEL       0x02  //Status register bits
#define SR_BP0       0x04  //Status register bits
#define SR_BP1       0x08  //Status register bits
#define SR_WPEN      0x80  //Status register bits

#define MYBITFIELD   0x1236, 0x78, 3

//sizeof(uint8_t)     =1
//sizeof(uint16_t)    =2
//sizeof(int)         =4
//sizeof(unsigned int)=4

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev1.1";
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay;

/* 
 * eeprom_transfer_byte takes care of single-byte input/output
 * 
 * Note that CS gets asserted after every single-byte transfer.
 * Therefore, eeprom_transfer_byte() is not suitable for accessing
 * data memory cells, but for I/O to STATUS register and for
 * issuing commands (WREN, WRDI...)
 */
//static uint8_t eeprom_transfer_byte(int fd, uint8_t data)
static uint8_t eeprom_transfer_byte(int fd, unsigned int data)
{
	int ret;
	uint8_t tx[] = {data, 0x00};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	data = (uint8_t)data;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,		
		.bits_per_word = bits,
	};
	
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't transfer byte");		
		
	for (ret = 0; ret < ARRAY_SIZE(tx); ret++) {
//		if (!(ret % 6))
//			puts("");
//		printf("%.2X ", rx[ret]);
	}
//	puts("");
	return rx[1];
}


static void eeprom_setWREN(int fd)
{
	//printf("setWREN\n");
	eeprom_transfer_byte(fd, CMD_WREN);
}

static uint8_t eeprom_get_status(int fd)
{
	uint8_t status = eeprom_transfer_byte(fd, CMD_RDSR); 
	//printf("get_status: %.2X\n", status);
	return status;
}

/* eeprom_write_databyte writes user data into eeprom memory
 *
 * Note that eeprom write accesses constitute multi-byte
 * statements during which CS needs to be held low. 
 * eeprom_transfer_byte is therefore not a suitable function
 */
//static void eeprom_write_databyte(int fd, unsigned int addr, uint8_t data)
static void eeprom_write_databyte(int fd, unsigned int addr, unsigned int data)
{
	int ret;
	data = (uint8_t)data;
	uint8_t * tx = (uint8_t*)malloc((cmd_len + addr_len + 1) * sizeof(uint8_t));
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	uint8_t addr_L = (uint8_t)(addr & 0x00FF);        //fix endianness
	uint8_t addr_H = (uint8_t)((addr & 0xFF00)>>8);	
	tx[0] = CMD_WRITE;
	tx[1] = addr_H;
	tx[2] = addr_L;
	tx[3] = data;
	
	eeprom_setWREN(fd);

	uint8_t status;
	status = eeprom_get_status(fd);
	while(status & SR_WIP)
		status = eeprom_get_status(fd);

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
}

/* eeprom_read_databyte reads user data from eeprom memory
 *
 * Note that eeprom read accesses constitute multi-byte
 * statements during which CS needs to be held low. 
 * eeprom_transfer_byte is therefore not a suitable function
 */
static uint8_t eeprom_read_databyte(int fd, unsigned int addr)
{
	int ret;
	uint8_t * tx = (uint8_t*)malloc((cmd_len + addr_len + 1) * sizeof(uint8_t));
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	uint8_t addr_L = (uint8_t)(addr & 0x00FF);
	uint8_t addr_H = (uint8_t)((addr & 0xFF00)>>8);	
	tx[0] = CMD_READ;
	tx[1] = addr_H;
	tx[2] = addr_L;
	tx[3] = 0x00;

	uint8_t status;
	status = eeprom_get_status(fd);
	while(status & SR_WIP)
		status = eeprom_get_status(fd);
	
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
		
	return rx[3];
}


struct spi_hw eeprom_hw = {
	.fd = 0,
	.transfer_byte = eeprom_transfer_byte,
	.write_databyte = eeprom_write_databyte,
	.read_databyte = eeprom_read_databyte,
};


//int main(int argc, char *argv[])
//{
//	int ret = 0;
//	int fd;
//
//	fd = open("/dev/spidev0.0", O_RDWR);
//	if (fd < 0)
//		pabort("can't open device");
//	
//	eeprom_hw.fd = fd;
//	
//	print_register_content(&eeprom_hw, 0x1234, 12);
//	printf("MYBITFIELD: %.2X\n", read_subreg(&eeprom_hw, MYBITFIELD));
//	write_subreg(&eeprom_hw, MYBITFIELD, 6);
//	printf("MYBITFIELD: %.2X\n", read_subreg(&eeprom_hw, MYBITFIELD));
//	print_register_content(&eeprom_hw, 0x1234, 12);
//}


