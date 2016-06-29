/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (C) 2016 IAS, RWTH Aachen University
 *	Moritz Schrey <mschrey@ias.rwth.-aachen.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
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

//sizeof(uint8_t)=1
//sizeof(uint16_t)=2
//sizeof(int)=4
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
static uint8_t eeprom_transfer_byte(int fd, uint8_t data)
{
	int ret;
	uint8_t tx[] = {data, 0x00};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
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
static void eeprom_write_databyte(int fd, unsigned int addr, uint8_t data)
{
	int ret;
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

/*
 * eeprom_read_subreg performs a read operation and returns the specified 
 * bitfield only, after performing shift and mask correction
 */
static uint8_t eeprom_read_subreg(int fd, unsigned int addr, unsigned int mask, unsigned int shift)
{
	uint8_t data = eeprom_read_databyte(fd, addr);
	return (data & mask) >> shift;
}

/*
 * eeprom_write_subreg performs a read-modify-write operation
 * to manipulate individual bitfields within registers
 */
static void eeprom_write_subreg(int fd, unsigned int addr, unsigned int mask, unsigned int shift, unsigned int value)
{
	uint8_t data = eeprom_read_databyte(fd, addr);	
	//printf("vorher:  %.2X\n", data);
	data &= ~mask;               //clear bits
	//printf("mitte:   %.2X\n", data);		
	data |= value << shift;	     //set bits
	//printf("nachher: %.2X\n", data);
	eeprom_write_databyte(fd, addr, data);	
}


static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3NR", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

//	printf("spi mode: %d\n", mode);
//	printf("bits per word: %d\n", bits);
//	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	printf("%.2X ", eeprom_read_databyte(fd, 0x1234));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1235));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1236));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1237));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1238));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1239));
	printf("%.2X ", eeprom_read_databyte(fd, 0x123A));
	printf("%.2X ", eeprom_read_databyte(fd, 0x123B));
	printf("\n");

	eeprom_write_subreg(fd, MYBITFIELD, 2);
	//eeprom_write_subreg(fd, 0x1236, 0x44);

	printf("%.2X ", eeprom_read_databyte(fd, 0x1234));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1235));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1236));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1237));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1238));
	printf("%.2X ", eeprom_read_databyte(fd, 0x1239));
	printf("%.2X ", eeprom_read_databyte(fd, 0x123A));
	printf("%.2X ", eeprom_read_databyte(fd, 0x123B));
	printf("\n");	
	
	printf("%.2X ", eeprom_read_subreg(fd, MYBITFIELD));
	printf("\n");	

	close(fd);

	return ret;
}

