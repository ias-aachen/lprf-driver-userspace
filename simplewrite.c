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
///usr/include/linux/spi/spidev.h
///home/pi/linux/include/config/spi/spidev.h
///home/pi/linux/include/config/bcm2708/spidev.h
///home/pi/linux/include/uapi/linux/spi/spidev.h
#include <byteswap.h>

#include </home/pi/lprf/lprf-driver/lprf_registers.h>

#include </home/pi/spidev/lprf_driver.c>


int main(int argc, char *argv[])
{
	if (argc != 3) {
		printf("wrong number of arguments! Usage: %s <0xaddress> <0xdata>\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	uint8_t addr;
	uint8_t data;
	sscanf(argv[2], "0x%X", &data);
	sscanf(argv[1], "0x%X", &addr);
	//printf("address is %s (0x%0.2X = %d)\n", argv[1], addr, addr);
	//printf("data is    %s (0x%0.2X = %d)\n", argv[2], data, data);

	int ret = 0;
	int fd;

	fd = open("/dev/spidev0.0", O_RDWR);
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

	//printf("spi mode: %d\n", mode);
	//printf("bits per word: %d\n", bits);
	//printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	
	lprf_hw.fd = fd;

	write_reg(&lprf_hw, addr, data);
	uint8_t regdata = read_reg(&lprf_hw, addr);
	if (data != regdata)
		printf("FAILED! read from 0x%0.2X returned 0x%02X instead of 0x%0.2X\n", addr, regdata, data);

	close(fd);

	return ret;
}

