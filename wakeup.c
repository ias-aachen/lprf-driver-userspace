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
#include <byteswap.h>

#include </home/pi/spidev/lprf_registers.h>
#include </home/pi/spidev/lprf_driver.c>


int main(int argc, char *argv[])
{
	if (argc != 2) {
		printf("wrong number of arguments! Usage: %s <0xaddress>\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	uint8_t addr;
	sscanf(argv[1], "0x%X", &addr);

	int ret = 0;
	int fd_spi;
	FILE *fd_gpio;

	fd_gpio = fopen("/sys/class/gpio/gpio24/value", "w");
	if (fd_gpio == NULL) {
		perror("GPIO24 is not set up properly!\n");
		exit(EXIT_FAILURE);
	} else
		fclose(fd_gpio);
	overwrite_CS(1);

	fd_spi = open("/dev/spidev0.0", O_RDWR);
	if (fd_spi < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd_spi, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");


	

	lprf_hw.fd = fd_spi;
	int i;
	uint8_t regdata = read_reg(&lprf_hw, 4);   //read 0x04 (CHIP_ID_L)
	for (i=0;i<10;i++) {
		printf("read from 0x%0.2X returned 0x%02X\n", addr, regdata);		
		if(regdata != 0x51)
			regdata = read_reg(&lprf_hw, 4);
	}


	printf("change state to SLEEP\n");
	write_subreg(&lprf_hw, SR_SM_COMMAND, CMD_SLEEP);   //change state to SLEEP

	close(fd_spi);

	return ret;
}

