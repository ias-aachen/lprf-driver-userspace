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

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	
	lprf_hw.fd = fd;

///////////////////////////
//    Testing            //
///////////////////////////


	//reset
	//printf("reset\n");
	//write_reg(&lprf_hw, RG_GLOBAL_RESETB, 0x00);
	//write_reg(&lprf_hw, RG_GLOBAL_RESETB, 0x01);		
	
	unsigned int chipid_h, chipid_l;	
	printf("read chip id\n");
	chipid_h = read_reg(&lprf_hw, RG_CHIP_ID_H);
	chipid_l = read_reg(&lprf_hw, RG_CHIP_ID_L);
	printf("chip id: H=0x%0.2X, L=0x%0.2X\n", chipid_h, chipid_l);

	int i, tries = 200;
	int success = 0;
	for(i=0;i<tries;i++) {
		//get random byte
		FILE *filep = fopen("/dev/urandom", "rb");
		uint8_t bytes[] = {0x00, 0x00};
		uint8_t *randbyte = bytes;
		fread(randbyte, 1, 2, filep);
		printf("%3d: randbyte:0x%0.2X 0x%0.2X ... ", i, randbyte[0], randbyte[1]);	
	
		printf("write ...");
		write_reg(&lprf_hw, RG_CHIP_ID_H, randbyte[0]);
		write_reg(&lprf_hw, RG_CHIP_ID_L, randbyte[1]);

		printf("read ... ");
		chipid_h = read_reg(&lprf_hw, RG_CHIP_ID_H);
		chipid_l = read_reg(&lprf_hw, RG_CHIP_ID_L);
		printf("chip id: H=0x%0.2X, L=0x%0.2X => ", chipid_h, chipid_l);
		if((chipid_h == randbyte[0]) && (chipid_l == randbyte[1])) {
			success++;
			printf("PASS!\n");
		} else {
			printf("  FAIL!\n");
		}
			
	}
	printf("Tries:%d  Successful reads:%d  Rate:%5.2f%\n", tries, success, (float)(100*success/tries));
	
	//get chip id
//	unsigned int chip_id = (read_reg(&lprf_hw, RG_CHIP_ID_H) << 8) + read_reg(&lprf_hw, RG_CHIP_ID_L);
//	if (chip_id != 0x01A5)
//		printf("LPRF chip not found! chip_id is %d\n", chip_id);
//		
//	//change chip id to make sure SPI write access works
//	write_reg(&lprf_hw, RG_CHIP_ID_H, 0x01);
//	write_reg(&lprf_hw, RG_CHIP_ID_H, 0xA6);
//	chip_id = (read_reg(&lprf_hw, RG_CHIP_ID_H) << 8) + read_reg(&lprf_hw, RG_CHIP_ID_L);
//	if (chip_id != 0x01A6)
//		printf("SPI write access does not work properly! chip_id is %d\n", chip_id);
//

	//printf("MYBITFIELD: %.2X\n", read_subreg(&eeprom_hw, MYBITFIELD));	
	//print_register_content(&eeprom_hw, 0x1234, 12);

	close(fd);	

	return ret;
}
