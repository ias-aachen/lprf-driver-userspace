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
//  DEEPSLEEP USE CASE   //
///////////////////////////


	//reset
	write_reg(&lprf_hw, RG_GLOBAL_RESETB, 0x00);
	write_reg(&lprf_hw, RG_GLOBAL_RESETB, 0x01);		

	//get chip id
	unsigned int chip_id = (read_reg(&lprf_hw, RG_CHIP_ID_H) << 8) + read_reg(&lprf_hw, RG_CHIP_ID_L);
	if (chip_id != 0x1A51)
		printf("LPRF chip not found! CHIP_ID = 0x%0.4X\n", chip_id);
		
	//change chip id to make sure SPI write access works
	write_reg(&lprf_hw, RG_CHIP_ID_H, 0x1A);
	write_reg(&lprf_hw, RG_CHIP_ID_L, 0x53);
	chip_id = (read_reg(&lprf_hw, RG_CHIP_ID_H) << 8) + read_reg(&lprf_hw, RG_CHIP_ID_L);
	if (chip_id != 0x1A53)
		printf("ERROR: SPI write access does not work properly! CHIP_ID = 0x%0.4X\n", chip_id);
	else {
		//printf("SPI write access sucessfully established. CHIP_ID = 0x%0.4X\n", chip_id);
	}
	write_reg(&lprf_hw, RG_CHIP_ID_H, 0x1A);
	write_reg(&lprf_hw, RG_CHIP_ID_L, 0x51);
	
	
	printf("configure clock domains\n");
	//enable CLKREF -> CLKOUT and CLKPLL path
	write_subreg(&lprf_hw, SR_CTRL_CLK_CDE_OSC, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_CDE_PAD, 1);
	write_subreg(&lprf_hw, SR_CTRL_CLK_DIG_OSC, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_DIG_PAD, 1);
	write_subreg(&lprf_hw, SR_CTRL_CLK_PLL_OSC, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_PLL_PAD, 1);
	write_subreg(&lprf_hw, SR_CTRL_CDE_ENABLE, 1);
	write_subreg(&lprf_hw, SR_CTRL_C3X_ENABLE, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_FALLB, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_ADC, 0);	
	write_subreg(&lprf_hw, SR_CTRL_CLK_IREF, 6);   //enable current for clock delay (CDE)

	printf("enable and configure LDOs\n");
	write_subreg(&lprf_hw, SR_LDO_A, 1);           //Enable all LDOs
	write_subreg(&lprf_hw, SR_LDO_PLL, 1);
	write_subreg(&lprf_hw, SR_LDO_VCO, 1);
	write_subreg(&lprf_hw, SR_LDO_TX24, 1);
	write_subreg(&lprf_hw, SR_LDO_D_VOUT, 15);     //configure LDOs
	write_subreg(&lprf_hw, SR_LDO_PLL_VOUT, 31);   //1.74V
	write_subreg(&lprf_hw, SR_LDO_VCO_VOUT, 31);   //1.76V
	write_subreg(&lprf_hw, SR_LDO_TX24_VOUT, 25);  //1.16V

	write_subreg(&lprf_hw, SR_PLL_BUFFER_EN, 1);
	//write_subreg(&lprf_hw, SR_IREF_PLL_CTRLB, 0);
	write_subreg(&lprf_hw, SR_PLL_EN, 1);

	uint8_t sm = 1;
	if(sm == 0) {    	//set channel (no State machine)
		write_subreg(&lprf_hw, SR_PLL_CHN_INT, 112);
		write_subreg(&lprf_hw, SR_PLL_CHN_FRAC_H, 0);
		write_subreg(&lprf_hw, SR_PLL_CHN_FRAC_M, 0);
		write_subreg(&lprf_hw, SR_PLL_CHN_FRAC_L, 0);
		write_subreg(&lprf_hw, SR_PLL_VCO_TUNE, 152); //set VCO tune word for 1696MHz an PLL_OUT   
		write_subreg(&lprf_hw, SR_IREF_PLL_CTRLB, 0);
		write_subreg(&lprf_hw, SR_PLL_EN, 1);
	} else {
		//reset PLL
		write_subreg(&lprf_hw, SR_PLL_RESETB, 0);
		write_subreg(&lprf_hw, SR_PLL_RESETB, 1);
		write_subreg(&lprf_hw, SR_CTRL_CLK_ADC, 1); //disable CLKOUT
	
//===============change to state machine operation=========================
		printf("configure center freq\n");
		write_subreg(&lprf_hw, SR_WAKEUPONSPI, 1);          //enable wakeup modes	
		write_subreg(&lprf_hw, SR_TX_CHAN_INT, 112);        //Set PLL divider, write PLL_CHN through statemachine,    //1792MHz an PLL_OUT
		write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_H, 0);       //for 2.4GHz output: sliding IF: 
		write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_M, 0);       //f_RF = 0.75 * f_vco = 0.75 * 32MHz * PLL_CHN
		write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_L, 0);
		write_subreg(&lprf_hw, SR_PLL_VCO_TUNE, 152);       //set VCO tune word for 1792MHz an PLL_OUT

		printf("enable state machine\n");
		write_subreg(&lprf_hw, SR_SM_EN, 1);          //enable state machine
		write_subreg(&lprf_hw, SR_TX_MODE, 0);        //set TX mode to 2.4GHz

		printf("set waiting times to max\n");
		write_subreg(&lprf_hw, SR_POWER_TX_TIME, 256);
		write_subreg(&lprf_hw, SR_PLL_PON_TIME, 256);
		write_subreg(&lprf_hw, SR_TX_TIME, 256);

		printf("change state to DEEPSLEEP\n");
		init_gpio();
		overwrite_CS(0);
		write_subreg(&lprf_hw, SR_SM_COMMAND, CMD_DEEPSLEEP);   //change state to DEEPSLEEP

	}
	close(fd);

	return ret;
}

