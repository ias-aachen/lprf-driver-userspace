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

	uint8_t txbuf0[] = {
			   0x40, 0x00, 0x00, 0x00,  //1 
        	           0x00, 0x00, 0x00, 0x00,  //2 
        	           0x00, 0x00, 0x00, 0x00,  //3 
        	           0x00, 0x00, 0x00, 0x00,  //4
        	           0x00, 0x00, 0x00, 0x00,  //5
        	           0x00, 0x00, 0x00, 0x00,  //6
        	           0x00, 0x00, 0x00, 0x00,  //7
        	           0x00, 0x00, 0x00, 0x00,  //8
        	           0x00, 0x00, 0x00, 0x00,  //9
        	           0x00, 0x00, 0x00, 0x00,  //10 
			   0x00, 0x00, 0x00, 0x00,  //11 
        	           0x00, 0x00, 0x00, 0x00,  //12 
        	           0x00, 0x00, 0x00, 0x00,  //13  
        	           0x00, 0x00, 0x00, 0x00,  //14  
        	           0x00, 0x00, 0x00, 0x00,  //15  
        	           0x00, 0x00, 0x00, 0x00,  //16  
        	           0x00, 0x00, 0x00, 0x00,  //17 
        	           0x00, 0x00, 0x00, 0x00,  //18 
        	           0x00, 0x00, 0x00, 0x00,  //19 
        	           0x00, 0x00, 0x00, 0x00,  //20  
			   0x00, 0x00, 0x00, 0x00,  //21
        	           0x00, 0x00, 0x00, 0x00,  //22
        	           0x00, 0x00, 0x00, 0x00,  //23
        	           0x00, 0x00, 0x00, 0x00,  //24
        	           0x00, 0x00, 0x00, 0x00,  //25
        	           0x00, 0x00, 0x00, 0x00,  //26
        	           0x00, 0x00, 0x00, 0x00,  //27
        	           0x00, 0x00, 0x00, 0x00,  //28
        	           0x00, 0x00, 0x00, 0x00,  //29
   	    	           0x00, 0x00, 0x00, 0x00,  //30
			   0x00, 0x00, 0x00, 0x00,  //31
        	           0x00, 0x00, 0x00, 0x00,  //32
        	           0x00, 0x00, 0x00, 0x00,  //33
        	           0x00, 0x00, 0x00, 0x00,  //34
        	           0x00, 0x00, 0x00, 0x00,  //35
        	           0x00, 0x00, 0x00, 0x00,  //36
        	           0x00, 0x00, 0x00, 0x00,  //37
        	           0x00, 0x00, 0x00, 0x00,  //38
        	           0x00, 0x00, 0x00, 0x00,  //39
        	           0x00, 0x00, 0x00, 0x00,  //40 
};
	uint8_t txbuf1[] = {
			   0xFF, 0xFF, 0xFF, 0xFF,  //1 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //2 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //3 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //4
        	           0xFF, 0xFF, 0xFF, 0xFF,  //5
        	           0xFF, 0xFF, 0xFF, 0xFF,  //6
        	           0xFF, 0xFF, 0xFF, 0xFF,  //7
        	           0xFF, 0xFF, 0xFF, 0xFF,  //8
        	           0xFF, 0xFF, 0xFF, 0xFF,  //9
        	           0xFF, 0xFF, 0xFF, 0xFF,  //10 
			   0xFF, 0xFF, 0xFF, 0xFF,  //11 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //12 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //13  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //14  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //15  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //16  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //17 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //18 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //19 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //20  
			   0xFF, 0xFF, 0xFF, 0xFF,  //21
        	           0xFF, 0xFF, 0xFF, 0xFF,  //22
        	           0xFF, 0xFF, 0xFF, 0xFF,  //23
        	           0xFF, 0xFF, 0xFF, 0xFF,  //24
        	           0xFF, 0xFF, 0xFF, 0xFF,  //25
        	           0xFF, 0xFF, 0xFF, 0xFF,  //26
        	           0xFF, 0xFF, 0xFF, 0xFF,  //27
        	           0xFF, 0xFF, 0xFF, 0xFF,  //28
        	           0xFF, 0xFF, 0xFF, 0xFF,  //29
        	           0xFF, 0xFF, 0xFF, 0xFF,  //30
			   0xFF, 0xFF, 0xFF, 0xFF,  //31
        	           0xFF, 0xFF, 0xFF, 0xFF,  //32
        	           0xFF, 0xFF, 0xFF, 0xFF,  //33
        	           0xFF, 0xFF, 0xFF, 0xFF,  //34
        	           0xFF, 0xFF, 0xFF, 0xFF,  //35
        	           0xFF, 0xFF, 0xFF, 0xFF,  //36
        	           0xFF, 0xFF, 0xFF, 0xFF,  //37
        	           0xFF, 0xFF, 0xFF, 0xFF,  //38
        	           0xFF, 0xFF, 0xFF, 0xFF,  //39
        	           0xFF, 0xFF, 0xFF, 0xFF,  //40 
};


	uint8_t txbuf10[] = {
			   0xFF, 0xFF, 0x20, 0x01,  //1 
        	           0x00, 0x00, 0x00, 0x00,  //2 
        	           0x00, 0x00, 0x00, 0x00,  //3 
        	           0x00, 0x00, 0x00, 0x00,  //4
        	           0x00, 0x00, 0x00, 0x00,  //5
        	           0x00, 0x00, 0x00, 0x00,  //6
        	           0x00, 0x00, 0x00, 0x00,  //7
        	           0x00, 0x00, 0x00, 0x00,  //8
        	           0x00, 0x00, 0x00, 0x00,  //9
        	           0x00, 0x00, 0x00, 0x00,  //10 
			   0xFF, 0xFF, 0xFF, 0xFF,  //11 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //12 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //13  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //14  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //15  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //16  
        	           0xFF, 0xFF, 0xFF, 0xFF,  //17 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //18 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //19 
        	           0xFF, 0xFF, 0xFF, 0xFF,  //20  
			   0x00, 0x00, 0x00, 0x00,  //21
        	           0x00, 0x00, 0x00, 0x00,  //22
        	           0x00, 0x00, 0x00, 0x00,  //23
        	           0x00, 0x00, 0x00, 0x00,  //24
        	           0x00, 0x00, 0x00, 0x00,  //25
        	           0x00, 0x00, 0x00, 0x00,  //26
        	           0x00, 0x00, 0x00, 0x00,  //27
        	           0x00, 0x00, 0x00, 0x00,  //28
        	           0x00, 0x00, 0x00, 0x00,  //29
   	    	           0x00, 0x00, 0x00, 0x00,  //30
			   0x00, 0x00, 0x00, 0x00,  //31
        	           0x00, 0x00, 0x00, 0x00,  //32
        	           0x00, 0x00, 0x00, 0x00,  //33
        	           0x00, 0x00, 0x00, 0x00,  //34
        	           0x00, 0x00, 0x00, 0x00,  //35
        	           0x00, 0x00, 0x00, 0x00,  //36
        	           0x00, 0x00, 0x00, 0x00,  //37
        	           0x00, 0x00, 0x00, 0x00,  //38
        	           0x00, 0x00, 0x00, 0x00,  //39
        	           0x00, 0x00, 0x00, 0x00,  //40 
};

uint8_t txbuf_tda_mod[] = {
			0xAA, 0xAA, 0xAA, 0xAA, //4
			0xAA, 0xAA, 0xAA, 0xAA, //8
			0xAA, 0xAA, 0xAA, 0xAA, //12
			0xAA, 0xAA, 0xAA, 0xAA, //16
			0xAA, 0xAA, 0xAA, 0xAA, //20
			0xAA, 0xAA, 0xAA, 0xAA, //24
			0xAA, 0xAA, 0xA9, 0x99, //28
			0xAA, 0xAA, 0xA9, 0xA9, //32
			0xA6, 0xA6, 0xA5, 0xA5, //36
			0x9A, 0x9A, 0x99, 0x99, //40
			0x96, 0x96, 0x95, 0x95, //44
			0x6A, 0x6A, 0x69, 0x69, //48
			0x66, 0x66, 0xAA, 0xAA, //52
			0xA9, 0xA9, 0xA6, 0xA6, //56
			0xA5, 0xA5, 0x9A, 0x9A, //60
			0x99, 0x99, 0x96, 0x96, //64
			0x95, 0x95, 0x6A, 0x6A, //68
			0x69, 0x69, 0x66, 0x66  //72bytes = 576bits
};

uint8_t txbuf_tda_unmod[] = {
			0x00, 0x00, 0x00, 0x00, //4
			0x00, 0x00, 0x00, 0x00, //8
			0x00, 0x00, 0x00, 0x00, //12
			0x00, 0x15, 0x00, 0x11, //16
			0x22, 0x33, 0x44, 0x55, //20
			0x66, 0x77, 0x88, 0x99, //24
			0xAA, 0x00, 0x11, 0x22, //28
			0x33, 0x44, 0x55, 0x66, //32
			0x77, 0x88, 0x99, 0xAA  //36bytes = 288bits
};


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
//    TX24 Testcase      //
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
		write_subreg(&lprf_hw, SR_WAKEUPONSPI, 0);          //disable wakeup modes	
		write_subreg(&lprf_hw, SR_TX_CHAN_INT, 112);        //Set PLL divider, write PLL_CHN through statemachine,    //1792MHz an PLL_OUT
		write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_H, 0);       //for 2.4GHz output: sliding IF: 
		write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_M, 0);       //f_RF = 0.75 * f_vco = 0.75 * 32MHz * PLL_CHN
		write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_L, 0);
		write_subreg(&lprf_hw, SR_PLL_VCO_TUNE, 152);       //set VCO tune word for 1792MHz an PLL_OUT

		printf("configure modulation\n");
		write_subreg(&lprf_hw, SR_TX_ON_CHIP_MOD, 1);       //enable on-chip modulation
		write_subreg(&lprf_hw, SR_PLL_MOD_EN, 1);           //enable modulation of PLL
		write_subreg(&lprf_hw, SR_PLL_MOD_DATA_RATE, 1);    //set tx data rate to <500kBit/s
		write_subreg(&lprf_hw, SR_TX_ON_CHIP_MOD_SP, 9);    //set TX data rate (3: 250kHz; 8: 20kHz; 9: 10kHz)
		write_subreg(&lprf_hw, SR_PLL_MOD_FREQ_DEV, 4);     //set frequency deviation (4: 128kHz; 8: 256kHz at PLL_OUT)
		write_subreg(&lprf_hw, SR_INVERT_FIFO_CLK, 0);      //invert FIFO clock to enable R/W operation to the fifo
		write_subreg(&lprf_hw, SR_TX_SPI_FIFO_OUT_EN, 1);   //enable FIFO -> txpath
		write_subreg(&lprf_hw, SR_PLL_TPM_COMP_EN, 0); //disable two point modulation compensation loop

		printf("enable state machine\n");
		write_subreg(&lprf_hw, SR_SM_EN, 1);          //enable state machine
		write_subreg(&lprf_hw, SR_TX_MODE, 0);        //set TX mode to 2.4GHz

		printf("set waiting times to max\n");
		write_subreg(&lprf_hw, SR_POWER_TX_TIME, 256);
		write_subreg(&lprf_hw, SR_PLL_PON_TIME, 256);
		write_subreg(&lprf_hw, SR_TX_TIME, 256);

		printf("write frame            ");
		uint8_t payload_len = 36;
		uint8_t *payload_ups = (uint8_t*) malloc(sizeof(uint8_t) * payload_len * 2);
		uint8_t *payload_mod = (uint8_t*) malloc(sizeof(uint8_t) * payload_len * 2);
		upsample(txbuf_tda_unmod, payload_ups, payload_len);
		manchester_encode(payload_ups, payload_mod, payload_len*2);
		write_frame(&lprf_hw, payload_mod, payload_len*2);   
		printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));

		printf("enable FIFO mode       ");
		write_subreg(&lprf_hw, SR_FIFO_MODE_EN, 1);   //enable FIFO mode
		printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));

		printf("change state to TX     ");
		write_subreg(&lprf_hw, SR_SM_COMMAND, CMD_TX);   //change state to TX
		write_subreg(&lprf_hw, SR_CTRL_CLK_ADC, 0); //enable CLKOUT at TX_START
		printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));

		int i;
		uint8_t val;
		for(i = 0; i < 100000; i++){
			val = read_reg(&lprf_hw, RG_SM_STATE);	
			if(!(val & 4)){	
				write_subreg(&lprf_hw, SR_SM_COMMAND, CMD_NONE);  //reset CMD bitfield
				write_subreg(&lprf_hw, SR_CTRL_CLK_ADC, 1);       //disable CLKOUT at TX_STOP
				printf("receiving...           ");
				printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X   i=%d\n", val, read_reg(&lprf_hw, RG_SM_FIFO), i);	
				break;
			}
		}

		printf("                       SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X   i=%d\n", val, read_reg(&lprf_hw, RG_SM_FIFO), i); 
	}
	close(fd);

	return ret;
}

