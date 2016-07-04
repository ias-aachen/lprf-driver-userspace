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
	
	
////	//when internal oscillator works ok:
////	write_subreg(&lprf_hw, SR_CTRL_CLK_DIG_OSC, 1);
////	write_subreg(&lprf_hw, SR_CTRL_CLK_DIG_PAD, 0);
////	write_subreg(&lprf_hw, SR_CTRL_CLK_FALLB, 0);
	//when internal oscillator does not work, supply clock externally 
	write_subreg(&lprf_hw, SR_CTRL_CLK_DIG_OSC, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_DIG_PAD, 1);
	write_subreg(&lprf_hw, SR_CTRL_CLK_FALLB, 0);
	write_subreg(&lprf_hw, SR_OSCI_BUFFER_EN, 1);
	write_subreg(&lprf_hw, SR_ULP_BUFFER_EN, 1);

	printf("enable CLKREF -> CLKOUT and CLKPLL path\n");
	write_subreg(&lprf_hw, SR_CTRL_CDE_ENABLE, 1);
	write_subreg(&lprf_hw, SR_CTRL_C3X_ENABLE, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_CDE_OSC, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_CDE_PAD, 1);
	write_subreg(&lprf_hw, SR_CTRL_CLK_ADC, 0);	
	write_subreg(&lprf_hw, SR_CTRL_CLK_IREF, 6);  //enable current for clock delay (CDE)
	write_subreg(&lprf_hw, SR_CTRL_CLK_PLL_OSC, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_PLL_PAD, 1);
	uint8_t regdata = read_reg(&lprf_hw, RG_CLK_MAIN);
	printf("CLK_MAIN=0x%0.2X\n", regdata);

	printf("configure PLL\n");
	//Enable all LDOs
	write_subreg(&lprf_hw, SR_LDO_A, 1);
	write_subreg(&lprf_hw, SR_LDO_PLL, 1);
	write_subreg(&lprf_hw, SR_LDO_VCO, 1);
	write_subreg(&lprf_hw, SR_LDO_TX24, 1);
	//configure LDOs
	write_subreg(&lprf_hw, SR_LDO_D_VOUT, 15);
	write_subreg(&lprf_hw, SR_LDO_PLL_VOUT, 31);   //1.74V
	write_subreg(&lprf_hw, SR_LDO_VCO_VOUT, 28);   //1.76V
	write_subreg(&lprf_hw, SR_LDO_TX24_VOUT, 25);  //1.16V
	//set channel
	write_subreg(&lprf_hw, SR_PLL_CHN_INT, 105);
	write_subreg(&lprf_hw, SR_PLL_CHN_FRAC_H, 0);
	write_subreg(&lprf_hw, SR_PLL_CHN_FRAC_M, 0);
	write_subreg(&lprf_hw, SR_PLL_CHN_FRAC_L, 0);
	write_subreg(&lprf_hw, SR_PLL_VCO_TUNE, 196); //set VCO tune word for 1696MHz an PLL_OUT   
	write_subreg(&lprf_hw, SR_IREF_PLL_CTRLB, 0);
	write_subreg(&lprf_hw, SR_PLL_BUFFER_EN, 1);
	write_subreg(&lprf_hw, SR_PLL_EN, 1);

	//reset PLL
	write_subreg(&lprf_hw, SR_PLL_RESETB, 0);
	write_subreg(&lprf_hw, SR_PLL_RESETB, 1);
	
//===============change to state machine operation=========================

	//disable wakeup modes
	write_subreg(&lprf_hw, SR_WAKEUPONSPI, 0);
	write_subreg(&lprf_hw, SR_WAKEUPONRX, 0);
	write_subreg(&lprf_hw, SR_WAKEUP_MODES_EN, 0);
	
	write_subreg(&lprf_hw, SR_TX_CHAN_INT, 112);     //Set PLL divider, write PLL_CHN through statemachine,    //1792MHz an PLL_OUT
	write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_H, 0);    //for 2.4GHz output: sliding IF: 
	write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_M, 0);    //f_RF = 0.75 * f_vco = 0.75 * 32MHz * PLL_CHN
	write_subreg(&lprf_hw, SR_TX_CHAN_FRAC_L, 0);
	write_subreg(&lprf_hw, SR_PLL_VCO_TUNE, 152); //set VCO tune word for 1792MHz an PLL_OUT

	
	write_subreg(&lprf_hw, SR_INVERT_FIFO_CLK, 0);  //invert FIFO clock to enable R/W operation to the fifo
	write_subreg(&lprf_hw, SR_PLL_MOD_EN, 1);       //enable on-chip modulation
	write_subreg(&lprf_hw, SR_PLL_MOD_DATA_RATE, 1);//set tx data rate to <500kBit/s
	write_subreg(&lprf_hw, SR_TX_PWR_CTRL, 15);     //set TX power control to maximum
	
	write_subreg(&lprf_hw, SR_TX_ON_CHIP_MOD_SP, 8);    //set TX_ON_CHIP_MOD_SP to ... (3: 250 kHz; 9: 10 kHz)
	write_subreg(&lprf_hw, SR_TX24_MIXER_PON, 1);       //enable tx mixer
	write_subreg(&lprf_hw, SR_TX24_EN, 1);              //enable 2.4GHz tx frontend
	write_subreg(&lprf_hw, SR_TX_EN, 1);                //enable tx frontend
	write_subreg(&lprf_hw, SR_TX_ON_CHIP_MOD, 1);       //enable on-chip modulation
	write_subreg(&lprf_hw, SR_TX_SPI_FIFO_OUT_EN, 1);   //enable FIFO -> txpath
	write_subreg(&lprf_hw, SR_TX_UPS, 3);               //set upsamling factor to 8
	write_subreg(&lprf_hw, SR_TX_AMPLI_OUT_MAN_L, 240); // set lower 7 of 8 bits to 240
	write_subreg(&lprf_hw, SR_PLL_TPM_COMP_EN, 0);      //disable two point modulation compensation loop

	//wait for TX24 to reach required voltage	
	//char str[80];
	//printf("<Enter>");
	//scanf("%s", str);
	usleep(40000);   //wait 20ms for TXVDD to settle
	printf("\n");
	printf("enable state machine\n");
	write_subreg(&lprf_hw, SR_SM_EN, 1);          //enable state machine
	write_subreg(&lprf_hw, SR_TX_MODE, 0);        //set TX mode to 2.4GHz
	printf("change state to TX\n");
	write_subreg(&lprf_hw, SR_SM_COMMAND, CMD_TX);   //change state to TX
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));

	printf("write frame\n");
	uint8_t len = 48;
	uint8_t *txbuf = txbuf10;
	write_frame(&lprf_hw, txbuf, len);   //160
	write_frame(&lprf_hw, txbuf, len);   //320
	write_frame(&lprf_hw, txbuf, len);   //480
	write_frame(&lprf_hw, txbuf, len);   //640
	write_frame(&lprf_hw, txbuf, len);   //800
	write_frame(&lprf_hw, txbuf, len);   //960
	write_frame(&lprf_hw, txbuf, len);   //1120
	write_frame(&lprf_hw, txbuf, len);   //1280
	write_frame(&lprf_hw, txbuf, len);   //1440
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	
	printf("enable FIFO mode\n");
	write_subreg(&lprf_hw, SR_FIFO_MODE_EN, 1);   //enable FIFO mode
	write_subreg(&lprf_hw, SR_DIRECT_RX, 1);      //enable state transition TX -> RX if fifo empty
	//write_subreg(&lprf_hw, SR_TX_IDLE_MODE_EN, 1);//enable state transition TX -> TX_IDLE if fifo empty

	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));	
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));	
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
	printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));


	int i;
	uint8_t val;

	for(i = 0; i < 100000; i++){
		val = read_reg(&lprf_hw, RG_SM_STATE);	
		if(val == 1){	
			printf("receiving...\n");
			printf("SM_STATE=0x%0.2X     SM_FIFO =0x%0.2X   i=%d\n", val, read_reg(&lprf_hw, RG_SM_FIFO), i);	
			break;
		}
	}

	close(fd);

	return ret;
}

