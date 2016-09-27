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
#include </home/pi/lprf/lprf-driver/lprf_registers.h>
#include </home/pi/spidev/lprf_driver.c>


static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev0.0)\n"
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
//    RX24 Testcase      //
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
	write_subreg(&lprf_hw, SR_CTRL_CLK_C3X_OSC, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_C3X_PAD, 1);
	write_subreg(&lprf_hw, SR_CTRL_CDE_ENABLE, 0);
	write_subreg(&lprf_hw, SR_CTRL_C3X_ENABLE, 1);
	write_subreg(&lprf_hw, SR_CTRL_CLK_FALLB, 0);
	write_subreg(&lprf_hw, SR_CTRL_CLK_ADC, 1);	
	write_subreg(&lprf_hw, SR_CTRL_CLK_IREF, 7);   //enable current for clock delay (CDE)

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
    uint8_t payload_len = 36;
    uint8_t *payload;
    payload = (uint8_t*)malloc(36+1);
    uint8_t i = 0;
    for(i=0;i<payload_len;i++) {
        payload[i] = 0;
    }

    //reset PLL
    write_subreg(&lprf_hw, SR_PLL_RESETB, 0);
    write_subreg(&lprf_hw, SR_PLL_RESETB, 1);
    write_subreg(&lprf_hw, SR_CTRL_CLK_ADC, 1); //disable CLKOUT


//===============change to state machine operation=========================
    printf("configure center freq\n");
    write_subreg(&lprf_hw, SR_RX_CHAN_INT, 102);    //Set PLL divider, write PLL_CHN through statemachine,    //1632MHz an PLL_OUT
    write_subreg(&lprf_hw, SR_RX_CHAN_FRAC_H, 0);       //for 2.448GHz output: sliding IF: 
    write_subreg(&lprf_hw, SR_RX_CHAN_FRAC_M, 0);       //f_RF = 0.75 * f_vco = 0.75 * 32MHz * PLL_CHN
    write_subreg(&lprf_hw, SR_RX_CHAN_FRAC_L, 0);
    write_subreg(&lprf_hw, SR_PLL_VCO_TUNE, 212);       //set VCO tune word for 1632MHz at PLL_OUT

    printf("configure demodulation\n");
    write_subreg(&lprf_hw, SR_DEM_CLK96_SEL, 0);            // set to 96MHz clock
    write_subreg(&lprf_hw, SR_DEM_AGC_EN, 0);               // enable automatic gain control
    write_subreg(&lprf_hw, SR_DEM_FREQ_OFFSET_CAL_EN,0);    // disable frequency offset calculation
    write_subreg(&lprf_hw, SR_DEM_OSR_SEL, 1);              // disable oversampling
    write_subreg(&lprf_hw, SR_DEM_BTLE_MODE, 1);            // disable Bluetooth Low Energy mode
    write_subreg(&lprf_hw, SR_DEM_IQ_INV, 0);               // 0 at IQ inversion
    write_subreg(&lprf_hw, SR_DEM_IQ_CROSS, 0);             // 0 at IQ crossing
    write_subreg(&lprf_hw, SR_DEM_IF_SEL, 2);               // set to 1MHz IF
    write_subreg(&lprf_hw, SR_DEM_DATA_RATE_SEL, 2);        // set data rate; 3=2MBps, 2=1MBps, 1=200kbps, 0=100kbps
    write_subreg(&lprf_hw, SR_DEM_GC3, 2);                  // set gain of 1Mbps CIC filter stage


    printf("configure ADC\n");
    //for ADC single bit operation
    write_subreg(&lprf_hw, SR_CTRL_ADC_IDAC, 0);       //ioSetReg('ADC_IDAC', '00');
    write_subreg(&lprf_hw, SR_CTRL_ADC_IADC, 0);
    write_subreg(&lprf_hw, SR_CTRL_ADC_DWA, 0);        //ioSetReg('ADC_TUNE1', '35');   //0x35 = 0b00110101
    write_subreg(&lprf_hw, SR_CTRL_ADC_DR_SEL, 1);
    //write_subreg(&lprf_hw, SR_CTRL_ADC_BW_TUNE, 5);
    write_subreg(&lprf_hw, SR_CTRL_ADC_BW_TUNE, 7);    //Tune BW to center 1M
    //write_subreg(&lprf_hw, SR_CTRL_ADC_BW_SEL, 1);
        write_subreg(&lprf_hw, SR_CTRL_ADC_BW_SEL, 2);     //BW to 1M

    write_subreg(&lprf_hw, SR_ADC_D_EN, 0);            //ioSetReg('ADC_TUNE2', '01');
    write_subreg(&lprf_hw, SR_CTRL_DSM_MB2SB, 0);      //multibit to single bit
    write_subreg(&lprf_hw, SR_CTRL_DSM_DOE, 0);        //dynamic output enable of dac cells
    write_subreg(&lprf_hw, SR_CTRL_DSM_SDWA, 0);       //single sided data weighed averaging
    write_subreg(&lprf_hw, SR_CTRL_ADC_IOP, 1);        //tunes single bit dac current

    write_subreg(&lprf_hw, SR_CTRL_ADC_D_BYPASS, 0);   //ioSetReg('ADC_MAIN', '11');
    write_subreg(&lprf_hw, SR_ADC_CTRL_RESETB, 1);     //no reset
    write_subreg(&lprf_hw, SR_CTRL_ADC_MULTIBIT, 0);   //no multibit

    printf("configure RX frontend\n");
    write_subreg(&lprf_hw, SR_RX_FE_EN, 1);            //enable RX frontend
    write_subreg(&lprf_hw, SR_RX_RF_MODE, 0);          //set band to 2.4GHz
    write_subreg(&lprf_hw, SR_RX_LO_EXT, 0);           //set to internal LO
    write_subreg(&lprf_hw, SR_RX24_PON, 1);            //power on RX24 frontend
    write_subreg(&lprf_hw, SR_RX800_PON, 0);           //power off RX800 frontend
    write_subreg(&lprf_hw, SR_RX433_PON, 0);           //power on RX433 frontend
    write_subreg(&lprf_hw, SR_PPF_M1, 1);              //magic Polyphase filter settings
    write_subreg(&lprf_hw, SR_PPF_M0, 0);              //magic Polyphase filter settings
    write_subreg(&lprf_hw, SR_PPF_TRIM, 5);            //magic Polyphase filter settings
    write_subreg(&lprf_hw, SR_PPF_HGAIN, 1);           //magic Polyphase filter settings
    write_subreg(&lprf_hw, SR_PPF_LLIF, 0);            //magic Polyphase filter settings
    write_subreg(&lprf_hw, SR_LNA24_ISETT, 7);         //ioSetReg('LNA24_ISETT','07');  max current for (wakeup?) 2.4GHz LNA
    write_subreg(&lprf_hw, SR_LNA24_CTRIM, 255);       //ioSetReg('52','FF'); for best S11
    write_subreg(&lprf_hw, SR_WAKEUPONSPI, 0);         //disable wakeup modes	
    write_subreg(&lprf_hw, SR_WAKEUPONRX, 0);          //disable wakeup modes	
    write_subreg(&lprf_hw, SR_INVERT_FIFO_CLK, 0);     //invert FIFO clock to enable R/W operation to the fifo

    printf("enable and configure state machine\n");
    write_subreg(&lprf_hw, SR_SM_EN, 1);               //enable state machine
    write_subreg(&lprf_hw, SR_FIFO_MODE_EN, 1);        //enable FIFO mode
    write_subreg(&lprf_hw, SR_DIRECT_TX, 0);           //disable transition to TX
    write_subreg(&lprf_hw, SR_DIRECT_TX_IDLE, 0);      //do not transition to TX based on packet counter or fifo full
    write_subreg(&lprf_hw, SR_RX_HOLD_MODE_EN, 0);     //do not transition to RX_HOLD based on packet counter or fifo full
    write_subreg(&lprf_hw, SR_RX_TIMEOUT_EN, 0);       //disable RX_TIMEOUT counter
    write_subreg(&lprf_hw, SR_RX_HOLD_ON_TIMEOUT, 0);  //disable transition RX -> RX_HOLD based on timeout counter
    write_subreg(&lprf_hw, SR_AGC_AUTO_GAIN, 0);       //disable LNA gain switching based on RSSI

    printf("set waiting times to max\n");
    write_subreg(&lprf_hw, SR_SM_TIME_POWER_RX, 256);
    write_subreg(&lprf_hw, SR_SM_TIME_PLL_PON, 256);
    write_subreg(&lprf_hw, SR_SM_TIME_PD_EN, 256);
    printf("SM_STATE=0x%0.2X     SM_FIFO=0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));

    printf("change state to RX     \n");
    write_subreg(&lprf_hw, SR_SM_COMMAND, STATE_CMD_RX);   //change state to TX
    printf("SM_STATE=0x%0.2X     SM_FIFO=0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));

    for(i = 0; i < 10; i++){
        printf("SM_STATE=0x%0.2X     SM_FIFO=0x%0.2X   i=%d\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO), i);	
    }

    printf("read frame            \n");
    payload_len = read_frame(&lprf_hw, payload);   
    printf("SM_STATE=0x%0.2X     SM_FIFO=0x%0.2X\n", read_reg(&lprf_hw, RG_SM_STATE), read_reg(&lprf_hw, RG_SM_FIFO));
    write_subreg(&lprf_hw, SR_SM_COMMAND, STATE_CMD_NONE);  //reset CMD bitfield

    close(fd);
    
    //output frame
    if(payload_len > 0) {
        int row_i, row_max, col_i, col_max, index;
        row_max = 16;    
        col_max = payload_len/16 + 1;
        printf("payload: (len=%d)\n", payload_len);
        for(col_i=0; col_i<col_max; col_i++) {
            printf("%0.4X  ", col_i*row_max);
            for(row_i=0; row_i<row_max; row_i++) {
                index = row_i + row_max*col_i;
                if(index < payload_len)
                    printf("%0.2X", payload[index]);
                if(index % 2)
                    printf(" ");
            }
            printf("\t");
            for(row_i=0; row_i<row_max; row_i++) {
                index = row_i + row_max*col_i;
                if((payload[index] > 31) && (payload[index] < 128))
                    printf("%c", payload[index]);
                else
                    printf(".");
            }
            printf("\n");
        }
        printf("\n");
    } else {
        printf("no payload data received!\n");
    }
    

    return ret;
}
