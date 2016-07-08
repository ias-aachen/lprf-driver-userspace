/*
 * Low Power Radio Frequency Chip Driver (Tapeout Fall 2015) (using spidev driver)
 *
 * Copyright (C) 2016 IAS, RWTH Aachen University
 *	Moritz Schrey <mschrey@ias.rwth.-aachen.de>
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
#include <stdbool.h>  //for false&true
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include </home/pi/spidev/lprf_registers.h>
#include <byteswap.h>
#include </home/pi/spidev/spi_hw_abstraction.h>


#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define CMD_REGR      0x80  //register read
#define CMD_REGW      0xC0  //register write
#define CMD_FRMR      0x20  //frame read
#define CMD_FRMW      0x60  //frame write
#define cmd_len       1
#define addr_len      1
#define data_len      1
#define MAX_FRAME_LEN 128

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

static bool lprf_writeable(unsigned int addr)
{
	switch (addr) {
    case RG_DEM_PD_OUT:        
    case RG_DEM_GC_AOUT:       
    case RG_DEM_GC_BOUT:       
    case RG_DEM_GC_COUT:       
    case RG_DEM_GC_DOUT:          
    case RG_DEM_FREQ_OFFSET_OUT:
    case RG_SM_STATE:          
    case RG_SM_FIFO:           
    case RG_SM_GLOBAL:         
    case RG_SM_POWER:          
    case RG_SM_RX:             
    case RG_SM_WAKEUP_EN:      
    case RG_SM_DEM_ADC:        
    case RG_SM_PLL_TX:         
    case RG_SM_PLL_CHAN_INT:   
    case RG_SM_PLL_CHAN_FRAC_H:
    case RG_SM_PLL_CHAN_FRAC_M:
    case RG_SM_PLL_CHAN_FRAC_L:
    case RG_SM_TX433:          
    case RG_SM_TX800:          
    case RG_SM_TX24:           
    case RG_PLL_TPM_GAIN_OUT_L:
    case RG_PLL_TPM_GAIN_OUT_M:
    case RG_PLL_TPM_GAIN_OUT_H:	
		return false;
	default:
		return true;
	}	
}

//converts from MSB-first to LSB-first and back
uint8_t reverse_bit_order(uint8_t source)
{
	int i;
	uint8_t dest = 0;
	for(i=0; i<8; i++) 
		if(source & (1<<i)) 
			dest += (1<<(7-i));
	return dest;
}

/* for DEEPSLEEP state, CS of SPI bus has to stay low. This is not possible with HW-SPI.
 * Therefore, the following hack is done: 
 *                                                                
 * --------+                                       +----------                        
 * SPI     |                                       |                  
 * Master  |                                       |  LPRF                
 * (Raspi) |                                       |  Chip              
 *         |         _______                       |                   
 * CS (out)o--------|_______|-------o--------------o CS (in)                      
 *         |          1k            |              |                   
 *         |                        |              |
 *  GPIO24 o----------|<|-----------+              |                                           
 *         |                                       |
 * --------+                                       +---------                                                                                  
 * 
 * If GPIO24 is high, the SPI bus functions normally. If GPIO24 is low, CS is kept low and 
 * the LPRF chip stays in DEEPSLEEP mode. 
 */
int overwrite_CS(int state)
{
	FILE *fp = fopen("/sys/class/gpio/gpio24/value", "w");
	if(state == 0)
		fprintf(fp, "0");
	else if(state == 1)
		fprintf(fp, "1");
	else {
		perror("unknown input parameter passed, only 0 and 1 are legal values!\n");
		return -1;
	}
	fclose(fp);
}

/* initialises GPIO24 (pin 18) as output. */
int init_gpio()
{
	FILE *fp;
	//first check if /sys/class/gpio/gpio24/value already exists with correct uid/gid
	fp = fopen("/sys/class/gpio/gpio24/value", "w");
	if (fp == NULL) {		 //fp is NULL, so fclose(fp) is not necessary		
		//we need root permissions at least once to set up 
		//GPIO24 and set its uid and gid to pi:pi
		int uid = getuid();
		if(uid != 0) {
			perror("You're not root! Run this program with sudo\n");
			exit(EXIT_FAILURE);
		}

		int res;
		//export GPIO24
		fp = fopen("/sys/class/gpio/export", "w");
		if (fp == NULL) {
			perror("could not open /sys/class/gpio/export\n");
			return -1;
		}
		res = fprintf(fp, "24");
		if (res != 2) {
			perror("could not write to/sys/class/gpio/export\n");
			fclose(fp);
			return -1;
		}
		fclose(fp);

		//set GPIO24 direction
		fp = fopen("/sys/class/gpio/gpio24/direction", "w");
		if (fp == NULL) {
			perror("could not write to /sys/class/gpio/gpio24/direction\n");
			fclose(fp);
			return -1;
		}
		res = fprintf(fp, "out");
		fclose(fp);
		if (res != 3) {
			perror("could not write to/sys/class/gpio/gpio24/direction\n");
			fclose(fp);	
			return -1;
		}

		//set uid and gid
		res = chown("/sys/class/gpio/gpio24/value", 1000, 1000);
		if(res) {
			perror("unable to set gpio file ownership to pi:pi\n");
			return -1;
		}		
	} else {
		fclose(fp);
	}
}


// lprf_write_reg writes config data into registers
static void lprf_write_reg(int fd, unsigned int addr, unsigned int data)
{
	if (!(lprf_writeable(addr))) {
		printf("register 0x%0.2X not writeable!\n", addr);
		return;
	}
	int ret;
	uint8_t data1b = (uint8_t)data;
	//uint8_t * tx = (uint8_t*)malloc(cmd_len + addr_len + data_len - 1);   
	uint8_t tx[] = {0x00, 0x00, 0x00};
	
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	uint8_t addr_L = (uint8_t)(addr & 0x00FF);        //fix endianness
	tx[0] = CMD_REGW;
	tx[1] = addr_L;
	tx[2] = data1b;
		
	//wait for completion of previous operation
	//printf("W: addr:0x%0.2X data:0x%0.2X\n", tx[1], tx[2]);

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

// lprf_read_reg reads config data from registers
static uint8_t lprf_read_reg(int fd, unsigned int addr)
{
	int ret;
	//uint8_t *tx = (uint8_t*)malloc(cmd_len + addr_len + data_len - 1);    
	uint8_t tx[] = {0x00, 0x00, 0x00};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
	uint8_t addr_L = (uint8_t)(addr & 0x00FF);  //cast addr from unsigned int to uint8_t
	tx[0] = CMD_REGR;
	tx[1] = addr_L;
	tx[2] = 0x00;

	//wait for completion of previous operation	
	
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
	//printf("R: addr:0x%0.2X data:0x%0.2X\n", tx[1], rx[2]);		
		
	return rx[2];
}

/*
 * read frame buffer content
 *  - fd is device file descriptor
 *  - rxbuf is pointer to memory for received data (allocate MAX_FRAME_LEN bytes)    
 * return value is number of data bytes 
 * 
 * Length of received frame is stated as second received byte
 * However, number of bytes to be read must be known before start of 
 * transfer. Therefore maximum amount of memory is allocated.
 */
uint8_t lprf_read_frame(int fd, uint8_t *rxbuf)
{
	int ret;
	uint8_t *tx = (uint8_t*)malloc((cmd_len + addr_len + MAX_FRAME_LEN) * sizeof(uint8_t));
	uint8_t rx[(cmd_len + addr_len + MAX_FRAME_LEN) * sizeof(uint8_t)] = {0, };
	
	tx[0] = CMD_FRMR;
	int i;
	for(i=1; i<MAX_FRAME_LEN; i++) {
		tx[i] = 0x00;
	}

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = cmd_len + addr_len + MAX_FRAME_LEN,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
	
	rxbuf = rx;

	return rx[1];	
}

/*
 * write frame buffer content
 *
 *  - txbuf points to data to be transmitted
 *  - len specifies the number of bytes to be transmitted
 * 
 * All 'len' bytes of txbuf need to be copied as they need
 * to be prefixed with the frame write command. 
 */
void lprf_write_frame(int fd, uint8_t *txbuf, uint8_t len)
{
	int ret;	
	uint8_t *tx;
	tx = (uint8_t*)malloc((len+1)*sizeof(uint8_t));
	tx[0] = CMD_FRMW;
	tx[1] = len;
	int i;
	for(i=0; i<len; i++) {
		txbuf[i] = reverse_bit_order(txbuf[i]);
	}
	memcpy(tx+2, txbuf, len);
	
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		//.rx_buf = NULL,
		//.len = ARRAY_SIZE(tx),
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");
}


struct spi_hw lprf_hw = {
	.fd = 0,
	.transfer_byte = NULL,
	.write_databyte = lprf_write_reg,
	.read_databyte = lprf_read_reg,
	.frame_write = lprf_write_frame,
	.frame_read = lprf_read_frame,
};
