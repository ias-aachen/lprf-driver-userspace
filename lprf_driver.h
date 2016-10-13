#ifndef LPRF_DRIVER
#define LPRF_DRIVER

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
#include </home/pi/lprf/lprf-driver/lprf_registers.h>
#include <byteswap.h>
#include "spi_hw_abstraction.h"


#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define CMD_REGR      0x80  //register read
#define CMD_REGW      0xC0  //register write
#define CMD_FRMR      0x20  //frame read
#define CMD_FRMW      0x60  //frame write
#define cmd_len       1
#define addr_len      1
#define data_len      1
#define LPRF_MAX_FRAME_LEN 255

//sizeof(uint8_t)     =1
//sizeof(uint16_t)    =2
//sizeof(int)         =4
//sizeof(unsigned int)=4


extern const char *device;
extern uint8_t mode;
extern uint8_t bits;
extern uint32_t speed;
extern uint16_t delay;

void pabort(const char *s);
uint8_t reverse_bit_order(uint8_t source);
void lprf_write_reg(int fd, unsigned int addr, unsigned int data);
uint8_t lprf_read_reg(int fd, unsigned int addr);
uint8_t lprf_phy_status_byte(int fd);
uint8_t lprf_read_frame(int fd, uint8_t *rxbuf);
void lprf_write_frame(int fd, uint8_t *txbuf, uint8_t len);

extern struct spi_hw lprf_hw;

#endif // LPRF_DRIVER
