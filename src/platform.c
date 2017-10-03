/*
 * platform.c
 *
 * Copyright (C) 2016 University of Utah
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Written by:
 * Anh Luong <luong@eng.utah.edu>
 * Peter Hillyard <peterhillyard@gmail.com>
 */

#include "platform.h"
#include <unistd.h> // for usleep
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "deca_regs.h"

#define SPI_SPEED_SLOW    				( 3000000)
#define SPI_SPEED_FAST  	  			(10000000)
#define SPI_PATH 						"/dev/spidev1.0"

static uint32_t mode 	= 0;
static uint8_t bits 	= 8;
static uint32_t speed 	= SPI_SPEED_SLOW;
static uint16_t delay 	= 0;

static int fd;

int RSTPin = 46; /* Reset GPIO pin - GPIO1_14 or pin 16 on the P8 header */
int IRQPin = 47; /* Reset GPIO pin - GPIO1_15 or pin 15 on the P8 header */
FILE *resetGPIO = NULL;
FILE *irqGPIO = NULL;

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
	sleep_ms(time_ms);
}

void sleep_ms(unsigned int time_ms)
{
	usleep(time_ms * 1000);
}

int spi_set_rate_low (void)
{
	speed = SPI_SPEED_SLOW;
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

	return 0;
}

int spi_set_rate_high (void)
{
	speed = SPI_SPEED_FAST;
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}

	return 0;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
	int status;
	int i;

	for (i = 0; i < 128; i++)
	{
		
	}

	uint8_t txbuf[headerLength+bodylength];
	uint8_t rxbuf[headerLength+bodylength];

	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)txbuf,
		.rx_buf = (unsigned long)rxbuf,
		.len = headerLength+bodylength,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	int j;
	for(j = 0; j < headerLength; j++)
	{
		txbuf[j] = headerBuffer[j];
	}

	for(j = 0; j < bodylength; j++)
	{
		txbuf[headerLength+j] = bodyBuffer[j];
	}

	//printf("hdr buf: %02X %02X body buf: %02X %02X tx buf: %02X %02X %02X %02X\n", headerBuffer[0], headerBuffer[1], bodyBuffer[0], bodyBuffer[1], txbuf[0], txbuf[1], txbuf[headerLength], txbuf[headerLength+1]);
	//printf("hdr: %d body: %d tx: %d\n", headerLength, bodylength, transfer.len);

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(status < 0)
		return DWT_ERROR;

	return DWT_SUCCESS;


} // end writetospi()

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	int status;
	uint8_t buf[readlength];

	struct spi_ioc_transfer transfer = {
		.tx_buf = (unsigned long)headerBuffer,
		.rx_buf = (unsigned long)buf,
		.len = headerLength+readlength,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	// send the SPI message (all of the above fields, inc. buffers)
	status = ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
	if(status < 0)
		return DWT_ERROR;

	int j;
	for(j = 0; j < readlength; j++)
	{
		readBuffer[j] = buf[j+headerLength];
	}

	return DWT_SUCCESS;

} // end readfromspi()

int hardware_init (void)
{
	char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];

    // Setup RESET
	sprintf(GPIOString, "%d", RSTPin);
	sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", RSTPin);
	sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", RSTPin);

    // Export the pin
	if ((resetGPIO = fopen("/sys/class/gpio/export", "ab")) == NULL){
		printf("Unable to export GPIO pin\n");
		return 1;
	}
	strcpy(setValue, GPIOString);
	fwrite(&setValue, sizeof(char), 2, resetGPIO);
	fclose(resetGPIO);

    // Set direction of the pin to an output
	if ((resetGPIO = fopen(GPIODirection, "rb+")) == NULL){
		printf("Unable to open direction handle\n");
		return 1;
	}
	strcpy(setValue,"out");
	fwrite(&setValue, sizeof(char), 3, resetGPIO);
	fclose(resetGPIO);

    // Setup IRQ
	sprintf(GPIOString, "%d", IRQPin);
	sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", IRQPin);
	sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", IRQPin);

    // Export the pin
	if ((irqGPIO = fopen("/sys/class/gpio/export", "ab")) == NULL){
		printf("Unable to export GPIO pin\n");
		return 1;
	}
	strcpy(setValue, GPIOString);
	fwrite(&setValue, sizeof(char), 2, irqGPIO);
	fclose(irqGPIO);

    // Set direction of the pin to an output
	if ((irqGPIO = fopen(GPIODirection, "rb+")) == NULL){
		printf("Unable to open direction handle\n");
		return 1;
	}
	strcpy(setValue,"in");
	fwrite(&setValue, sizeof(char), 3, irqGPIO);
	fclose(irqGPIO);

	// The following calls set up the SPI bus properties
	if((fd = open(SPI_PATH, O_RDWR))<0){
		perror("SPI Error: Can't open device.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode)==-1){
		perror("SPI: Can't set SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MODE, &mode)==-1){
		perror("SPI: Can't get SPI mode.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't set bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits)==-1){
		perror("SPI: Can't get bits per word.");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't set max speed HZ");
		return -1;
	}
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed)==-1){
		perror("SPI: Can't get max speed HZ.");
		return -1;
	}
	return 0;
}

int reset_DW1000(void)
{
	char setValue[4], GPIOValue[64];
	sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", RSTPin);

    // Set output to low
	if ((resetGPIO = fopen(GPIOValue, "rb+")) == NULL){
		printf("Unable to open value handle\n");
		return 1;
	}
    strcpy(setValue, "0"); // Set value low
    fwrite(&setValue, sizeof(char), 1, resetGPIO);
    fclose(resetGPIO);
    sleep_ms(2);

	// Set output to high
    if ((resetGPIO = fopen(GPIOValue, "rb+")) == NULL){
    	printf("Unable to open value handle\n");
    	return 1;
    }
    strcpy(setValue, "1"); // Set value high
    fwrite(&setValue, sizeof(char), 1, resetGPIO);
    fclose(resetGPIO);
    sleep_ms(2);
    return 0;
}

decaIrqStatus_t decamutexon(void) 
{
	decaIrqStatus_t s = 0;

	if(s) {
		// no interrupt lines
	}
	return s ;   // return state before disable, value is used to re-enable in decamutexoff call
}

void decamutexoff(decaIrqStatus_t s)        // put a function here that re-enables the interrupt at the end of the critical section
{
	if(s) { //need to check the port state as we can't use level sensitive interrupt on the STM ARM
		// no interrupt lines
	}
}

void dwt_readtx_sys_count(uint8 * timestamp)
{
    dwt_readfromdevice(TX_TIME_ID, TX_TIME_TX_RAWST_OFFSET, TX_TIME_TX_STAMP_LEN, timestamp) ; // Read bytes directly into buffer
}

void dwt_readrx_sys_count(uint8 * timestamp)
{
    dwt_readfromdevice(RX_TIME_ID, RX_TIME_FP_RAWST_OFFSET, RX_TIME_RX_STAMP_LEN, timestamp) ; // Read bytes directly into buffer
}
