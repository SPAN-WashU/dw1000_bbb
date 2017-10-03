/*
 * platform.h
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

#include "deca_types.h"
#include "deca_device_api.h"
#include <stdint.h>
#include <fcntl.h>

#define DECA_MAX_SPI_HEADER_LENGTH      (3)                     // max number of bytes in header (for formating & sizing)

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn hardware_init()
 *
 * @brief Initialise all peripherals at once.
 *
 * @param none
 *
 * @return none
 */
int hardware_init();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn reset_DW1000()
 *
 * @brief Hardware reset DW1000.
 *
 * @param none
 *
 * @return none
 */
int reset_DW1000();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
int spi_set_rate_low();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
int spi_set_rate_high();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn sleep_ms()
 *
 * @brief sleep for <time_ms> milliseconds
 *
 * @param <time_ms> milliseconds
 *
 * @return none
 */
void sleep_ms(unsigned int time_ms);

// ---------------------------------------------------------------------------
//
// NB: The purpose of this section is to provide for microprocessor interrupt enable/disable, this is used for 
//     controlling mutual exclusion from critical sections in the code where interrupts and background 
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also 
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in the
//     deca_irq.h include file to map these calls transparently to the target system.  Alternatively the 
//     appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//	   If HW port uses EXT_IRQ line to receive ready/busy status from DW1000 then mutex should use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
//	   For critical section use this mutex instead
//	   __save_intstate()
//     __restore_intstate()
// ---------------------------------------------------------------------------

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:	
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
decaIrqStatus_t decamutexon(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon 
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:	
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn reset_DW1000()
 *
 * @brief Hardware reset DW1000.
 *
 * @param none
 *
 * @return none
 */
void sleep_ms(unsigned int time_ms);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readtx_sys_count()
 *
 * @brief This is used to read the system counter associated with the TX timestamp
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read system counter
 *
 * output parameters - the system counter buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readtx_sys_count(uint8 * timestamp);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn dwt_readrx_sys_count()
 *
 * @brief This is used to read the system counter associated with the RX timestamp
 *
 * input parameters
 * @param timestamp - a pointer to a 5-byte buffer which will store the read system counter
 *
 * output parameters - the system counter buffer will contain the value after the function call
 *
 * no return value
 */
void dwt_readrx_sys_count(uint8 * timestamp);