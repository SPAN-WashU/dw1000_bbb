/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Simple RX with diagnostics example code
 *
 *           This application waits for reception of a frame. After each frame received with a good CRC it reads some data provided by DW1000:
 *               - Diagnostics data (e.g. first path index, first path amplitude, channel impulse response, etc.). See dwt_rxdiag_t structure for more
 *                 details on the data read.
 *               - Accumulator values around the first path.
 *           It also reads event counters (e.g. CRC good, CRC error, PHY header error, etc.) after any event, be it a good frame or an RX error. See
 *           dwt_deviceentcnts_t structure for more details on the counters read.
 *
 * @attention
 *
 * Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

 /*
 * Modified by:
 * Anh Luong <luong@eng.utah.edu>
 */

#include <stdio.h>
#include <stdlib.h> // malloc, free
#include <unistd.h>
#include <string.h> // memset

#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "RX DIAG v1.1"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dwt_rxdiag_t diagnostics;

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

// 992 samples for 16MHz PRF - 3968 bytes
// 1016 samples for 64MHz PRF - 4064 bytes
#define CIR_SAMPLES 100 //1016

struct cir_tap_struct {
    uint16 real;
    uint16 img;
};

#define ACC_CHUNK 64 // bytes read at the same time

void copyCIRToBuffer(uint8 *buffer, uint16 len)
{
    int loc = 0;
    uint16 toRead = 0;
    int lastRead = 0;

    // we need to create a buffer to discard the dummy byte
    uint8 buf[ACC_CHUNK+1];
    
    while(1)
    {

        memset((void *) buf, 0, ACC_CHUNK + 1);

        if(len > ACC_CHUNK)
        {
            // need to loop again
            // only read the max allowable number of characters
            toRead = ACC_CHUNK;
        }
        else
        {
            // we will cover the entire length in this iteration of the loop
            // only read the remaining length
            toRead = len;
            lastRead = 1;
        }

        // read from device
        dwt_readaccdata(buf, toRead, loc);

        // copy to the buffer
        memcpy((void *) &buffer[loc], (void *) &buf[1], toRead);

        if(lastRead)
        {
            break;
        }

        // decrease remaining length
        len = len - toRead;

        // increase pointer in buffer
        loc = loc + toRead;
    }
}

/**
 * Application entry point.
 */
int main(nargs, args)
{

    uint8 *cir_buffer;
    int i;

    cir_buffer = (uint8 *) malloc(4*CIR_SAMPLES);

    struct cir_tap_struct *cir = (struct cir_tap_struct *) &cir_buffer[0];

    if(cir_buffer == NULL)
    {
        printf("Could not allocate memory\r\n");
        exit(1);
    }

    /* Start with board specific hardware init. */
    hardware_init();

    /* Reset and initialise DW1000. See NOTE 2 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        printf("Unable to initialize UCODE\r\n");
        exit(1);
    }
    spi_set_rate_high();

    /* Configure DW1000. */
    dwt_configure(&config);

    printf("%s\r\n", APP_NAME);

    /* Loop forever receiving frames. */
    while (1)
    {

        /* TESTING BREAKPOINT LOCATION #1 */

        /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
         * the RX buffer.
         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
        for (i = 0 ; i < FRAME_LEN_MAX; i++ )
        {
            rx_buffer[i] = 0;
        }

        // clear cir_buffer before next sampling
        memset((void *) cir_buffer, 0, 4*CIR_SAMPLES);

        diagnostics.firstPath = 0;
        diagnostics.firstPathAmp1 = 0;
        diagnostics.firstPathAmp2 = 0;
        diagnostics.firstPathAmp3 = 0;
        diagnostics.maxGrowthCIR = 0;
        diagnostics.rxPreamCount = 0;
        diagnostics.maxNoise = 0;
        diagnostics.stdNoise = 0;

        /* Activate reception immediately. See NOTE 3 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
   
            printf("MSG Received! DATA: %s\r\n", rx_buffer);

            dwt_readdiagnostics(&diagnostics);
            printf("FP: %d, STD_NOISE: %d, MAX_NOISE: %d \r\n", diagnostics.firstPath, diagnostics.stdNoise, diagnostics.maxNoise);

            // uint16 fp_int = diagnostics.firstPath / 64;
            //dwt_readaccdata((uint8 *)&cir, CIR_SAMPLES, (fp_int - 2) * 4);
            // dwt_readaccdata((uint8 *) cir_buffer, 4*CIR_SAMPLES + 1, 0);

            copyCIRToBuffer((uint8 *) cir_buffer, 4*CIR_SAMPLES);

            // Print CIR

            // printf("CIR Bytes:");
            // for(i = 0; i < 4*CIR_SAMPLES + 1; i++)
            // {
            //     printf("%01X", cir_buffer[i]);
            // }

            printf("CIR Real: ");
            for (i = 0; i < CIR_SAMPLES; i++)
            {
                printf("%04X ", cir[i].real);
            }
            printf("\n");

            printf("CIR Imaginary: ");
            for (i = 0; i < CIR_SAMPLES; i++)
            {
                 printf("%04X ", cir[i].img);
            }

            printf("\n");
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
        }
    }

    cir = NULL;
    free(cir_buffer);
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW1000 supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Accumulator values are complex numbers: one 16-bit integer for real part and one 16-bit value for imaginary part, for each sample. In this
 *    example, we chose to read 3 values below the first path index and 3 values above. It must be noted that the first byte read when accessing the
 *    accumulator memory is always garbage and must be discarded, that is why the data length to read is increased by one byte here.
 * 3. In this example, LDE microcode is loaded even if timestamps are not used because diagnostics values are computed during LDE execution. If LDE is
 *    not loaded and running, dwt_readdiagnostics will return all 0 values.
 * 4. Manual reception activation is performed here but DW1000 offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 5. We use polled mode of operation here to keep the example as simple as possible but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW1000 User Manual for more details on "interrupts".
 * 6. Here we chose to read only a few values around the first path index but it is possible and can be useful to get all accumulator values, using
 *    the relevant offset and length parameters. Reading the whole accumulator will require 4064 bytes of memory. First path value gotten from
 *    dwt_readdiagnostics is a 10.6 bits fixed point value calculated by the DW1000. By dividing this value by 64, we end up with the integer part of
 *    it. This value can be used to access the accumulator samples around the calculated first path index as it is done here.
 * 7. Event counters are never reset in this example but this can be done by re-enabling them (i.e. calling again dwt_configeventcounters with
 *    "enable" parameter set).
 * 8. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
