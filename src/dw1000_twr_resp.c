/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), and then waits for a "response" message expected from the "DS TWR responder" example
 *           code (companion to this application). When the response is received its RX time-stamp is recorded and we send a "final" message to
 *           complete the exchange. The final message contains all the time-stamps recorded by this application, including the calculated/predicted TX
 *           time-stamp for the final message itself. The companion "DS TWR responder" example application works out the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * Written by:
 * Peter Hillyard <peterhillyard@gmail.com>
 * Anh Luong <luong@eng.utah.edu>
 * Adwait Dongare <adongare@cmu.edu>
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

// DW1000
#include "deca_device_api.h"
#include "deca_regs.h"
#include "platform.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "TWR W CIR v1.0"

// INITIATOR

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

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
    (1024 + 1 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'I', 'N', 'I', 'T', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'E', 'S', 'P', 0x10, 0x02, 0, 0, 0, 0};
// static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define INIT_RX_BUF_LEN 20
static uint8 rx_buffer_init[INIT_RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 5000 //3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 20000 //5000 //2700
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
// static uint64 final_tx_ts;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
// static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

// RESPONDER

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'I', 'N', 'I', 'T', 0x21, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'E', 'S', 'P', 0x10, 0x02, 0, 0, 0, 0};
// static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RESP_RX_BUF_LEN 24
static uint8 rx_buffer_resp[RESP_RX_BUF_LEN];

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 5000 //2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 6000 //3300

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
// static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/***** CIR INFORMATION *****/
// 992 samples for 16MHz PRF - 3968 bytes
// 1016 samples for 64MHz PRF - 4064 bytes
#define CIR_SAMPLES 1016 //1016

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

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
// static double tof;
// static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
// char dist_str[16] = {0};

// /* Declaration of static functions. */
// static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int main(int argc, char* argv[])
{
    // User input from terminal
    // int isRESP = 0;
    // uint32 msg_counter = 0;
    
    unsigned long exp_number;
    char mode;
    uint8 rx_seq_no;
    int i;
    /* Frame sequence number, incremented after each exchange. */
    uint8 frame_seq_nb = 0;


    char filename[32];
    FILE *output_file;

    uint8 *cir_buffer;
    struct cir_tap_struct *cir;

    dwt_rxdiag_t diagnostics;
    uint16 lde_ppind_val;
    uint16 lde_ppamp_val;
    uint16 lde_threshold_val;
    uint8 lde_config_val;


    /* allocate memory for CIR */
    cir_buffer = (uint8 *) malloc(4*CIR_SAMPLES);
    
    if(cir_buffer == NULL)
    {
        printf("Could not allocate memory\r\n");
        exit(1);
    }

    /* memory map samples for easier decoding */
    cir = (struct cir_tap_struct *) cir_buffer;
    
    if(argc == 3)
    {
        if(!strcmp(argv[1], "INIT"))
        {
            // isRESP = 0;
            mode = 'I';

        } else if (!strcmp(argv[1], "RESP"))
        {
            // isRESP = 1;
            mode = 'R';
        } else
        {
            mode = 'U';
            printf("unrecognized mode\n");
            exit(1);
        }

        exp_number = strtoul(argv[2], NULL, 0);
    }
    else
    {
        printf("usage: %s INIT/RESP exp_number\n", argv[0]);
        exit(1);
    }

    /* Start with board specific hardware init. */
    hardware_init();

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    spi_set_rate_low();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        printf("%s\n", "INIT FAILED");
        exit(1);
    }
    spi_set_rate_high();

    /* Configure DW1000. See NOTE 7 below. */
    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    //dwt_setpreambledetecttimeout(PRE_TIMEOUT); /* Sets the receiver to timeout and disable when no preamble is received within the specified time 5.31 api */

    // Run INITIATOR program
    if(mode == 'I')
    {
        printf("Starting INITIATOR\n");

        /* INITIATOR ONLY */
        /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
         * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS); /* Sets delay to turn on receiver after a frame transmission has completed 5.52 api */
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS); /* Sets the receiver to timeout and disable when no frame is received within the specified time 5.30 api */

        /* Loop forever initiating ranging exchanges. */
        while (1)
        {

            /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
            tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
            dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

            /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
             * set by dwt_setrxaftertxdelay() has elapsed. */
            dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

            printf("INIT %u sent\n", frame_seq_nb);

            /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
            { };

            if (status_reg & SYS_STATUS_RXFCG)
            {
                uint32 frame_len;

                /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                if (frame_len <= INIT_RX_BUF_LEN)
                {
                    dwt_readrxdata(rx_buffer_init, frame_len, 0);
                }

                /* Check that the frame is the expected response from the companion "DS TWR responder" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_seq_no = rx_buffer_init[ALL_MSG_SN_IDX];
                rx_buffer_init[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer_init, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    printf("RESP %u received\n", rx_seq_no);
                    // uint32 final_tx_time;
                    // int ret;

                    /* Retrieve poll transmission and response reception timestamp. */
                    poll_tx_ts = get_tx_timestamp_u64();
                    resp_rx_ts = get_rx_timestamp_u64();

                    printf("exchange,%u\n",  frame_seq_nb);
                    printf("TX1,%llu,RX2,%llu\n", poll_tx_ts, resp_rx_ts);

                    // clear cir_buffer and diagnostics before next sampling
                    memset((void *) cir_buffer, 0, 4*CIR_SAMPLES);
                    memset((void *) &diagnostics, 0, sizeof(dwt_rxdiag_t));
                    /* Copy CIR data */
                    copyCIRToBuffer((uint8 *) cir_buffer, 4*CIR_SAMPLES);
                    /* Get diagnostic data */
                    dwt_readdiagnostics(&diagnostics);
                    /* read other information */
                    lde_threshold_val = dwt_read16bitoffsetreg(LDE_IF_ID, 0x0000);
                    lde_ppind_val = dwt_read16bitoffsetreg(LDE_IF_ID, 0x1000);
                    lde_ppamp_val = dwt_read16bitoffsetreg(LDE_IF_ID, 0x1002);
                    lde_config_val = dwt_read8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET);

                    snprintf(filename, 31, "exp%lu_msg%u_%c.csv", exp_number, frame_seq_nb, mode);
                    printf("start writing to %s...\n", filename);


                    /***** FILE OPERATIONS *****/

                    output_file = fopen(filename, "w");

                    if (output_file == NULL){
                        printf("unable to write\n");
                        
                    } else {
                        
                        fprintf(output_file, "exchange, %u\n",  frame_seq_nb);
                        fprintf(output_file, "TX1,%llu,RX2,%llu\n", poll_tx_ts, resp_rx_ts);
                        fprintf(output_file, "maxNoise,%u\n", diagnostics.maxNoise);
                        fprintf(output_file, "stdNoise,%u\n", diagnostics.stdNoise);
                        fprintf(output_file, "firstPathAmp1,%u\n", diagnostics.firstPathAmp1);
                        fprintf(output_file, "firstPathAmp2,%u\n", diagnostics.firstPathAmp2);
                        fprintf(output_file, "firstPathAmp3,%u\n", diagnostics.firstPathAmp3);
                        fprintf(output_file, "maxGrowthCIR,%u\n", diagnostics.maxGrowthCIR);
                        fprintf(output_file, "rxPreamCount,%u\n", diagnostics.rxPreamCount);
                        fprintf(output_file, "firstPath,%u\n", diagnostics.firstPath);
                        fprintf(output_file, "LDE_THRESH,%u\n", lde_threshold_val);
                        fprintf(output_file, "LDE_CFG1,%u\n", lde_config_val);
                        fprintf(output_file, "LDE_PPINDX,%u\n", lde_ppind_val);
                        fprintf(output_file, "LDE_PPAMPL,%u\n", lde_ppamp_val);

                        fprintf(output_file, "CIRIQ\n");
                        for (i = 0; i < CIR_SAMPLES; i++)
                        {
                            fprintf(output_file, "%04X,%04X\n", cir[i].real, cir[i].img);
                        }
                        
                        fclose(output_file);

                        printf("done writing\n");
                    }

                    printf("\n");
                    //usleep(50);

                    /* We have all info, print diagnostics and save to file*/

                    // /* Compute final message transmission time. See NOTE 10 below. */
                    // final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    // dwt_setdelayedtrxtime(final_tx_time);

                    // /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                    // final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;


                    // /* Write all timestamps in the final message. See NOTE 11 below. */
                    // final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                    // final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                    // final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                    // /* Write and send final message. See NOTE 8 below. */
                    // tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    // dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
                    // dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                    // ret = dwt_starttx(DWT_START_TX_DELAYED);

                    // /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
                    // if (ret == DWT_SUCCESS)
                    // {
                    //     /* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
                    //     while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    //     { };

                    //     printf("Transmission 3 sent\n");

                    //     /* Clear TXFRS event. */
                    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    //     /* Increment frame sequence number after transmission of the final message (modulo 256). */
                    //     frame_seq_nb++;
                    // }
                }
            }
            else
            {
                printf("Timeout: did not receive RESP\n");
                /* Clear RX error/timeout events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();
            }

            /* Increment frame sequence number after transmission of the poll message (modulo 256). */
            frame_seq_nb++;

            /* Execute a delay between ranging exchanges. */
            sleep_ms(RNG_DELAY_MS);
        }
    }
    else
    {
        /* RESPONDER */
        printf("Starting RESPONDER\n");

        /* Loop forever responding to ranging requests. */
        while (1)
        {
            /* Clear reception timeout to start next ranging process. */
            dwt_setrxtimeout(0);

            /* Activate reception immediately. */
            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
            { };

            if (status_reg & SYS_STATUS_RXFCG)
            {

                uint32 frame_len;

                /* Clear good RX frame event in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= RESP_RX_BUF_LEN)
                {
                    dwt_readrxdata(rx_buffer_resp, frame_len, 0);
                }

                /* Check that the frame is a poll sent by "DS TWR initiator" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_seq_no = rx_buffer_resp[ALL_MSG_SN_IDX];
                rx_buffer_resp[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer_resp, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    printf("INIT %u received\n", rx_seq_no);
                    uint32 resp_tx_time;
                    int ret;

                    /* Retrieve poll reception timestamp. */
                    poll_rx_ts = get_rx_timestamp_u64();
                    //usleep(50);

                    frame_seq_nb = rx_seq_no;

                    /* Set send time for response. See NOTE 9 below. */
                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
                    // dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                    // dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                    /* Write and send the response message. See NOTE 10 below.*/
                    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
                    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                    ret = dwt_starttx(DWT_START_TX_DELAYED);

                    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
                    if (ret == DWT_ERROR)
                    {
                        printf("RESP %u abandonned\n", frame_seq_nb);
                        continue;
                    }

                    printf("RESP %u sent\n", frame_seq_nb);

                    /* Poll DW1000 until TX frame sent event set. */
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                    { };

                    resp_tx_ts = get_tx_timestamp_u64();

                    /* Clear TX frame sent event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                    printf("exchange,%u\n",  frame_seq_nb);
                    printf("TX1,%llu,RX2,%llu\n", poll_rx_ts, resp_tx_ts);

                    // clear cir_buffer and diagnostics before next sampling
                    memset((void *) cir_buffer, 0, 4*CIR_SAMPLES);
                    memset((void *) &diagnostics, 0, sizeof(dwt_rxdiag_t));
                    /* Copy CIR data */
                    copyCIRToBuffer((uint8 *) cir_buffer, 4*CIR_SAMPLES);
                    /* Get diagnostic data */
                    dwt_readdiagnostics(&diagnostics);
                    // printf("CIR\n");
                    // for (i = 0; i < CIR_SAMPLES; i++)
                    // {
                    //     printf("%04X,%04X\n", cir[i].real, cir[i].img);
                    // }
                    /* read other information */
                    lde_threshold_val = dwt_read16bitoffsetreg(LDE_IF_ID, 0x0000);
                    lde_ppind_val = dwt_read16bitoffsetreg(LDE_IF_ID, 0x1000);
                    lde_ppamp_val = dwt_read16bitoffsetreg(LDE_IF_ID, 0x1002);
                    lde_config_val = dwt_read8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET);

                    snprintf(filename, 31, "exp%lu_msg%u_%c.csv", exp_number, frame_seq_nb, mode);
                    printf("start writing to %s...\n", filename);


                    /***** FILE OPERATIONS *****/

                    output_file = fopen(filename, "w");

                    if (output_file == NULL){
                        printf("unable to write\n");
                        
                    } else {
                        
                        fprintf(output_file, "exchange, %u\n",  frame_seq_nb);
                        fprintf(output_file, "RX1,%llu,TX2,%llu\n", poll_rx_ts, resp_tx_ts);
                        fprintf(output_file, "maxNoise,%u\n", diagnostics.maxNoise);
                        fprintf(output_file, "stdNoise,%u\n", diagnostics.stdNoise);
                        fprintf(output_file, "firstPathAmp1,%u\n", diagnostics.firstPathAmp1);
                        fprintf(output_file, "firstPathAmp2,%u\n", diagnostics.firstPathAmp2);
                        fprintf(output_file, "firstPathAmp3,%u\n", diagnostics.firstPathAmp3);
                        fprintf(output_file, "maxGrowthCIR,%u\n", diagnostics.maxGrowthCIR);
                        fprintf(output_file, "rxPreamCount,%u\n", diagnostics.rxPreamCount);
                        fprintf(output_file, "firstPath,%u\n", diagnostics.firstPath);
                        fprintf(output_file, "LDE_THRESH,%u\n", lde_threshold_val);
                        fprintf(output_file, "LDE_CFG1,%u\n", lde_config_val);
                        fprintf(output_file, "LDE_PPINDX,%u\n", lde_ppind_val);
                        fprintf(output_file, "LDE_PPAMPL,%u\n", lde_ppamp_val);

                        fprintf(output_file, "CIRIQ\n");
                        for (i = 0; i < CIR_SAMPLES; i++)
                        {
                            fprintf(output_file, "%04X,%04X\n", cir[i].real, cir[i].img);
                        }
                        
                        fclose(output_file);

                        printf("done writing\n");
                    }

                    printf("\n");

                    // /* Increment frame sequence number after transmission of the response message (modulo 256). */
                    // // frame_seq_nb++;

                    // if (status_reg & SYS_STATUS_RXFCG)
                    // {
                    //     /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                    //     /* A frame has been received, read it into the local buffer. */
                    //     frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    //     if (frame_len <= RESP_RX_BUF_LEN)
                    //     {
                    //         dwt_readrxdata(rx_buffer_resp, frame_len, 0);
                    //     }

                    //     /* Check that the frame is a final message sent by "DS TWR initiator" example.
                    //      * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
                    //     rx_buffer_resp[ALL_MSG_SN_IDX] = 0;
                    //     if (memcmp(rx_buffer_resp, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                    //     {
                    //         //printf("Tranmission 3 received\n");
                    //         uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                    //         uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                    //         double Ra, Rb, Da, Db;
                    //         int64 tof_dtu;

                    //         /* Retrieve response transmission and final reception timestamps. */
                    //         resp_tx_ts = get_tx_timestamp_u64();
                    //         final_rx_ts = get_rx_timestamp_u64();

                    //         /* Get timestamps embedded in the final message. */
                    //         final_msg_get_ts(&rx_buffer_resp[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                    //         final_msg_get_ts(&rx_buffer_resp[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                    //         final_msg_get_ts(&rx_buffer_resp[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                    //         /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                    //         poll_rx_ts_32 = (uint32)poll_rx_ts;
                    //         resp_tx_ts_32 = (uint32)resp_tx_ts;
                    //         final_rx_ts_32 = (uint32)final_rx_ts;
                    //         Ra = (double)(resp_rx_ts - poll_tx_ts);
                    //         Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                    //         Da = (double)(final_tx_ts - resp_rx_ts);
                    //         Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                    //         tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                    //         tof = tof_dtu * DWT_TIME_UNITS;
                    //         distance = tof * SPEED_OF_LIGHT;

                    //         printf("%3.9e sec ", tof);
                    //         printf("%4.3f m\n", tof*299792458.0*0.84);

                    //         /* Display computed distance on LCD. */
                    //         // sprintf(dist_str, "DIST: %3.2f m", distance);
                    //         // lcd_display_str(dist_str);
                    //     }
                    // }
                    // else
                    // {
                    //     /* Clear RX error/timeout events in the DW1000 status register. */
                    //     dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                    //     /* Reset RX to properly reinitialise LDE operation. */
                    //     dwt_rxreset();
                    // }
                }
            }
            else
            {
                /* Clear RX error/timeout events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

                /* Reset RX to properly reinitialise LDE operation. */
                dwt_rxreset();
            }
        }
    }

    cir = NULL;
    free(cir_buffer);

}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}


// /*! ------------------------------------------------------------------------------------------------------------------
//  * @fn final_msg_set_ts()
//  *
//  * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
//  *        message, the least significant byte is at the lower address.
//  *
//  * @param  ts_field  pointer on the first byte of the timestamp field to fill
//  *         ts  timestamp value
//  *
//  * @return none
//  */
// static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
// {
//     int i;
//     for (i = 0; i < FINAL_MSG_TS_LEN; i++)
//     {
//         ts_field[i] = (uint8) ts;
//         ts >>= 8;
//     }
// }


// /*! ------------------------------------------------------------------------------------------------------------------
//  * @fn final_msg_get_ts()
//  *
//  * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
//  *        significant byte is at the lower address.
//  *
//  * @param  ts_field  pointer on the first byte of the timestamp field to read
//  *         ts  timestamp value
//  *
//  * @return none
//  */
// static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
// {
//     int i;
//     *ts = 0;
//     for (i = 0; i < FINAL_MSG_TS_LEN; i++)
//     {
//         *ts += ts_field[i] << (i * 8);
//     }
// }











/*****************************************************************************************************************************************************
 * INITIATOR NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    110k data rate used (around 3 ms).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 8. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 9. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 10. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
 *     register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *     response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *     lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *     8 bits.
 * 11. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *     time-of-flight computation) can be handled by a 32-bit subtraction.
 * 12. When running this example on the EVB1000 platform with the RESP_RX_TO_FINAL_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange to try another one after 1 second. If this error handling code was not here, a late dwt_starttx() would result in the code
 *     flow getting stuck waiting for a TX frame sent event that will never come. The companion "responder" example (ex_05b) should timeout from
 *     awaiting the "final" and proceed to have its receiver on ready to poll of the following exchange.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the EVB1000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
