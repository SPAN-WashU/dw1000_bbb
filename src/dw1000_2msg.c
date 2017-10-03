/*! ----------------------------------------------------------------------------
 *  @file    dw1000_2msg.c
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
#define APP_NAME "2MSG w/CIR v1.0"

/***** USEFUL PARAMETERS *****/

/* Inter-ranging delay period, in milliseconds. */
#define INTER_RANGING_TIME  1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY          16436
#define RX_ANT_DLY          16436

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define TX_TO_RX_DLY_UUS    150

/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RX_TO_TX_DLY_UUS    5000

/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

/* RX timeout for messages in same exchange */
#define RX_TIMEOUT_UUS      20000

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME     65536

/***** CIR INFORMATION *****/
// 992 samples for 16MHz PRF - 3968 bytes
// 1016 samples for 64MHz PRF - 4064 bytes
#define CIR_SAMPLES 1016 //1016

/* Due to limits oif the kernel SPI driver, long SPI transactions must be broken down into multiple parts
   ACC_CHUNKS describes the number of CIR data bytes that are read at a time */
#define ACC_CHUNK 64 // bytes read in a single SPI transaction

/* size of decawave rx buffer*/
#define RX_BUF_LEN 24

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


/* Message content details */
#define MSGNO_IDX   2   // all messages

#define PREV_RX_IDX     6
#define THIS_TX_IDX     14

#define TX1_IDX         THIS_TX_IDX // init message
#define RX1_IDX         PREV_RX_IDX // resp message
#define TX2_IDX         THIS_TX_IDX // resp message

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

typedef unsigned long long uint64;
typedef signed long long int64;

struct cir_struct {
    int16 real;
    int16 img;
};

struct completeChannelInfo {
    uint32                  exchangeNo;
    dwt_rxdiag_t            diagnostics;
    uint16                  lde_thresh;
    uint16                  lde_ppindx;
    uint16                  lde_ppampl;
    uint8                   lde_cfg1;
    int32                   carrierintegrator;
    struct cir_struct       cir[CIR_SAMPLES];
};

struct ts_struct {
    uint64 tx_timestamp[2];
    uint64 rx_timestamp[2];
};

/***** Function declarations *****/

static void copyCIRToBuffer(uint8 *buffer, uint16 len);
static void initiatorTask(unsigned long exp_number, char mode);
static void responderTask(unsigned long exp_number, char mode);

static void setup_dw1000(void);
static void copyChannelInfo(struct completeChannelInfo *info, uint32 exchange_no);
static void saveChannelInfoToFile(char *filename, struct completeChannelInfo *info, struct ts_struct *ts, int type);

static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static uint64 get_system_timestamp_u64(void);
static uint32 next_delayed_tx(uint64 time, uint32 delay_uus);
static uint64 delayed_tx_timestamp(uint32 delayed_tx_time);
static double get_tof(struct ts_struct *timestamps, float clockOffset);
static float get_clock_offset(int32 carrier_offset);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int main(int argc, char* argv[]) {
    // User input from terminal
    // int isRESP = 0;
    // uint32 msg_counter = 0;
    
    unsigned long exp_number;
    char mode;
    
    if(argc == 3) {
        if(!strcmp(argv[1], "INIT")) {
            // isRESP = 0;
            mode = 'I';

        } else if (!strcmp(argv[1], "RESP")) {
            // isRESP = 1;
            mode = 'R';
        } else {
            mode = 'U';
            printf("unrecognized mode\n");
            exit(1);
        }

        exp_number = strtoul(argv[2], NULL, 0);
    } else {
        printf("usage: %s INIT/RESP exp_number\n", argv[0]);
        exit(1);
    }

    /* Start with board specific hardware init. */
    hardware_init();
    setup_dw1000();

    
    if(mode == 'I') {
        // Run INITIATOR program
        initiatorTask(exp_number, mode);
    } else {
        // Run RESPONDER program
        responderTask(exp_number, mode);
    }

    return 0;

}

static void setup_dw1000(void) {

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

}

static void initiatorTask(unsigned long exp_number, char mode) {

    uint32 exchangeNo = 0;
    uint32 frame_len = 0;
    uint32 status_reg = 0;
    int ret = 0;
    uint64 time = 0;
    uint32 delayed_tx_time = 0;
    double tof;
    float clockOffset;
    struct completeChannelInfo *rxInfo;
    struct ts_struct timestamps;
    char filename[32];
    
    /* Frames used in the ranging process. See NOTE 2 below. */
    uint8 tx1_msg[] = {0xab, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // size = 1+1+4+8+8+2 = 24
    uint8 rx2_msg[] = {0xab, 0x01, 0, 0, 0, 0}; // actual message size = 1+1+4+8+8+2 = 24
    uint8 rx_buffer[RX_BUF_LEN];

    printf("Starting INITIATOR\n");

    

    // allocate memory
    rxInfo = (struct completeChannelInfo *) malloc(sizeof(struct completeChannelInfo));
    if (rxInfo == NULL) {
        printf("Error: could not allocate memory\n");
        exit(1);
    }

    /* INITIATOR ONLY */
    /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(TX_TO_RX_DLY_UUS); /* Sets delay to turn on receiver after a frame transmission has completed 5.52 api */
    dwt_setrxtimeout(RX_TIMEOUT_UUS); /* Sets the receiver to timeout and disable when no frame is received within the specified time 5.30 api */

    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
        /* Execute a delay between ranging exchanges. */
        sleep_ms(INTER_RANGING_TIME);

        memset((void *) &timestamps, 0, sizeof(struct ts_struct));
        time = get_system_timestamp_u64();
        delayed_tx_time = next_delayed_tx(time, 2000);
        timestamps.tx_timestamp[0] = delayed_tx_timestamp(delayed_tx_time);
        dwt_setdelayedtrxtime(delayed_tx_time);

        /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
        memcpy((void *) &tx1_msg[MSGNO_IDX], (void *) &exchangeNo, sizeof(uint32)); // copy exchange number to message buffer
        memcpy((void *) &tx1_msg[TX1_IDX], (void *) &timestamps.tx_timestamp[0], sizeof(uint64)); // copy tx timestamp

        dwt_writetxdata(sizeof(tx1_msg), tx1_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx1_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

        if (ret == DWT_ERROR) {
            printf("Transmission aborted\n");
            continue;
        }

        printf("INIT %lu scheduled\n", exchangeNo);

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };

        if (status_reg & SYS_STATUS_RXFCG) {

            /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            // the exchange number in received message should match with original exchange number
            memcpy(&rx2_msg[2], &exchangeNo, sizeof(uint32));

            /* Check that the frame is the expected response from the companion "DS TWR responder" example. */
            if (memcmp(rx_buffer, rx2_msg, 6) == 0) {
                printf("RESP %lu received\n", exchangeNo);
                
                /* Retrieve poll transmission and response reception timestamp. */
                timestamps.tx_timestamp[0] = get_tx_timestamp_u64();
                timestamps.rx_timestamp[1] = get_rx_timestamp_u64();

                /* copy remaining timestamps from response message */
                memcpy((void *) &timestamps.rx_timestamp[0], (void *) &rx_buffer[RX1_IDX], sizeof(uint64));
                memcpy((void *) &timestamps.tx_timestamp[1], (void *) &rx_buffer[TX2_IDX], sizeof(uint64));

                // clear all previous information
                memset((void *) rxInfo, 0, sizeof(struct completeChannelInfo));
                // copy diagnostic and other info
                copyChannelInfo(rxInfo, exchangeNo);

                printf("exchange,%lu\n",  exchangeNo);
                printf("TX1,%llu\n", timestamps.tx_timestamp[0]);
                printf("RX1,%llu\n", timestamps.rx_timestamp[0]);
                printf("TX2,%llu\n", timestamps.tx_timestamp[1]);
                printf("RX2,%llu\n", timestamps.rx_timestamp[1]);

                clockOffset = get_clock_offset(rxInfo->carrierintegrator);
                printf("clockOffset,%f\n", clockOffset);

                tof = get_tof(&timestamps, get_clock_offset(rxInfo->carrierintegrator));
                printf("TOF,%f\n", tof*1e9);
                printf("dist,%3.2f\n", tof*SPEED_OF_LIGHT);

                snprintf(filename, 31, "exp%lu_msg%lu_%c.csv", exp_number, exchangeNo, mode);
                printf("start writing to %s...\n", filename);

                /***** FILE OPERATIONS *****/
                saveChannelInfoToFile(filename, rxInfo, &timestamps, 1);
                
                printf("done writing\n");
                printf("\n");
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
        exchangeNo++;

    }

    free(rxInfo);

}

static void responderTask(unsigned long exp_number, char mode) {

    uint32 exchangeNo;
    uint32 frame_len;
    uint32 status_reg = 0;
    int ret;
    // uint64 time;
    uint32 delayed_tx_time;
    struct completeChannelInfo *rxInfo;
    struct ts_struct timestamps;
    char filename[32];
    
    /* Frames used in the ranging process. See NOTE 2 below. */
    uint8 rx1_msg[] = {0xab, 0x00}; // actual size = 1+1+4+8+8+2 = 24
    uint8 tx2_msg[] = {0xab, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // size = 1+1+4+8+8+2 = 24
    uint8 rx_buffer[RX_BUF_LEN]; // same size as rx2

    /* RESPONDER */
    printf("Starting RESPONDER\n");

    memset((void *) &timestamps, 0, sizeof(struct ts_struct));

    // allocate memory
    rxInfo = (struct completeChannelInfo *) malloc(sizeof(struct completeChannelInfo));
    if (rxInfo == NULL) {
        printf("Error: could not allocate memory\n");
        exit(1);
    }

    /* Loop forever responding to ranging requests. */
    while (1) {

        memset((void *) &timestamps, 0, sizeof(struct ts_struct));

        /* Start listening forever */
        dwt_setrxtimeout(0);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) { };

        if (status_reg & SYS_STATUS_RXFCG) {

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            } else {
                printf("wrong frame size\n");
            }

            /* Check that the frame is the correct message by looking at the header */
            if (memcmp((void*) rx_buffer, (void *) rx1_msg, 2) == 0) {

                memcpy((void *) &exchangeNo, (void *) &rx_buffer[MSGNO_IDX], sizeof(uint32));    // get exchange number
                memcpy((void *) &timestamps.tx_timestamp[0], &rx_buffer[TX1_IDX], sizeof(uint64)); // get TX timestamp from message

                printf("INIT %lu received\n", exchangeNo);
                
                /* Retrieve poll reception timestamp. */
                timestamps.rx_timestamp[0] = get_rx_timestamp_u64();

                /* Set send time for response. See NOTE 9 below. */
                delayed_tx_time = next_delayed_tx(timestamps.rx_timestamp[0], RX_TO_TX_DLY_UUS);
                timestamps.tx_timestamp[1] = delayed_tx_timestamp(delayed_tx_time);

                dwt_setdelayedtrxtime(delayed_tx_time);

                memcpy((void *) &tx2_msg[MSGNO_IDX], (void *) &exchangeNo, sizeof(uint32));
                memcpy((void *) &tx2_msg[RX1_IDX], (void *) &timestamps.rx_timestamp[0], sizeof(uint64));
                memcpy((void *) &tx2_msg[TX2_IDX], (void *) &timestamps.tx_timestamp[1], sizeof(uint64));

                dwt_writetxdata(sizeof(tx2_msg), tx2_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx2_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
                ret = dwt_starttx(DWT_START_TX_DELAYED);

                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
                if (ret == DWT_ERROR)
                {
                    printf("RESP %lu abandoned\n", exchangeNo);
                    continue;
                }

                printf("RESP %lu sent\n", exchangeNo);

                /* Poll DW1000 until TX frame sent event set. */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) { };

                /* Clear TX frame sent event. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                memset((void *) rxInfo, 0, sizeof(struct completeChannelInfo));
                copyChannelInfo(rxInfo, exchangeNo);

                printf("exchange,%lu\n",  exchangeNo);
                printf("TX1,%llu\n", timestamps.tx_timestamp[0]);
                printf("RX1,%llu\n", timestamps.rx_timestamp[0]);
                printf("TX2,%llu\n", timestamps.tx_timestamp[1]);

                snprintf(filename, 31, "exp%lu_msg%lu_%c.csv", exp_number, exchangeNo, mode);
                printf("start writing to %s...\n", filename);

                /***** FILE OPERATIONS *****/

                saveChannelInfoToFile(filename, rxInfo, &timestamps, 2);
                printf("done writing\n");
                printf("\n");
            }
        } else {

            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            printf("RX error\n");

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
        }
    }
    free(rxInfo);
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

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_system_timestamp_u64()
 *
 * @brief Get the system in a 64-bit variable.
 *
 * @param  none
 *
 * @return  64-bit value of the system time.
 */
static uint64 get_system_timestamp_u64(void) {
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*
 *  Given current time, find next viable transmit time
 *  
 *  Returns value to program on the DW1000
 */
static uint32 next_delayed_tx(uint64 time, uint32 delay_uus) {
    return ((time + (delay_uus * UUS_TO_DWT_TIME)) >> 8);
}

static uint64 delayed_tx_timestamp(uint32 delayed_tx_time) {
    return ((((uint64)(delayed_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY);
}

static double get_tof(struct ts_struct *timestamps, float clockOffset) {
    double tof;
    int64 rtd_init;
    int64 rtd_resp;
    rtd_init = timestamps->rx_timestamp[1] - timestamps->tx_timestamp[0];
    rtd_resp = timestamps->tx_timestamp[1] - timestamps->rx_timestamp[0];
    tof = (((rtd_init - rtd_resp) * (1 - clockOffset)) / 2.0) * DWT_TIME_UNITS;
    return tof;
}

static float get_clock_offset(int32 carrier_offset) {
    return (carrier_offset * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_2 / 1.0e6));
}

static void copyCIRToBuffer(uint8 *buffer, uint16 len) {
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

static void copyChannelInfo(struct completeChannelInfo *info, uint32 exchange_no) {

    info->exchangeNo = exchange_no;

    /* Copy CIR data */
    copyCIRToBuffer((uint8 *) info->cir, 4*CIR_SAMPLES);
    
    /* Get diagnostic data */
    dwt_readdiagnostics(&info->diagnostics);

    /* read other information */
    info->lde_thresh = dwt_read16bitoffsetreg(LDE_IF_ID, 0x0000);
    info->lde_ppindx = dwt_read16bitoffsetreg(LDE_IF_ID, 0x1000);
    info->lde_ppampl = dwt_read16bitoffsetreg(LDE_IF_ID, 0x1002);
    info->lde_cfg1 = dwt_read8bitoffsetreg(LDE_IF_ID, LDE_CFG1_OFFSET);

    /* Get carrier offset */
    info->carrierintegrator = dwt_readcarrierintegrator();
}

static void saveChannelInfoToFile(char *filename, struct completeChannelInfo *info, struct ts_struct *ts, int type)
{
    FILE *output_file;
    double tof;
    int i;
    float clockOffset;

    output_file = fopen(filename, "w");
    if (output_file == NULL){
        printf("unable to write\n");
    } else {

        clockOffset = get_clock_offset(info->carrierintegrator);
        
        fprintf(output_file, "exchangeNo, %lu\n",    info->exchangeNo);

        if (type == 1) {
            // complete info on initiator
            tof = get_tof(ts, clockOffset);
            fprintf(output_file, "TX1,%llu\n", ts->tx_timestamp[0]);
            fprintf(output_file, "RX1,%llu\n", ts->rx_timestamp[0]);
            fprintf(output_file, "TX2,%llu\n", ts->tx_timestamp[1]);
            fprintf(output_file, "RX2,%llu\n", ts->rx_timestamp[1]);
            fprintf(output_file, "TOF,%f\n", tof*1e9);
            fprintf(output_file, "DIST,%3.2f\n", tof*SPEED_OF_LIGHT);

        } else if (type == 2) {
            // partial infor on responder
            fprintf(output_file, "TX1,%llu\n", ts->tx_timestamp[0]);
            fprintf(output_file, "RX1,%llu\n", ts->rx_timestamp[0]);
            fprintf(output_file, "TX2,%llu\n", ts->tx_timestamp[1]);
        }

        fprintf(output_file, "maxNoise,%u\n", info->diagnostics.maxNoise);
        fprintf(output_file, "stdNoise,%u\n", info->diagnostics.stdNoise);
        fprintf(output_file, "firstPathAmp1,%u\n", info->diagnostics.firstPathAmp1);
        fprintf(output_file, "firstPathAmp2,%u\n", info->diagnostics.firstPathAmp2);
        fprintf(output_file, "firstPathAmp3,%u\n", info->diagnostics.firstPathAmp3);
        fprintf(output_file, "maxGrowthCIR,%u\n", info->diagnostics.maxGrowthCIR);
        fprintf(output_file, "rxPreamCount,%u\n", info->diagnostics.rxPreamCount);
        fprintf(output_file, "firstPath,%u\n", info->diagnostics.firstPath);
        fprintf(output_file, "LDE_THRESH,%u\n", info->lde_thresh);
        fprintf(output_file, "LDE_CFG1,%u\n", info->lde_cfg1);
        fprintf(output_file, "LDE_PPINDX,%u\n", info->lde_ppindx);
        fprintf(output_file, "LDE_PPAMPL,%u\n", info->lde_ppampl);
        fprintf(output_file, "carrierInt,%ld\n", info->carrierintegrator);
        fprintf(output_file, "clockOffset,%f\n", clockOffset);

        fprintf(output_file, "CIRIQ\n");
        for (i = 0; i < CIR_SAMPLES; i++)
        {
            fprintf(output_file, "%d,%d\n", info->cir[i].real, info->cir[i].img);
        }
        
        fclose(output_file);
    }
}


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
