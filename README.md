# dw1000_bbb
DW1000 &amp; BBB App

Please download DW1000 API from http://www.decawave.com/support/software. Copy
api files (decadriver) to src. Make and enjoy!

# Instructions

# Quick

    cd overlay
    ./build_overlay
    cd ..
    src/setup_dw1000.sh
    make clean
    make <app_name>

## Compiling and Loading the overlay

The SPIDEV module we use requires a custom device tree overlay to be built and loaded. The device tree compile script is `overlay/build_overlay.sh`. This generates the overlay and copies it to `/lib/firmware`.

Load the device tree overlay from `/lib/firmware`. This is required by the SPIDEV module

    echo DW1000-SPI0 > /sys/devices/platform/bone_capemgr/slots

 `src/setup_dw1000.sh` is a script to automate this. Run this once at device startup.

## Compile

    make <application_name>

The following applications are currently implemented:

1. `dw1000_tx`: simple periodic transmitter. Takes no parameters.
2. `dw1000_rx`: simple receiver that continuously listens for packets. Takes no parameters.
3. `dw1000_rx_cir`: like simple receiver but also outputs the CIR (channel impulse respone) for each reception to the console. Takes no parameters.
4. `dw1000_twr_resp`: adapted ranging application using one polling and one response message. Also outputs the entire CIR in a file. Takes 2 parameters:
5. 
    - `INIT` or `RESP`: Choose which device is the INITIATOR or RESPONDER
    - `<exp_number>`: An unsigned integer used for naming the output.
    
    Output files have the naming scheme `exp<exp_number>_msg<msg_number>_I/R.csv`. The `<msg_number>` is modulo 256.

# Known Quirks

1. The Beaglebone make system does not like long file names (>16 characters). Make fails to behave properly with long file names.
2. The beaglebone SPIDEV based SPI device can only transfer a limited number of bytes per transaction. This number is somewhere close to 128 (not exactly sure). In cases where more successive bytes have to be read, try to split the transfer operation into multiple chunks.

# Code Sources

The Decawave Software API is provided by Decawave Ltd. Please refer to `Decawave_Disclaimer.txt` for further details
