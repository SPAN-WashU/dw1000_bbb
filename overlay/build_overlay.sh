#!/bin/bash

echo "Compiling the overlay from .dts to .dtbo"

dtc -O dtb -o DW1000-SPI0-00A0.dtbo -b 0 -@ DW1000-SPI0-00A0.dts
dtc -O dtb -o DW1000-SPI1-00A0.dtbo -b 0 -@ DW1000-SPI1-00A0.dts
sudo mv *.dtbo /lib/firmware
