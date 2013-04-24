# MLAB configured ChibiOS

![ChibiOS logo](http://www.chibios.org/dokuwiki/lib/exe/fetch.php?cache=&media=logo.png "ChibiOS")
![UST logo](http://www.ust.cz/include/Logo_UST.png "UST")

## What
ChibiOS is an open-source real-time operating system. 

This repository provides ChibiOS together with the board configurations for the MLAB modules.

## Supported modules
###[STM32F10xRxT](http://www.ust.cz/shop/product_info.php?cPath=22_36&products_id=84)

## Howto

1. cd to project in demos/ directory

2. make

3. cd build 

4. stm32flash -w ch.bin -v /dev/tty.usbserial-A4009ctC 

Mac: "tty.usbserial-A4009ctC" is created by virtual com driver and has most probably little different name in your case

Linux: the device is named differently, i.e. /dev/ttyS0.

