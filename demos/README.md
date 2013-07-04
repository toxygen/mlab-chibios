# demos

All demos in this directory are provided with built binaries (in build/ dir).

## How

To build demos just type make inside the directory.

Programming the MLAB board is described it README.md in top directory.

Be sure to have a look at ../boards/... config.

### STM32F103-i2c 

Basic demonstration of I2C device communication

### STM32F103-led-blinking

This is the example you want to start with. If this doesn't work, something is wrong with hw.

### STM32F103-sdspi

Demonstration of SD card communication over SPI. Contains basic FAT commands.

### STM32F103-serialshell

Little bit advanced shell (microrl) over UART2.

### STM32F103-usbserial

Basic ChibiOS shell over UART2.

### STM32F103-usbshell

Little bit advanced shell (microrl) over USB.
