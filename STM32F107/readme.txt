There are 3 ways to program STM32F107:

# 1) Programming through USB (OTG). Difficulty: easy.

# 2) Programming through serial link programmer. Difficulty: medium.

# 3) Programming through JTAG. Difficulty: hard.



## USB (OTG)

/* TODO */

## Serial link

1. cd to project in demos/ directory

2. make

3. stm32flash -w build/ch.bin -v /dev/tty.usbserial-A4009ctC

Mac: "tty.usbserial-A4009ctC" is created by virtual com driver and has most probably little different name in your case

Linux: the device is named differently, i.e. /dev/ttyS0.

## JTAG

Build OpenOCD 

''
./configure --enable-ft2232_ftd2xx openocd -f busblaster.cfg -f /usr/local/share/openocd/scripts/target/stm32f1x.cfg
''

busblaster.cfg
'' 
interface ft2232
ft2232_device_desc "Dual RS232-HS"
ft2232_layout jtagkey
ft2232_vid_pid 0x0403 0x6010
''

follow: http://pramode.net/fosstronics/stm32-circle.txt
