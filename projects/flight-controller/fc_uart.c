/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#include "ch.h"
#include "hal.h"

#include "fc_gpio.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/*
 * UART Printf buffer
 */
#define BUF_SIZE	160
static char UARTPrintBuf[BUF_SIZE];
static Semaphore UARTSem;

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
  (void)uartp;
  palClearPad(IOPORT2, GPIOB_LED1);

	// Next line can be printed
  chSysLockFromIsr();
	chSemSignalI(&UARTSem);
  chSysUnlockFromIsr();
}


/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
  (void)uartp;
}


/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {

  (void)uartp;
  (void)e;
}


/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  (void)uartp;
  (void)c;
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;
}

/*
 * UART driver configuration structure.
 * uart_cfg_1 Debug console
 * uart_cfg_2 GPS interface
 */
static UARTConfig uart_cfg_1 = {
  txend1,
  txend2,
  rxend,
  rxchar,
  rxerr,
  //38400,
	115200,
  0,
  USART_CR2_LINEN,
  0
};

static SerialConfig serial_cfg_2 = {
		4800,
		0,
		0,
		0,
};


/*
 * Initialize the serial port
 */
void UARTInit(void)
{
	/*
	 * Initialize the semaphore
	 */
	chSemInit(&UARTSem, 1);

  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  uartStart(&UARTD1, &uart_cfg_1);

	/*
	 * Use serial driver for second uart, because a DMA-bug prevents the use of the UART driver.
	 * TODO When this works well, move all to the serial driver.
	 */
	sdStart(&SD2, &serial_cfg_2);
}


/*
 * Print formatted string to serial port
 */
void UARTPrintf(const char *format, ...)
{
	va_list ap; 
	va_start (ap, format);

	/*
	 * Wait for buffer to become available again.
	 */
	chSemWait(&UARTSem);

	/* 
	 * Build string to send to the buffer.
	 */
	vsnprintf(UARTPrintBuf, BUF_SIZE, format, ap);

	/*
   * Print stuff UART
	 */
	uartStartSend(&UARTD1, strlen(UARTPrintBuf), UARTPrintBuf);
}
