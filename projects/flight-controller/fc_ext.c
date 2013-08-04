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

#include <string.h>

#include "fc_gpio.h"
#include "fc_nrf24l01.h"

static VirtualTimer vt;

/*
 * Predeclarations of callback functions
 */
static void extcb1(EXTDriver *extp, expchannel_t channel);
static void extcb2(EXTDriver *extp, expchannel_t channel);
static void extcb5(EXTDriver *extp, expchannel_t channel); /* IRQ line to the NRF24L01 */

/*
 * Structure holding the interrupt settings
 */
static const EXTConfig extcfg = {
  {
   {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, extcb1},
   {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, extcb2},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART, extcb5},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
   {EXT_CH_MODE_DISABLED, NULL},
  },
  EXT_MODE_EXTI(EXT_MODE_GPIOC,
                EXT_MODE_GPIOC,
                0,
                0,
                0,
                EXT_MODE_GPIOC,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0)
};

/* 
 * Setup the external interrupt.
 */
void EXTInit(void)
{
	/*
	 * Setup NRF24L01 IRQ pad.
	 */
	//palSetPadMode(NRF_PORT_CE_IRQ, NRF_PORT_IRQ, PAL_MODE_INPUT_PULLDOWN);	
	palSetPadMode(NRF_PORT_CE_IRQ, NRF_PORT_IRQ, PAL_MODE_INPUT);	

	/*
   * Setup interrupts using the structures above.
	 */
  extStart(&EXTD1, &extcfg);

	/*
	 * Enable interrupts.
   */
	extChannelEnable(&EXTD1, 0);
	extChannelEnable(&EXTD1, 1);
	extChannelEnable(&EXTD1, 5);
}


/* 
 * Triggered when the button is pressed or released. The LED is set to ON.
 */
static void extcb1(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  palClearPad(GPIOB, GPIOB_LED1);
  chSysLockFromIsr();
  if (!chVTIsArmedI(&vt)) {
    uartStartSendI(&UARTD1, 12, "Button 1...\r\n");
    chVTSetI(&vt, MS2ST(200), ledoff1, NULL);
	}
  chSysUnlockFromIsr();
}

/* 
 * Triggered when the LED goes OFF.
 */
static void extcb2(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

  palClearPad(GPIOB, GPIOB_LED1);
  chSysLockFromIsr();
  if (!chVTIsArmedI(&vt)){
    uartStartSendI(&UARTD1, 12, "Button 3...\r\n");
    chVTSetI(&vt, MS2ST(200), ledoff1, NULL);
	}
  chSysUnlockFromIsr();
}

/*
 * Triggered when the NRF24L01 triggers an interrupt
 */
static void extcb5(EXTDriver *extp, expchannel_t channel) {
  (void)extp;
  (void)channel;

	/*
	 * Call interrupt handler
	 */
  chSysLockFromIsr();
	//uartStartSendI(&UARTD1, 12, "NRF IRQ...\r\n");
	NRFReportIRQ();
  chSysUnlockFromIsr();
}
