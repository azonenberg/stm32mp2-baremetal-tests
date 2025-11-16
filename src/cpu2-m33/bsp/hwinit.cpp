/***********************************************************************************************************************
*                                                                                                                      *
* stm32mp2-baremetal-tests                                                                                             *
*                                                                                                                      *
* Copyright (c) 2025 Andrew D. Zonenberg and contributors                                                              *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author	Andrew D. Zonenberg
	@brief	Boot-time hardware initialization
 */
#include <core/platform.h>
#include "hwinit.h"
//#include <peripheral/Power.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System status indicator LEDs
/*
GPIOPin g_pgoodLED(&GPIOB, 9, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_faultLED(&GPIOB, 8, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_sysokLED(&GPIOH, 0, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common global hardware config used by both bootloader and application

//UART console
//USART1 is on APB1 (80 MHz), so we need a divisor of 694.44, round to 694
//UART<16, 256> g_uart(&USART1, 694);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level init

void BSP_MainLoopIteration()
{

}

/*
	Initialize the
 */
void BSP_InitPower()
{
	//VDDIO1/2/3/4 should be valid by the time we get to this point but make sure
	Power::EnableVDDIO1Monitor();
	while(!Power::IsVDDIO1Ready())
	{}
	Power::SetVDDIO1Valid();

	Power::EnableVDDIO2Monitor();
	while(!Power::IsVDDIO2Ready())
	{}
	Power::SetVDDIO2Valid();

	Power::EnableVDDIO3Monitor();
	while(!Power::IsVDDIO3Ready())
	{}
	Power::SetVDDIO3Valid();

	Power::EnableVDDIO4Monitor();
	while(!Power::IsVDDIO4Ready())
	{}
	Power::SetVDDIO4Valid();

	//Enable the backup domain
	Power::EnableBackupSramWrites();

	//TODO: initialize power for the ADCs

	//DEBUG: Turn on a bunch of LEDs
	GPIOPin blue(&GPIOJ, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin red_n(&GPIOH, 4, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin green(&GPIOD, 8, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin orange(&GPIOJ, 6, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	blue = 1;
	red_n = 0;
	green = 1;
	orange = 1;
	asm("dsb");
}

void BSP_InitClocks()
{
	while(1)
	{}
}

void BSP_InitUART()
{
	while(1)
	{}
	/*
	//Initialize the UART for local console: 115.2 Kbps
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOA, 9, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);
	GPIOPin uart_rx(&GPIOA, 10, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 7);

	g_logTimer.Sleep(10);	//wait for UART pins to be high long enough to remove any glitches during powerup

	//Enable the UART interrupt
	NVIC_EnableIRQ(37);
	*/
}

void BSP_InitLog()
{
	while(1)
	{
	}
	/*
	static LogSink<MAX_LOG_SINKS> sink(&g_cliUART);
	g_logSink = &sink;

	g_log.Initialize(g_logSink, &g_logTimer);
	g_log("Firmware compiled at %s on %s\n", __TIME__, __DATE__);
	*/
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common features shared by both application and bootloader

void BSP_Init()
{
	InitGPIOs();
	App_Init();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIOs for LEDs etc

void InitGPIOs()
{
	while(1)
	{}
	/*
	g_log("Initializing GPIOs\n");

	//turn off all LEDs
	g_pgoodLED = 0;
	g_faultLED = 0;
	g_sysokLED = 0;

	//Set up GPIOs for I2C bus
	static GPIOPin i2c_scl(&GPIOB, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);
	static GPIOPin i2c_sda(&GPIOB, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 4, true);

	//Turn off DUT power
	g_dutVddEn = 0;
	g_dutVccioEn = 0;
	*/
}
