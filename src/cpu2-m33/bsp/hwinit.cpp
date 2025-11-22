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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System status indicator LEDs

GPIOPin g_blueLED(&GPIOJ, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_redLED_n(&GPIOH, 4, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_greenLED(&GPIOD, 8, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_orangeLED(&GPIOJ, 6, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common global hardware config used by both bootloader and application

//UART console
//USART6 kernel clock is 100 MHz, so we need a divisor of 868.05, round to 868
UART<16, 256> g_uart(&USART6, 868);

//APB clocks are not divided, so timer clock equals PCLK (see table 134)
//Divide down to get 10 kHz ticks
Timer g_logTimer(&TIM2, Timer::FEATURE_GENERAL_PURPOSE, 20000);

//Flash controller
OctoSPI_SpiFlashInterface* g_flash = nullptr;

GPIOPin g_user1Button(&GPIOD, 2, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_user2Button(&GPIOG, 8, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level init

void BSP_MainLoopIteration()
{
	/* nothing here */
}

/*
	Initialize on device power domains
 */
void BSP_InitPower()
{
	BSP_InitDebug();

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
}

/**
	@brief Initialize debug stuff
 */
void BSP_InitDebug()
{
	//Turn on debug access as early as possible to enable troubleshooting (if booting from flash)
	switch(SYSCFG.BOOTSR)
	{
		//Development boot, nothing needed
		case BOOT_MODE_DEV1:
		case BOOT_MODE_DEV2:
			break;

		//Otherwise booting from flash
		default:

			//unlock debug in BSEC
			_BSEC.DENR = 0xdeb60fff;

			//enable debug and trace in RCC
			RCC.DBGCFGR |= 0x300;

			for(int i=0; i<16; i++)
				asm("nop");

			//Prevent A35 from sleeping
			//TODO: DBGMCU.CR
			*reinterpret_cast<volatile uint32_t*>(0x4a010004) = 0x00000017;
			break;
	}

	//Turn on the orange LED to show we got this far
	g_orangeLED = 1;
}

void BSP_InitClocks()
{
	//Turn on external oscillator (40 MHz crystal on the HSE)
	RCCHelper::EnableHighSpeedExternalClock();

	//Set PLL4 input to come from the HSE
	RCCHelper::SetPLLInputMux(RCC_MUXSEL_PLL4, RCC_MUXSEL_HSE);

	//Set up PLL4 to run ck_icn_hs_mcu, must be 400 MHz max so run it at exactly 400
	RCCHelper::ConfigureGeneralPLL(
		4,	//PLL4
		1,	//PFD = 40 MHz ref / 1 = 40 MHz
		20,	//VCO = 40 MHz PFD * 20 = 800 MHz
		2,	//Divide 1 = 800 MHz / 2 = 400 MHz
		1);	//Divide 2 = 400 MHz / 1 = 400 MHz

	//ck_icn_ls_mcu is max 200 MHz, so enable the divide-by-2 from ck_icn_hs_mcu so it runs at exactly 200
	//(once we select the PLL, for the moment we're using the HSI clock... but we need dividers set before we switch)
	RCCHelper::SetLowSpeedMCUClockDivider(true);

	//Set all APB clocks to ck_icn_ls_mcu (200 MHz) with no further division
	RCCHelper::SetAPB1ClockDivider(RCC_APB_DIV_1);
	RCCHelper::SetAPB2ClockDivider(RCC_APB_DIV_1);
	RCCHelper::SetAPB3ClockDivider(RCC_APB_DIV_1);
	RCCHelper::SetAPB4ClockDivider(RCC_APB_DIV_1);
	RCCHelper::SetAPBDebugClockDivider(RCC_APB_DIV_1);

	//Crossbar path to the MCU subsystem runs off PLL4 with no further division
	RCCHelper::SetCrossbarDivider(RCC_ck_icn_hs_mcu, RCC_PREDIV_1, 1);
	RCCHelper::SetCrossbarMux(RCC_ck_icn_hs_mcu, RCC_XBAR_PLL4);
}

void BSP_InitUART()
{
	//USART6 clock is 100 MHz max (PLL4 / 4)
	RCCHelper::SetCrossbarDivider(RCC_ck_ker_usart6, RCC_PREDIV_4, 1);
	RCCHelper::SetCrossbarMux(RCC_ck_ker_usart6, RCC_XBAR_PLL4);

	//Initialize the UART for local console: 115.2 Kbps
	//TODO: nice interface for enabling UART interrupts
	GPIOPin uart_tx(&GPIOF, 13, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 3);
	GPIOPin uart_rx(&GPIOF, 14, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_SLOW, 3);

	//Reinitialize the timer because for some reason it doesn't seem to reliably get initialized in the constructor
	//(maybe something isn't set up in RCC yet or something when global constructors are called?)
	g_logTimer.Initialize(20000);

	g_logTimer.Sleep(50);	//wait for UART pins to be high long enough to remove any glitches during powerup

	//Enable the UART interrupt once the poins have stabilized
	NVIC_EnableIRQ(136);
}

void BSP_InitLog()
{
	static LogSink<MAX_LOG_SINKS> sink(&g_uart);
	g_logSink = &sink;

	g_log.Initialize(g_logSink, &g_logTimer);
	g_log("Firmware compiled at %s on %s\n", __TIME__, __DATE__);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common features shared by both application and bootloader

void BSP_Init()
{
	InitGPIOs();
	InitQSPI();

	App_Init();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIOs for LEDs etc

void InitGPIOs()
{
	g_log("Initializing GPIOs\n");

	//Turn off all LEDs
	g_blueLED = 0;
	g_redLED_n = 1;
	g_greenLED = 0;
	//g_orangeLED = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// OCTOSPI for talking to the flash

void InitQSPI()
{
	g_log("Initializing QSPI\n");
	LogIndenter li(g_log);

	//Configure the GPIOs
	static GPIOPin csn(&GPIOD, 3, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 10);
	static GPIOPin sck(&GPIOD, 0, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 10);
	static GPIOPin dq0(&GPIOD, 4, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 10);
	static GPIOPin dq1(&GPIOD, 5, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 10);
	static GPIOPin dq2(&GPIOD, 6, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 10);
	static GPIOPin dq3(&GPIOD, 7, GPIOPin::MODE_PERIPHERAL, GPIOPin::SLEW_FAST, 10);

	//Set pullup on  WP#
	dq2.SetPullMode(GPIOPin::PULL_UP);

	//Initialize the OCTOSPI itself
	RCCHelper::Enable(&OCTOSPIM);

	//Set up clock for the OCTOSPI to 100 MHz (PLL4 / 4), max is 133
	RCCHelper::SetCrossbarDivider(RCC_ck_ker_ospi1, RCC_PREDIV_4, 1);
	RCCHelper::SetCrossbarMux(RCC_ck_ker_ospi1, RCC_XBAR_PLL4);

	g_log("Waiting...\n");
	g_logTimer.Sleep(20000);

	//Set up the QSPI flash to 25 MHz clock (100 MHz / 4)
	static OctoSPI_SpiFlashInterface flash(&OCTOSPI1, 64 * 1024 * 1024, 4);
	flash.Discover();
	g_flash = &flash;
}
