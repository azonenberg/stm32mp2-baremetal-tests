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
#include <multicore/MulticoreLogDevice.h>
#include <peripheral/PageTable.h>

/*
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// System status indicator LEDs

GPIOPin g_blueLED(&GPIOJ, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_redLED_n(&GPIOH, 4, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_greenLED(&GPIOD, 8, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
GPIOPin g_orangeLED(&GPIOJ, 6, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
*/

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common global hardware config used by both bootloader and application

//APB clocks are not divided, so timer clock equals PCLK (see table 134)
//Divide down to get 10 kHz ticks
Timer g_logTimer(&TIM3, Timer::FEATURE_GENERAL_PURPOSE, 20000);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Page tables

const uint64_t leafPageSize = 2*1024*1024;

/*
	Level 1 page table, each entry is 1GB (0x4000_0000).
	Can cover up to 512GB but we only fill out the first 16 entries because we only have 34 bits of virtual
	address space enabled in TCR_EL3.
 */
PageTable<512*leafPageSize, 0x0000'0000, 16> g_level1PageTable __attribute__((aligned(4096)));

//Level 2 page table covering 0000_0000 to 3fff_ffff, each entry is 2 MB (0x20_0000)
//Mostly on chip SRAM but also covers PCIe BAR region
PageTable<leafPageSize, 0x0000'0000> g_level2PageTable_0to3 __attribute__((aligned(4096)));

//Level 2 page table covering 4000_0000 to 7fff_ffff, each entry is 2 MB (0x20_0000)
//Peripherals, OCTOSPI, FMC NOR
PageTable<leafPageSize, 0x4000'0000> g_level2PageTable_4to7 __attribute__((aligned(4096)));

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Low level init

void BSP_MainLoopIteration()
{
	/* nothing here */
}

void BSP_InitPower()
{
	//Looks like hardware/bootrom does all of this for us on the A35, nothing needed on our side
	//TODO: any debug init needed to turn on trace or something? Do we have to set up somethign on the PMIC?
}

void BSP_InitMemory()
{
	//handled by the M33
}

void BSP_InitPageTables()
{
	//Level 2 page table for on chip SRAM
	g_level2PageTable_0to3.Clear();
	//Do not create a mapping for ROM, we want it unmapped so null pointers segfault!
	g_level2PageTable_0to3.SetLeafEntry(0x0a00'0000, 0x0a00'0000, false, MAIR_IDX_NORMAL);	//On chip SRAM, boot/nonsec
	g_level2PageTable_0to3.SetLeafEntry(0x0e00'0000, 0x0e00'0000, false, MAIR_IDX_NORMAL);	//On chip SRAM, boot/sec
	//TODO: PCIe BAR region at 0x1000'0000 to 1fff'ffff
	g_level2PageTable_0to3.SetLeafEntry(0x2000'0000, 0x2000'0000, true, MAIR_IDX_NORMAL);	//On chip SRAM, nonsec, NX
	g_level2PageTable_0to3.SetLeafEntry(0x3000'0000, 0x3000'0000, true, MAIR_IDX_NORMAL);	//On chip SRAM, sec, NX

	//Level 2 page table for peripherals, OCTOSPI, FMC
	g_level2PageTable_4to7.Clear();
	for(uint64_t base=0x4000'0000; base<0x6000'0000; base += g_level2PageTable_4to7.GetEntrySize())	//SFRs are NX device
		g_level2PageTable_4to7.SetLeafEntry(base,  base, true, MAIR_IDX_DEVICE);
	//Don't create mappings for OCTOSPI and FMC at this stage since we don't use those

	//Set up level 1 page table pointing to the level 2 tables
	g_level1PageTable.Clear();
	g_level1PageTable.SetChildEntry(0x0000'0000, g_level2PageTable_0to3);
	g_level1PageTable.SetChildEntry(0x4000'0000, g_level2PageTable_4to7);
	//We don't have DDR set up yet so leave those entries as blank (fault)
	//as well as anything past the end of DDR
}

extern "C" void BSP_InitMMU()
{
	//Turn on the MMU
	InitializeMMU(g_level1PageTable.GetData());
}

void BSP_InitClocks()
{
	GPIOPin g_greenLED(&GPIOD, 8, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin g_blueLED(&GPIOJ, 7, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	GPIOPin g_orangeLED(&GPIOJ, 6, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);

	//Turn on the green LED
	g_greenLED = 1;

	//Select PLL1 clock source as the HSE clock (ck_pll1_ref)
	RCCHelper::SetPLLInputMux(RCC_MUXSEL_PLL1, RCC_MUXSEL_HSE);

	//Turn on the CPU PLL1
	{
		//Select the external clock source as the CPU1 clock source so we can reconfigure the PLL
		CA35SS.CHGCLKREQ.set = 1;
		while( (CA35SS.CHGCLKREQ.reg & 2) != 2)
		{}

		//Turn the orange LED off
		g_orangeLED = 0;

		//Hold the PLL in reset
		CA35SS.PLL_EN.clear = 4;

		/*
			Default configuration at reset: FREQ1 = 2004b, FREQ2 = 0e
			This translates to prediv=2, fbdiv=75, postdiv2 = 1, postdiv1 = 6
			so total of /2 *75 /6 = 250 MHz CPU and 1500 MHz VCO from a 40 MHz input
		 */

		//Keep the same VCO settings as default: divide to 20 MHz PFD, multiply to 1.5 GHz VCO
		CA35SS.PLL_FREQ1.reg = (2 << 16) | 75;

		//Post-divide by 1 for now to give 1.5 GHz CPU
		CA35SS.PLL_FREQ2.reg = 9;

		//Start the PLL
		CA35SS.PLL_EN.set = 1;

		//Wait for it to lock
		while( (CA35SS.PLL_EN.reg & 1) == 0)
		{}

		//Release PLL reset
		CA35SS.PLL_EN.set = 4;

		//Select PLL as clock source
		CA35SS.CHGCLKREQ.clear = 1;

		//Wait for ack
		while( (CA35SS.CHGCLKREQ.reg & 2) != 0)
		{}
	}

	g_blueLED = 1;
}

void BSP_InitUART()
{
	//nothing needed, uart is on the m33
}

void BSP_InitLog()
{
	static MulticoreLogDevice logdev;
	logdev.LookupChannel(0, "log.a35-0");
	logdev.LookupChannel(1, "log.a35-1");

	//Sync the timers
	TIM3.CNT = TIM2.CNT;

	static LogSink<MAX_LOG_SINKS> sink(&logdev);
	g_logSink = &sink;

	g_log.Initialize(g_logSink, &g_logTimer);
	g_log("Firmware compiled at %s on %s\n", __TIME__, __DATE__);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common features shared by both application and bootloader

void BSP_Init()
{
	App_Init();
}
