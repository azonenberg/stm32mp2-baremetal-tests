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
#include <core/LogFlushTask.h>
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

GPIOPin g_triggerOut(&GPIOG, 5, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_FAST);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Logging

MulticoreLogDevice g_logDevice;

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
	for(uint64_t base=0x1000'0000; base < 0x2000'0000; base += g_level2PageTable_0to3.GetEntrySize())	//PCIe
		g_level2PageTable_0to3.SetLeafEntry(base, base, true, MAIR_IDX_DEVICE);

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
	g_logDevice.LookupChannel(0, "log.a35-0");
	g_logDevice.LookupChannel(1, "log.a35-1");

	//Sync the timers
	TIM3.CNT = TIM2.CNT;

	static LogSink<MAX_LOG_SINKS> sink(&g_logDevice);
	g_logSink = &sink;

	g_log.Initialize(g_logSink, &g_logTimer);
	g_log("Firmware compiled at %s on %s\n", __TIME__, __DATE__);

	//Set up the flusher
	static LogFlushTask flushTask0(g_logDevice);
	static LogFlushTask flushTask1(g_logDevice);
	g_tasks[0].push_back(&flushTask0);
	g_tasks[1].push_back(&flushTask1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Common features shared by both application and bootloader

void BSP_Init()
{
	InitPCIe();
	App_Init();
}

void EnumeratePCIe();
void PrintPCIeDevice(volatile uint32_t* ptr);
void PrintCapabilities(volatile uint8_t* basePtr, uint8_t capsOffset);
void PrintPCIeCapabilities(volatile pcie_pcie_cap_t* pcap);

void InitPCIe()
{
	g_log("Initializing PCIe\n");
	LogIndenter li(g_log);

	//Must be pulled high card side to indicate wake capable. We don't care about this, so leave them unused
	//GPIOPin nwake(&GPIOH, 5, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	//GPIOPin clkreq(&GPIOJ, 0, GPIOPin::MODE_INPUT, GPIOPin::SLEW_SLOW);
	g_triggerOut = 0;

	//Set up the PCIe protocol stack
	//TODO: troubleshoot 5 GT/s link startup, see if it actually works as we expect
	g_log("PHY init\n");
	PCIE::Initialize(
		PCIE::REFCLK_EXT_100MHZ,	//REFCLK_INT_25MHZ doesn't work reliably on the EV1
		false);						//Not training to full speed on link up for now, stay at 2.5 GT/s

	//Reset the link partner
	g_log("Reset device\n");
	g_logDevice.Flush();
	GPIOPin perst(&GPIOJ, 8, GPIOPin::MODE_OUTPUT, GPIOPin::SLEW_SLOW);
	perst = 0;
	g_logTimer.Sleep(5);
	perst = 1;
	g_logTimer.Sleep(1000);	//wait 100ms for things to come up

	//Wait for PCIESR1 to show phy and data link up
	PCIE::WaitForLinkUp();
	g_log("PHY and data link layer up\n");

	//TODO: configure driver, impedance, etc
	EnumeratePCIe();
}

//DEBUG this shouldnt be global
uint32_t nextBus = 1;
uint32_t thisBus = 0;

void EnumeratePCIe()
{
	g_log("Bus enumeration\n");
	LogIndenter li(g_log);

	//Root complex is PCI compatible address space at 0x58400000 (4 MB in size not 256)
	//Touching the high half of it seems to hang
	auto base = reinterpret_cast<uint32_t*>(0x58400000);

	/*
		Read extra synopsys registers

		Reference:
		* https://github.com/u-boot/u-boot/blob/master/drivers/pci/pcie_dw_common.c
		* https://github.com/u-boot/u-boot/blob/master/drivers/pci/pcie_dw_common.h

		LINK_STATUS at 0x80
		PORT_LINK_CONTROL at 0x710
		LINK_WIDTH_SPEED_CONTROL at 0x80c
		MISC_CONTROL at 0x8bc (bit 0 is DBI_RO_WR_EN)

		ATU_BASE is DBI_BASE + 0x30_0000? when dumped there's some 00s and ffs so something is there

		See https://wiki.st.com/stm32mpu/wiki/PCIe_device_tree_configuration
		DBI seems to be 5840_0000 (or 4840 for nonsecure)
		Config seems to be 1000_0000?? so far this is all zeroes though
	 */
	g_log("Dumping Synopsys registers\n");
	{
		uint32_t link_status = base[0x80 / 4];
		uint32_t port_link_ctl = base[0x710 / 4];
		uint32_t link_speed = (link_status >> 16) & 0xf;
		uint32_t link_width = (link_status >> 20) & 0xf;
		uint32_t link_width_speed_ctrl = base[0x80c / 4];
		uint32_t misc_ctrl = base[0x8bc / 4];
		LogIndenter li2(g_log);
		g_log("Link status:           %08x (speed %d, width %d)\n", link_status, link_speed, link_width);
		g_log("Port link ctrl:        %08x\n", port_link_ctl);
		g_log("Link width/speed ctrl: %08x\n", link_width_speed_ctrl);
		g_log("Misc ctrl:             %08x\n", misc_ctrl);

		uint32_t versionNum = base[0x8f8 / 4];
		uint32_t versionType = base[0x8fc / 4];
		g_log("IP version:            %c.%c%c (%c%c)\n",
			(versionNum >> 24),
			(versionNum >> 16) & 0xff,
			(versionNum >> 8) & 0xff,
			(versionType >> 24) & 0xff,
			(versionType >> 16) & 0xff);

		if(base[0x900/4] == 0xffffffff)
			g_log("iATU configured in unrolled mode, viewport registers not present\n");
	}

	//unlock config
	PCIE::UnlockConfig();

	//Set command register
	//Enable SERR, bus mastering, IO space, memory space
	base[0x4 / 4] = (base[0x4 / 4] & 0xffff0000) | (0x507);

	//Set root complex BAR, TODO what's there?
	base[0x10/4] 	= 0x1010'0000;
	base[0x14 / 4]	= 0x1002'0000;

	//Assign bus numbers for the root complex
	//Subordinate 255, secondary 01, primary 00
	base[0x18 / 4] = (base[0x18 / 4] & 0xff000000) | 0xff0100;

	//Secondary status, IO limit, IO base
	base[0x1c / 4] = 0xf0;

	//Set memory limit, prefetchable limit
	base[0x20 / 4] = 0x10201020;
	base[0x24 / 4] = 0x0001fff1;

	//bridge control
	base[0x3c / 4] = 0x02014f;

	//Set device class??
	base[0x08 / 4] = (base[0x08 / 4] & 0xffff) | (0x0604 << 16);

	//re lock
	PCIE::LockConfig();

	//Print the root device
	uint32_t bus = 0;
	uint32_t device = 0;
	uint32_t function = 0;
	{
		auto ptr = base + bus*0x40000 + device*0x2000 + function*0x400;
		g_log("%02x:%02x.%d (%08x)\n", bus, device, function, ptr);
		PrintPCIeDevice(ptr);
	}

	//iATU region 0: configuration access
	//Map this near the very end of the PCIe region so it's not in the way of anything else
	PCIE::SetupOutboundATURegion(
		0,							//ATU region
		PCIE_TLP_TYPE_CONFIG,		//Convert to configuration TLP
		0x1fff'0000,				//CPU address start
		0x1fff'ffff,				//CPU address end
		0x0100'0000);				//BDF base: 01:00.0

	//Inbound region 0: DMA reads from peripheral I guess? We can tweak this later
	/*
	PCIE::SetupInboundATURegion(
		0,
		PCIE_TLP_TYPE_NORMAL,
		0x8000'0000,				//start of DRAM
		0xffff'ffff,				//end of lower 2GB of DRAM
		0x8000'0000);				//passthrough mapping
	*/

	//Configure the second iATU region as passthrough
	PCIE::SetupOutboundATURegion(
		1,							//ATU region
		PCIE_TLP_TYPE_NORMAL,		//Convert to normal memory access TLP
		0x1000'0000,				//CPU address start
		0x17ff'ffff,				//CPU address end
		0x0000'0000);				//Bus address
	PCIE::ClearInboundATURegion(1);

	//Not using regions 2/3 in either direction
	PCIE::ClearOutboundATURegion(2);
	PCIE::ClearInboundATURegion(2);
	PCIE::ClearOutboundATURegion(3);
	PCIE::ClearInboundATURegion(3);

	/*
	//Configure the third iATU region to match Linux
	PCIE::SetupOutboundATURegion(
		2,							//ATU region
		PCIE_TLP_TYPE_NORMAL,		//Convert to normal memory access TLP
		0x1800'0000,				//CPU address start
		0x1fff'ffff,				//CPU address end
		0x1800'0000);				//Bus address


	//Fourth iATU region
	PCIE::SetupOutboundATURegion(
		3,							//ATU region
		PCIE_TLP_TYPE_IO,			//Convert to I/O TLP
		0x1001'0000,				//CPU address start
		0x1001'ffff,				//CPU address end
		0x1001'0000);				//Bus address
	*/

	//Enumerate it
	volatile uint32_t* iobase = reinterpret_cast<volatile uint32_t*>(0x1fff'0000);
	volatile uint32_t* membase = reinterpret_cast<volatile uint32_t*>(0x1000'0000);
	g_log("%02x:%02x.%d (%08x)\n", 1, 0, 0, iobase);
	PrintPCIeDevice(iobase);

	//Trigger the scope
	g_triggerOut = 1;

	/*
	//Send a test write to each iATU region
	iobase[0x0000'0000/4] = 0xdeadbeef;
	asm("dmb st");
	iobase[0x0002'0000/4] = 0xfeedface;
	asm("dmb st");
	//g_log("Readback: %08x\n", iobase[0]);
	*/

	//Do a large block write
	uint32_t foo[4] =
	{
		0x11223344,
		0x55667788,
		0x99aabbcc,
		0xddeeff00
	};
	memcpy((uint32_t*)membase, foo, sizeof(foo));
}

/**
	@brief Print out the PCIe configuration space

	//We have only one bus, can have up to 32 devices / 8 functions / 4 kB each
	//so incrementing function is +4 kB
	//incrementing device is +32 kB = 0x8000
 */
void PrintPCIeDevice(volatile uint32_t* ptr)
{
	LogIndenter li2(g_log);

	auto base = reinterpret_cast<volatile pcie_cfg_t*>(ptr);

	uint16_t vid = base->vendor;
	const char* svid = "unknown";
	switch(vid)
	{
		case 0x1c5c:
			svid = "SK hynix";
			break;

		case 0x16c3:
			svid = "Synopsys, Inc.";
			break;
	}
	g_log("%04x:%04x (%s)\n", vid, base->device, svid);

	/*
	//Enable IO space access
	//ptr[1] = 0x7;

	uint32_t scmd = ptr[1];
	g_log("Status / Command: %08x\n", scmd);
	if(scmd & 0x00100000)
		g_log("Extended capabilities present\n");
	uint32_t class_rev = ptr[2];
	g_log("Base class %d, subclass %d, program interface %d, revision %d\n",
		class_rev >> 24,
		(class_rev >> 16) & 0xff,
		(class_rev >> 8) & 0xff,
		(class_rev >> 0) & 0xff);*/
	auto headerType = base->headerType;
	g_log("Header type %d\n", headerType & 0x7f);
	g_log("BAR0 %08x\n", ptr[4]);
	g_log("BAR1 %08x\n", ptr[5]);

	if(headerType & 0x80)
	{
		g_log("Multifunction devices not supported, skipping\n");
		return;
	}

	if(headerType == 1)
	{
		g_log("Device is a PCIe bridge or root complex\n");

		//configure it if not already configured
		//if(ptr[6] == 0)
		//	ptr[6] = 0x00ff0100;

		//Assign it bus number 1 on the secondary and subordinate side
		//ptr[6] = 0x00ff0000 | (nextBus << 8) | thisBus;
		//thisBus ++;
		//nextBus ++;

		uint32_t busid = ptr[6];
		g_log("Primary bus number %d, secondary bus number %d, subordinate bus number %d\n",
			(busid >> 0) & 0xff,
			(busid >> 8) & 0xff,
			(busid >> 16) & 0xff);
		g_log("Secondary status: %04x\n", ptr[7] >> 16);

		uint32_t mem = ptr[8];
		g_log("Memory limit = %04x, memory base = %04x\n",
			mem >> 16, mem & 0xffff);
	}

	/*
	else
	{
		uint32_t sid = ptr[11];
		uint16_t subsystem = sid >> 16;
		uint16_t svendor = sid & 0xff;
		g_log("Subsystem %04x, vendor %04x\n", subsystem, svendor);
	}
	*/

	//Walk the capabilities
	PrintCapabilities((volatile uint8_t*)ptr, base->type0.capsPtr);
}

void PrintCapabilities(volatile uint8_t* basePtr, uint8_t capsOffset)
{
	//Get pointer to the current capability
	auto pcap = reinterpret_cast<volatile pcie_cap_t*>(basePtr + capsOffset);

	//Print top level capability info
	uint8_t captype = pcap->type;
	uint8_t nextcap = pcap->nextcap;
	const char* stype = "unknown";
	switch(captype)
	{
		case PCIE_CAPTYPE_PM:
			stype = "PCI Power Management";
			break;

		case PCIE_CAPTYPE_MSI:
			stype = "Message Signaled Interrupts";
			break;

		case PCIE_CAPTYPE_PCIE:
			stype = "PCI Express";
			break;

		case PCIE_CAPTYPE_MSI_X:
			stype = "MSI-X";
			break;
	}
	g_log("[%02x]: %s (%02x), next cap %02x, cap reg %04x\n",
		capsOffset, stype, captype, nextcap, pcap->capreg);

	//Print more details
	switch(captype)
	{
		case PCIE_CAPTYPE_PM:
		case PCIE_CAPTYPE_MSI:
		case PCIE_CAPTYPE_MSI_X:
			//TODO print stuff
			break;

		case PCIE_CAPTYPE_PCIE:
			PrintPCIeCapabilities(reinterpret_cast<volatile pcie_pcie_cap_t*>(pcap));
			break;
	}

	//Print next capability if there is one
	if(nextcap > capsOffset)
		PrintCapabilities(basePtr, nextcap);
}

void PrintPCIeCapabilities(volatile pcie_pcie_cap_t* pcap)
{
	LogIndenter li(g_log);

	//Base
	auto capreg = pcap->base.capreg;
	auto dtype = (capreg >> 4) & 0xf;
	const char* dstype = "unknown";
	switch(dtype)
	{
		case PCIE_PTYPE_ENDPOINT:
			dstype = "Endpoint";
			break;

		case PCIE_PTYPE_ROOT_OF_ROOT:
			dstype = "Root port of root complex";
			break;
	}
	g_log("Device/port type          %d (%s)\n", dtype, dstype);

	{
		LogIndenter li3(g_log);
		if(capreg & 0x100)
			g_log("Slot implemented\n");
	}

	auto devcaps = pcap->device_caps;
	g_log("Device capabilities       %08x\n", devcaps);
	{
		LogIndenter li3(g_log);
		g_log("Max payload size      %d\n", (128 << (devcaps & 7)));
		if(devcaps & PCIE_DEVCAPS_FLR)
			g_log("Function level reset available\n");
	}

	//TODO: print device ctrl/status

	auto linkcaps = pcap->link_caps;
	g_log("Link capabilities         %08x\n", linkcaps);
	{
		LogIndenter li3(g_log);
		uint8_t speedBitmask = linkcaps & 0xf;
		uint8_t maxSpeed = 0;
		switch(speedBitmask)
		{
			case 1:
				maxSpeed = 25;
				break;

			case 2:
				maxSpeed = 50;
				break;

			case 3:
				maxSpeed = 80;
				break;
		}
		g_log("Port number           %d\n", linkcaps >> 24);
		g_log("Max width             x%d\n", (linkcaps >> 4) & 0x3f);
		g_log("Max speed             %d.%d GT/s\n", maxSpeed/10, maxSpeed % 10);
	}

	/*
	g_log("Link ctrl/status          %08x\n", pcaps[4]);
	{
		LogIndenter li3(g_log);
		uint32_t linkStatus = pcaps[4] >> 16;
		//uint32_t linkCtrl = pcaps[4] & 0xffff;

		uint8_t currentSpeedBitmask = linkStatus & 0xf;
		uint8_t currentWidth = (linkStatus >> 4) & 0x3f;

		uint8_t currentSpeed = 0;
		switch(currentSpeedBitmask)
		{
			case 1:
				currentSpeed = 25;
				break;

			case 2:
				currentSpeed = 50;
				break;

			case 3:
				currentSpeed = 80;
				break;
		}
		if(linkStatus & 0x2000)
			g_log("Data link layer active\n");
		g_log("Negotiated width      x%d\n", currentWidth);
		g_log("Negotiated speed      %d.%d GT/s\n", currentSpeed/10, currentSpeed % 10);
	}

	g_log("Slot capabilities         %08x\n", pcaps[5]);
	g_log("Slot control/status       %08x\n", pcaps[6]);
	//Turn on CSR Software Visibility
	pcaps[7] = 0x10;
	g_log("Root control/capabilities %08x\n", pcaps[7]);
	g_log("Root status               %08x\n", pcaps[8]);
	g_log("Device capabilities 2     %08x\n", pcaps[9]);
	g_log("Device control 2          %08x\n", pcaps[10]);
	g_log("Link capabilities 2       %08x\n", pcaps[11]);
	{
		LogIndenter li3(g_log);
		if(pcaps[11] & 2)
			g_log("2.5 GT/s supported\n");
		if(pcaps[11] & 4)
			g_log("5 GT/s supported\n");
		if(pcaps[11] & 8)
			g_log("8 GT/s supported\n");
		if(pcaps[11] & 0x10)
			g_log("16 GT/s supported\n");
	}
	*/
}
