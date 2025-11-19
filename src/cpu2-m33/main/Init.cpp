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

#include "m33test.h"
//#include "LEDTask.h"
#include <math.h>
//#include <peripheral/DWT.h>
//#include "ITMTask.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Task tables

etl::vector<Task*, MAX_TASKS>  g_tasks;
etl::vector<TimerTask*, MAX_TIMER_TASKS>  g_timerTasks;

//Firmware image we are going to flash
uint8_t g_firmwareImage[65536] __attribute__((aligned(16384))) __attribute__((section(".bssnoinit")));

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripheral initialization

void App_Init()
{
	g_log("Application initialization\n");
	LogIndenter li(g_log);

	//RCCHelper::Enable(&_RTC);

	//Format version string
	/*
	StringBuffer buf(g_version, sizeof(g_version));
	static const char* buildtime = __TIME__;
	buf.Printf("%s %c%c%c%c%c%c",
		__DATE__, buildtime[0], buildtime[1], buildtime[3], buildtime[4], buildtime[6], buildtime[7]);
	g_log("Firmware version %s\n", g_version);
	*/
	/*
	//Start tracing
	#ifdef _DEBUG
		ITM::Enable();
		DWT::EnablePCSampling(DWT::PC_SAMPLE_SLOW);
		ITM::EnableDwtForwarding();
	#endif

	static LEDTask ledTask;
	static ButtonTask buttonTask;
	static DumptruckSuperSPIServer spiserver(g_spi);
	static SensorTask sensorTask;
	#ifdef _DEBUG
		static ITMTask itmTask;
	#endif

	g_tasks.push_back(&ledTask);
	g_tasks.push_back(&buttonTask);
	g_tasks.push_back(&g_super);
	g_tasks.push_back(&spiserver);
	g_tasks.push_back(&sensorTask);
	#ifdef _DEBUG
		g_tasks.push_back(&itmTask);
	#endif

	g_timerTasks.push_back(&ledTask);
	#ifdef _DEBUG
		g_timerTasks.push_back(&itmTask);
	#endif
	*/

	//Read the buttons and check if we should flash
	if(g_user1Button)
	{
		g_log("USER1 button pressed, writing %d bytes from address %08x to flash\n",
			sizeof(g_firmwareImage), g_firmwareImage);
		LogIndenter li2(g_log);

		g_log("Waiting...\n");
		g_logTimer.Sleep(30000);

		//Validate the image starts with what we expect
		const uint8_t magic[] = { 'S', 'T', 'M', 0x32};
		if(0 != memcmp(g_firmwareImage, magic, sizeof(magic)))
		{
			g_log(Logger::ERROR, "Bad magic number on image (expected %02x %02x %02x %02x, got %02x %02x %02x %02x\n",
				magic[0], magic[1], magic[2], magic[3],
				g_firmwareImage[0], g_firmwareImage[1], g_firmwareImage[2], g_firmwareImage[3]);

			while(1)
			{}
		}

		{
			g_log("Erasing...\n");
			LogIndenter li3(g_log);
			for(uintptr_t addr=0; addr<sizeof(g_firmwareImage); addr += g_flash->GetSectorSize())
				g_flash->EraseSector(reinterpret_cast<uint8_t*>(addr));
		}

		g_log("Waiting...\n");
		g_logTimer.Sleep(30000);

		{
			g_log("Blank checking...\n");
			LogIndenter li3(g_log);

			uint8_t tmp[128] = {0};
			for(uintptr_t addr=0; addr<sizeof(g_firmwareImage); addr += sizeof(tmp))
			{
				//low level manual for now since we dont have memory map set up or want to deal with caches
				g_flash->SetAddressMode(OctoSPI::MODE_SINGLE, 3);
				g_flash->SetDataMode(OctoSPI::MODE_SINGLE);
				g_flash->BlockingRead(0x03, addr, tmp, sizeof(tmp));

				bool fail = false;
				for(size_t i=0; i<sizeof(tmp); i++)
				{
					if(tmp[i] != 0xff)
					{
						g_log(Logger::ERROR, "Blank check FAILED at address %08x, read %02x expected 0xff\n",
							addr + i, tmp[i]);
						fail = true;
						break;
					}
				}
				if(fail)
					break;
			}
		}

		g_log("Waiting...\n");
		g_logTimer.Sleep(30000);

		//TODO: program
		{
			g_log("Programming...\n");
			LogIndenter li3(g_log);
			g_flash->Write(reinterpret_cast<uint8_t*>(0), g_firmwareImage, sizeof(g_firmwareImage));
		}

		g_log("Waiting...\n");
		g_logTimer.Sleep(30000);

		{
			g_log("Verifying...\n");
			LogIndenter li3(g_log);

			uint8_t tmp[128] = {0};
			for(uintptr_t addr=0; addr<sizeof(g_firmwareImage); addr += sizeof(tmp))
			{
				//low level manual for now since we dont have memory map set up or want to deal with caches
				g_flash->SetAddressMode(OctoSPI::MODE_SINGLE, 3);
				g_flash->SetDataMode(OctoSPI::MODE_SINGLE);
				g_flash->BlockingRead(0x03, addr, tmp, sizeof(tmp));

				bool fail = false;
				for(size_t i=0; i<sizeof(tmp); i++)
				{
					uint8_t expected = g_firmwareImage[addr+i];
					if(tmp[i] != expected)
					{
						g_log(Logger::ERROR, "Verification FAILED at address %08x, read %02x expected %02x\n",
							addr + i, tmp[i], expected);
						fail = true;
						break;
					}
				}
				if(fail)
					break;
			}
		}
	}
	else
		g_log("USER1 button not pressed, not flashing\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
