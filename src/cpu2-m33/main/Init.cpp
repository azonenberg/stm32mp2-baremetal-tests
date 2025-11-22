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
#include "LocalConsoleTask.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Task tables

etl::vector<Task*, MAX_TASKS>  g_tasks;
etl::vector<TimerTask*, MAX_TIMER_TASKS>  g_timerTasks;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Peripheral initialization

void App_Init()
{
	g_log("Application initialization\n");
	LogIndenter li(g_log);

	g_log("Firmware image partition in RAM is at %08x\n", g_firmwareImage);

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
	*/
	static LocalConsoleTask localConsoleTask;

	g_tasks.push_back(&localConsoleTask);
	/*
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
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
