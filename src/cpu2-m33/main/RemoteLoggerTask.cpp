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
#include "RemoteLoggerTask.h"

#define LOG_BUFFER_SIZE 256

//RX buffer for logger
char g_coreLogChannelName[NUM_SECONDARY_CORES][16] __attribute__((section(".ipcbuf")));
volatile uint8_t g_coreLogBuffer[NUM_SECONDARY_CORES][LOG_BUFFER_SIZE] __attribute__((section(".ipcbuf")));

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

RemoteLoggerTask::RemoteLoggerTask()
{
	g_log("Initializing remote logger task\n");
	LogIndenter li(g_log);

	//Allocate IPC channels
	for(uint32_t i=0; i<NUM_SECONDARY_CORES; i++)
	{
		char* namebuf = g_coreLogChannelName[i];
		strncpy(namebuf, "log.a35-0", sizeof(g_coreLogChannelName[i]));
		namebuf[8] = '0' + i;
		m_logChannels[i] = g_ipcDescriptorTable.AllocateChannel(
			namebuf,
			nullptr, 0,
			g_coreLogBuffer[i], sizeof(g_coreLogBuffer[i]));
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pop stuff

void RemoteLoggerTask::Iteration()
{
	//We can't bound how much data the remote side will give us, or how long the lines will be
	//so we need two full max-sized buffers on the stack (512 bytes)
	char tmp[LOG_BUFFER_SIZE + 1];
	char tmp2[LOG_BUFFER_SIZE + 1];

	for(uint32_t i=0; i<NUM_SECONDARY_CORES; i++)
	{
		//Pop the buffer
		uint32_t readSize = m_logChannels[i]->GetSecondaryFifo().Pop(reinterpret_cast<uint8_t*>(tmp));
		if(readSize == 0)
			continue;
		tmp[readSize] = '\0';

		//Read and process one line at a time
		const char* pline = tmp;
		const char* pend = tmp + LOG_BUFFER_SIZE;
		while( (pline < pend) && (pline[0] != '\0') )
		{
			//Look for newline
			auto pnl = strstr(pline, "\n");
			if(pnl)
			{
				//Extract and null terminate the string
				auto len = (pnl+1 - pline);
				memcpy(tmp2, pline, len);
				tmp2[len] = '\0';

				g_log.WriteRaw(tmp2);

				pline = pnl + 1;
			}

			else
			{
				//No newline found, log the message and add our own newline
				//(this should never happen and indicates a bug on the other side of the link)
				g_log.WriteRaw(pline);
				g_log.WriteRaw("\n");
				break;
			}
		}
	}
}
