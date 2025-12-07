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
#include "RISAFMonitorTask.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction / destruction

RISAFMonitorTask::RISAFMonitorTask()
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pop stuff

void RISAFMonitorTask::Iteration()
{
	Check(&RISAF1, 1);
	Check(&RISAF2, 2);
	//there is no RISAF3?
	Check(&RISAF4, 4);
	Check(&RISAF5, 5);
}

void RISAFMonitorTask::Check(volatile risaf_t* risaf, int i)
{
	if(risaf->IASR & RISAF_IASR_CAEF)
	{
		g_log(Logger::ERROR, "[RISAF%d]: Illegal access detected to configuration register\n", i);
		risaf->IACR = RISAF_IASR_CAEF;
	}

	if(risaf->IASR & RISAF_IASR_IAEF0)
	{
		g_log(Logger::ERROR, "[RISAF%d]: Illegal access detected by checker 0: %s %s %s by CID%d to 0x%08x\n",
			i,
			(risaf->IAESR0 & RISAF_IAESR_IASEC) ? "secure" : "nonsecure",
			(risaf->IAESR0 & RISAF_IAESR_IAPRIV) ? "privileged" : "unprivileged",
			(risaf->IAESR0 & RISAF_IAESR_IANRW) ? "write" : "read",
			risaf->IAESR0 & 7,
			risaf->IADDR0);
		risaf->IACR = RISAF_IASR_IAEF0;
	}

	if(risaf->IASR & RISAF_IASR_IAEF1)
	{
		g_log(Logger::ERROR, "[RISAF%d]: Illegal access detected by checker 1: %s %s %s by CID%d to 0x%08x\n",
			i,
			(risaf->IAESR1 & RISAF_IAESR_IASEC) ? "secure" : "nonsecure",
			(risaf->IAESR1 & RISAF_IAESR_IAPRIV) ? "privileged" : "unprivileged",
			(risaf->IAESR1 & RISAF_IAESR_IANRW) ? "write" : "read",
			risaf->IAESR1 & 7,
			risaf->IADDR1);

		risaf->IACR = RISAF_IASR_IAEF1;
	}
}
