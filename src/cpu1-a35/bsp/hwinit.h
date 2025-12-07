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

#ifndef hwinit_h
#define hwinit_h

#include <core/platform.h>

//#include <peripheral/ADC.h>
//#include <peripheral/EXTI.h>
#include <peripheral/GPIO.h>
//#include <peripheral/I2C.h>
//#include <peripheral/OctoSPI.h>
//#include <peripheral/Power.h>
#include <peripheral/PCIE.h>
//#include <peripheral/SPI.h>
//#include <peripheral/UART.h>

#include <multicore/IPCDescriptorTable.h>

//#include <embedded-utils/OctoSPI_SpiFlashInterface.h>

///@brief Initialize application-specific hardware stuff
extern void App_Init();
void InitPCIe();
/*
//Common ISRs used by application and bootloader
void USART6_Handler();

//Helper functions that need to move to boilerplate eventually
void BSP_InitPower();
void BSP_InitClocks();
void BSP_InitUART();
void BSP_InitLog();
void BSP_Init();
void BSP_InitDebug();

//GPIOs
void InitGPIOs();
extern GPIOPin g_blueLED;
extern GPIOPin g_redLED_n;
extern GPIOPin g_greenLED;
extern GPIOPin g_orangeLED;
extern GPIOPin g_user1Button;

void InitMemory();

//The UART
extern UART<16, 256> g_uart;

//QSPI flash interface
void InitQSPI();
extern OctoSPI_SpiFlashInterface* g_flash;
*/

#endif
