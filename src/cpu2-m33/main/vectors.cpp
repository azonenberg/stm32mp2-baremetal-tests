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

typedef void(*fnptr)();

extern uint32_t __stack;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

//prototypes
extern "C" void _start();
void MemManage_Handler();
void UsageFault_Handler();
void BusFault_Handler();
void HardFault_Handler();
void NMI_Handler();

void defaultISR();
void USART1_Handler();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt vector table

fnptr __attribute__((section(".vector"))) vectorTable[] =
{
	(fnptr)&__stack,		//stack
	_start,					//reset
	NMI_Handler,			//NMI
	HardFault_Handler,		//hardfault
	MemManage_Handler,		//mem management fault
	BusFault_Handler,		//busfault
	UsageFault_Handler,		//usagefault
	defaultISR,				//reserved_7
	defaultISR,				//reserved_8
	defaultISR,				//reserved_9
	defaultISR,				//reserved_10
	defaultISR,				//svcall
	defaultISR,				//debug
	defaultISR,				//reserved_13
	defaultISR,				//pend_sv
	defaultISR,				//systick
	defaultISR,				//irq0 PVD
	defaultISR,				//irq1 PVM
	defaultISR,				//irq2 IWDG3
	defaultISR,				//irq3 IWDG4
	defaultISR,				//irq4 IWDG1_RST
	defaultISR,				//irq5 IWDG2_RST
	defaultISR,				//irq6 IWDG4_RST
	defaultISR,				//irq7 IWDG5_RST
	defaultISR,				//irq8 WWDG1
	defaultISR,				//irq9 reserved
	defaultISR,				//irq10 reserved
	defaultISR,				//irq11 WWDG2_RST
	defaultISR,				//irq12 TAMP
	defaultISR,				//irq13 RTC
	defaultISR,				//irq14 TAMP_S
	defaultISR,				//irq15 RTC_S
	defaultISR,				//irq16 RCC
	defaultISR,				//irq17 EXTI2_0
	defaultISR,				//irq18 EXTI2_1
	defaultISR,				//irq19 EXTI2_2
	defaultISR,				//irq20 EXTI2_3
	defaultISR,				//irq21 EXTI2_4
	defaultISR,				//irq22 EXTI2_5
	defaultISR,				//irq23 EXTI2_6
	defaultISR,				//irq24 EXTI2_7
	defaultISR,				//irq25 EXTI2_8
	defaultISR,				//irq26 EXTI2_9
	defaultISR,				//irq27 EXTI2_10
	defaultISR,				//irq28 EXTI2_11
	defaultISR,				//irq29 EXTI2_12
	defaultISR,				//irq30 EXTI2_13
	defaultISR,				//irq31 EXTI2_14
	defaultISR,				//irq32 EXTI2_15
	defaultISR,				//irq33 HPDMA1_Channel0
	defaultISR,				//irq34 HPDMA1_Channel1
	defaultISR,				//irq35 HPDMA1_Channel2
	defaultISR,				//irq36 HPDMA1_Channel3
	defaultISR,				//irq37 HPDMA1_Channel4
	defaultISR,				//irq38 HPDMA1_Channel5
	defaultISR,				//irq39 HPDMA1_Channel6
	defaultISR,				//irq40 HPDMA1_Channel7
	defaultISR,				//irq41 HPDMA1_Channel8
	defaultISR,				//irq42 HPDMA1_Channel9
	defaultISR,				//irq43 HPDMA1_Channel10
	defaultISR,				//irq44 HPDMA1_Channel11
	defaultISR,				//irq45 HPDMA1_Channel12
	defaultISR,				//irq46 HPDMA1_Channel13
	defaultISR,				//irq47 HPDMA1_Channel14
	defaultISR,				//irq48 HPDMA1_Channel15
	defaultISR,				//irq49 HPDMA2_Channel0
	defaultISR,				//irq50 HPDMA2_Channel1
	defaultISR,				//irq51 HPDMA2_Channel2
	defaultISR,				//irq52 HPDMA2_Channel3
	defaultISR,				//irq53 HPDMA2_Channel4
	defaultISR,				//irq54 HPDMA2_Channel5
	defaultISR,				//irq55 HPDMA2_Channel6
	defaultISR,				//irq56 HPDMA2_Channel7
	defaultISR,				//irq57 HPDMA2_Channel8
	defaultISR,				//irq58 HPDMA2_Channel9
	defaultISR,				//irq59 HPDMA2_Channel10
	defaultISR,				//irq60 HPDMA2_Channel11
	defaultISR,				//irq61 HPDMA2_Channel12
	defaultISR,				//irq62 HPDMA2_Channel13
	defaultISR,				//irq63 HPDMA2_Channel14
	defaultISR,				//irq64 HPDMA2_Channel15
	defaultISR,				//irq65 HPDMA3_Channel0
	defaultISR,				//irq66 HPDMA3_Channel1
	defaultISR,				//irq67 HPDMA3_Channel2
	defaultISR,				//irq68 HPDMA3_Channel3
	defaultISR,				//irq69 HPDMA3_Channel4
	defaultISR,				//irq70 HPDMA3_Channel5
	defaultISR,				//irq71 HPDMA3_Channel6
	defaultISR,				//irq72 HPDMA3_Channel7
	defaultISR,				//irq73 HPDMA3_Channel8
	defaultISR,				//irq74 HPDMA3_Channel9
	defaultISR,				//irq75 HPDMA3_Channel10
	defaultISR,				//irq76 HPDMA3_Channel11
	defaultISR,				//irq77 HPDMA3_Channel12
	defaultISR,				//irq78 HPDMA3_Channel13
	defaultISR,				//irq79 HPDMA3_Channel14
	defaultISR,				//irq80 HPDMA3_Channel15
	defaultISR,				//irq81 LPDMA1_Channel0
	defaultISR,				//irq82 LPDMA1_Channel1
	defaultISR,				//irq83 LPDMA1_Channel2
	defaultISR,				//irq84 LPDMA1_Channel3
	defaultISR,				//irq85 ICACHE
	defaultISR,				//irq86 DCACHE
	defaultISR,				//irq87 ADC1
	defaultISR,				//irq88 ADC2
	defaultISR,				//irq89 ADC3
	defaultISR,				//irq90 FDCAN_CAL
	defaultISR,				//irq91 FDCAN1_IT0
	defaultISR,				//irq92 FDCAN2_IT0
	defaultISR,				//irq93 FDCAN3_IT0
	defaultISR,				//irq94 FDCAN1_IT1
	defaultISR,				//irq95 FDCAN2_IT1
	defaultISR,				//irq96 FDCAN3_IT1
	defaultISR,				//irq97 TIM1_BRK
	defaultISR,				//irq98 TIM1_UP
	defaultISR,				//irq99 TIM1_TRG_COM
	defaultISR,				//irq100 TIM1_CC
	defaultISR,				//irq101 TIM20_BRK
	defaultISR,				//irq102 TIM20_UP
	defaultISR,				//irq103 TIM20_TRG_COM
	defaultISR,				//irq104 TIM20_CC
	defaultISR,				//irq105 TIM2
	defaultISR,				//irq106 TIM3
	defaultISR,				//irq107 TIM4
	defaultISR,				//irq108 I2C1
	defaultISR,				//irq109 I3C1
	defaultISR,				//irq110 I2C2
	defaultISR,				//irq111 I3C2
	defaultISR,				//irq112 SPI1
	defaultISR,				//irq113 SPI2
	defaultISR,				//irq114 USART1
	defaultISR,				//irq115 USART2
	defaultISR,				//irq116 USART3
	defaultISR,				//irq117 VDEC
	defaultISR,				//irq118 TIM8_BRK
	defaultISR,				//irq119 TIM8_UP
	defaultISR,				//irq120 TIM8_TRG_COM
	defaultISR,				//irq121 TIM8_CC
	defaultISR,				//irq122 FMC
	defaultISR,				//irq123 SDMMC1
	defaultISR,				//irq124 TIM5
	defaultISR,				//irq125 SPI3
	defaultISR,				//irq126 UART4
	defaultISR,				//irq127 UART5
	defaultISR,				//irq128 TIM6
	defaultISR,				//irq129 TIM7
	defaultISR,				//irq130 ETH1_SBD
	defaultISR,				//irq131 ETH1_PMT
	defaultISR,				//irq132 ETH1_LPI
	defaultISR,				//irq133 ETH2_SBD
	defaultISR,				//irq134 ETH2_PMT
	defaultISR,				//irq135 ETH2_LPI
	defaultISR,				//irq136 USART6
	defaultISR,				//irq137 I2C3
	defaultISR,				//irq138 I3C3
	defaultISR,				//irq139 USBH_EHCI
	defaultISR,				//irq140 USBH_OHCI
	defaultISR,				//irq141 DCMI_PSSI
	defaultISR,				//irq142 CSI
	defaultISR,				//irq143 DSI
	defaultISR,				//irq144 CRYP1
	defaultISR,				//irq145 HASH
	defaultISR,				//irq146 PKA
	defaultISR,				//irq147 FPU
	defaultISR,				//irq148 UART7
	defaultISR,				//irq149 UART8
	defaultISR,				//irq150 UART9
	defaultISR,				//irq151 LPUART1
	defaultISR,				//irq152 SPI4
	defaultISR,				//irq153 SPI5
	defaultISR,				//irq154 SPI6
	defaultISR,				//irq155 SPI7
	defaultISR,				//irq156 SPI8
	defaultISR,				//irq157 SAI1
	defaultISR,				//irq158 LTDC
	defaultISR,				//irq159 LTDC_ER
	defaultISR,				//irq160 LTDC_SEC
	defaultISR,				//irq161 LTDC_SEC_ER
	defaultISR,				//irq162 SAI2
	defaultISR,				//irq163 OCTOSPI1
	defaultISR,				//irq164 OCTOSPI2
	defaultISR,				//irq165 OTFDEC1
	defaultISR,				//irq166 LPTIM1
	defaultISR,				//irq167 VENC
	defaultISR,				//irq168 I2C4
	defaultISR,				//irq169 USBH_WAKEUP
	defaultISR,				//irq170 SPDIFRX
	defaultISR,				//irq171 IPCC1_RX
	defaultISR,				//irq172 IPCC1_TX
	defaultISR,				//irq173 IPCC1_RX_S
	defaultISR,				//irq174 IPCC1_TX_S
	defaultISR,				//irq175 IPCC2_RX
	defaultISR,				//irq176 IPCC2_TX
	defaultISR,				//irq177 IPCC2_RX_S
	defaultISR,				//irq178 IPCC2_TX_S
	defaultISR,				//irq179 SAES
	defaultISR,				//irq180 CRYP2
	defaultISR,				//irq181 I2C5
	defaultISR,				//irq182 USB3DR_WAKEUP
	defaultISR,				//irq183 GPU
	defaultISR,				//irq184 MDF1_FLT0
	defaultISR,				//irq185 MDF1_FLT1
	defaultISR,				//irq186 MDF1_FLT2
	defaultISR,				//irq187 MDF1_FLT3
	defaultISR,				//irq188 MDF1_FLT4
	defaultISR,				//irq189 MDF1_FLT5
	defaultISR,				//irq190 MDF1_FLT6
	defaultISR,				//irq191 MDF1_FLT7
	defaultISR,				//irq192 SAI3
	defaultISR,				//irq193 TIM15
	defaultISR,				//irq194 TIM16
	defaultISR,				//irq195 TIM17
	defaultISR,				//irq196 TIM12
	defaultISR,				//irq197 SDMMC2
	defaultISR,				//irq198 DCMIPP
	defaultISR,				//irq199 HSEM_IT1
	defaultISR,				//irq200 HSEM_S_IT1
	defaultISR,				//irq201 nCTIIRQ1
	defaultISR,				//irq202 nCTIIRQ2
	defaultISR,				//irq203 TIM13
	defaultISR,				//irq204 TIM14
	defaultISR,				//irq205 TIM10
	defaultISR,				//irq206 RNG
	defaultISR,				//irq207 ADF1_FLT
	defaultISR,				//irq208 I2C6
	defaultISR,				//irq209 COMBOPHY_WAKEUP
	defaultISR,				//irq210 I2C7
	defaultISR,				//irq211 reserved
	defaultISR,				//irq212 I2C8
	defaultISR,				//irq213 I3C4
	defaultISR,				//irq214 SDMMC3
	defaultISR,				//irq215 LPTIM2
	defaultISR,				//irq216 LPTIM3
	defaultISR,				//irq217 LPTIM4
	defaultISR,				//irq218 LPTIM5
	defaultISR,				//irq219 OTFDEC2
	defaultISR,				//irq220 CPU1_SEV
	defaultISR,				//irq221 CPU3_SEV
	defaultISR,				//irq222 RCC_WAKEUP
	defaultISR,				//irq223 SAI4
	defaultISR,				//irq224 DTS
	defaultISR,				//irq225 TIM11
	defaultISR,				//irq226 CPU2_WAKEUP_PIN
	defaultISR,				//irq227 USB3DR_BC
	defaultISR,				//irq228 USB3DR
	defaultISR,				//irq229 UCPD
	defaultISR,				//irq230 reserved
	defaultISR,				//irq231 reserved
	defaultISR,				//irq232 reserved
	defaultISR,				//irq233 reserved
	defaultISR,				//irq234 reserved
	defaultISR,				//irq235 reserved
	defaultISR,				//irq236 reserved
	defaultISR,				//irq237 reserved
	defaultISR,				//irq238 reserved
	defaultISR,				//irq239 reserved
	defaultISR,				//irq240 SERC
	defaultISR,				//irq241 reserved
	defaultISR,				//irq242 RAMCFG
	defaultISR,				//irq243 DDRCTRL
	defaultISR,				//irq244 DDRPHYC
	defaultISR,				//irq245 DDRPHYC_DFI
	defaultISR,				//irq246 IAC
	defaultISR,				//irq247 VDDCPU_VD
	defaultISR,				//irq248 VDDCORE_VD
	defaultISR,				//irq249 reserved
	defaultISR,				//irq250 ETHSW
	defaultISR,				//irq251 ETHSW_MSGBUF
	defaultISR,				//irq252 ETHSW_FSC
	defaultISR,				//irq253 HPDMA1_WKUP
	defaultISR,				//irq254 HPDMA2_WKUP
	defaultISR,				//irq255 HPDMA3_WKUP
	defaultISR,				//irq256 LPDMA1_WKUP
	defaultISR,				//irq257 UCPD_VBUS_VALID
	defaultISR,				//irq258 UCPD_VBUS_VSAFE5V
	defaultISR,				//irq259 RCC_HS_FMON
	defaultISR,				//irq260 VDDGPU_VD
	defaultISR,				//irq261 reserved
	defaultISR,				//irq262 reserved
	defaultISR,				//irq263 reserved
	defaultISR,				//irq264 reserved
	defaultISR,				//irq265 reserved
	defaultISR,				//irq266 reserved
	defaultISR,				//irq267 reserved
	defaultISR,				//irq268 EXTI1_0
	defaultISR,				//irq269 EXTI1_1
	defaultISR,				//irq270 EXTI1_2
	defaultISR,				//irq271 EXTI1_3
	defaultISR,				//irq272 EXTI1_4
	defaultISR,				//irq273 EXTI1_5
	defaultISR,				//irq274 EXTI1_6
	defaultISR,				//irq275 EXTI1_7
	defaultISR,				//irq276 EXTI1_8
	defaultISR,				//irq277 EXTI1_9
	defaultISR,				//irq278 EXTI1_10
	defaultISR,				//irq279 EXTI1_11
	defaultISR,				//irq280 EXTI1_12
	defaultISR,				//irq281 EXTI1_13
	defaultISR,				//irq282 EXTI1_14
	defaultISR,				//irq283 EXTI1_15
	defaultISR,				//irq284 reserved
	defaultISR,				//irq285 reserved
	defaultISR,				//irq286 reserved
	defaultISR,				//irq287 reserved
	defaultISR,				//irq288 reserved
	defaultISR,				//irq289 reserved
	defaultISR,				//irq290 reserved
	defaultISR,				//irq291 reserved
	defaultISR,				//irq292 reserved
	defaultISR,				//irq293 reserved
	defaultISR,				//irq294 reserved
	defaultISR,				//irq295 reserved
	defaultISR,				//irq296 reserved
	defaultISR,				//irq297 reserved
	defaultISR,				//irq298 reserved
	defaultISR,				//irq299 reserved
	defaultISR,				//irq300 reserved
	defaultISR,				//irq301 reserved
	defaultISR,				//irq302 reserved
	defaultISR,				//irq303 reserved
	defaultISR,				//irq304 reserved
	defaultISR,				//irq305 reserved
	defaultISR,				//irq306 reserved
	defaultISR,				//irq307 reserved
	defaultISR,				//irq308 reserved
	defaultISR,				//irq309 reserved
	defaultISR,				//irq310 DDRPERFM
	defaultISR,				//irq311 reserved
	defaultISR,				//irq312 reserved
	defaultISR,				//irq313 reserved
	defaultISR,				//irq314 reserved
	defaultISR,				//irq315 reserved
	defaultISR,				//irq316 reserved
	defaultISR,				//irq317 reserved
	defaultISR,				//irq318 reserved
	defaultISR				//irq319 reserved
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Firmware version string used for the bootloader

extern "C" const char
	__attribute__((section(".fwid")))
	__attribute__((used))
	g_firmwareID[] = "stm32mp2-bmtest-m33";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Stub for unused interrupts

void defaultISR()
{
	//g_bbram->m_state = STATE_CRASH;
	//g_bbram->m_crashReason = CRASH_UNUSED_ISR;
	Reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Exception vectors

void NMI_Handler()
{
	//g_bbram->m_state = STATE_CRASH;
	//g_bbram->m_crashReason = CRASH_NMI;
	Reset();
}

void HardFault_Handler()
{
	/*
	//Save registers
	uint32_t* msp;
	asm volatile("mrs %[result], MSP" : [result]"=r"(msp));
	msp += 12;	//locals/alignment
	uint32_t r0 = msp[0];
	uint32_t r1 = msp[1];
	uint32_t r2 = msp[2];
	uint32_t r3 = msp[3];
	uint32_t r12 = msp[4];
	uint32_t lr = msp[5];
	uint32_t pc = msp[6];
	uint32_t xpsr = msp[7];

	g_uart.Printf("Hard fault\n");
	g_uart.Printf("    HFSR  = %08x\n", *(volatile uint32_t*)(0xe000ed2C));
	g_uart.Printf("    MMFAR = %08x\n", *(volatile uint32_t*)(0xe000ed34));
	g_uart.Printf("    BFAR  = %08x\n", *(volatile uint32_t*)(0xe000ed38));
	g_uart.Printf("    CFSR  = %08x\n", *(volatile uint32_t*)(0xe000ed28));
	g_uart.Printf("    UFSR  = %08x\n", *(volatile uint16_t*)(0xe000ed2a));
	g_uart.Printf("    DFSR  = %08x\n", *(volatile uint32_t*)(0xe000ed30));
	g_uart.Printf("    MSP   = %08x\n", msp);
	g_uart.BlockingFlush();
	g_uart.Printf("    r0    = %08x\n", r0);
	g_uart.Printf("    r1    = %08x\n", r1);
	g_uart.Printf("    r2    = %08x\n", r2);
	g_uart.Printf("    r3    = %08x\n", r3);
	g_uart.Printf("    r12   = %08x\n", r12);
	g_uart.Printf("    lr    = %08x\n", lr);
	g_uart.Printf("    pc    = %08x\n", pc);
	g_uart.Printf("    xpsr  = %08x\n", xpsr);

	g_uart.Printf("    Stack:\n");
	g_uart.BlockingFlush();
	for(int i=0; i<16; i++)
	{
		g_uart.Printf("        %08x\n", msp[i]);
		g_uart.BlockingFlush();
	}

	//Shut down as we can no longer properly supervise the rails
	g_super.PanicShutdown();

	while(1)
	{}

	//g_bbram->m_state = STATE_CRASH;
	//g_bbram->m_crashReason = CRASH_HARD_FAULT;
	//Reset();
	*/
}

void BusFault_Handler()
{
	//g_bbram->m_state = STATE_CRASH;
	//g_bbram->m_crashReason = CRASH_BUS_FAULT;
	Reset();
}

void UsageFault_Handler()
{
	//g_bbram->m_state = STATE_CRASH;
	//g_bbram->m_crashReason = CRASH_USAGE_FAULT;
	Reset();
}

void MemManage_Handler()
{
	//g_bbram->m_state = STATE_CRASH;
	//g_bbram->m_crashReason = CRASH_MMU_FAULT;
	Reset();
}
