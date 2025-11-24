#include <stdint.h>

extern "C" void MainLoop_core0();
extern "C" void MainLoop_core1();
extern "C" void InitHook_core0();
extern "C" void InitHook_core1();

extern "C" void MainLoop_core0()
{
	//DEBUG: Turn on an LED (blue, PJ7) just so we can verify it works
	//Should already be configured by the M33 just need to turn it on
	volatile uint32_t* gpioj = reinterpret_cast<volatile uint32_t*>(0x442d0000);
	gpioj[6] = 0x80;

	while(1)
	{}
}

extern "C" void MainLoop_core1()
{
	//DEBUG: Turn on an LED (green, PD8) just so we can verify it works
	volatile uint32_t* gpiod = reinterpret_cast<volatile uint32_t*>(0x44270000);
	gpiod[6] = 0x100;

	while(1)
	{}
}

extern "C" void InitHook_core0()
{
}

extern "C" void InitHook_core1()
{
}
