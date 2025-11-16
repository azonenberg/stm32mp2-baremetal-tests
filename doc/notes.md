# Progress notes

## Minimum signals to bring up blue LED

* RCC_GPIOJCFGR / 0x5420_0550 = 0x6 (enable GPIOJ)
* GPIOJ_MODER / 0x542d_0000 = 0xffff7fff (set PJ7 as output)
* GPIOJ_ODR / 0x542d_0014  = 0x80 (turn it on)

## Bringing up CPU2 (Cortex-M33)

* RETRAM is 128 kB at 0a08_0000

* Writing RCC_C2RSTCSETR (0x5420_040c) to 1 should trigger a reset of CPU2.
* Boot config registers are documented under CA35SS SYSCFG
* Reset: secure VTOR is 0e08_0000, non-secure VTOR is 0a08_0000
* So we want to modify secure VTOR to the same so we don't care if we're in tz or not
    * CA35SS_SYSCFG_M33_INITSVTOR_CR (0x5880_20a4) = 0x0a08_0000
    * CA35SS_SYSCFG_M33_INITNSVTOR_CR (0x5880_20a8) = 0x0a08_0000 per docs
    * but we actually see 3008_0000 and 2008_0000! this is still the start of RETRAM but a different alias

## Bringing up CPU1 (Cortex-A35 x2 cluster)

* RCC_C1RSTCSETR (0x5420_0404) to 1 resets CPU1 **and the boot mode registers**
    * CA35SS_SYSCFG_AARCH_MODE_CR (0x5880_2080) to 1 makes CPU boot in aarch64 mode
    * CA35SS_SYSCFG_VBAR_CR (0x5880_2084) is aarch64-only reset vector (aarch32 reset vector hard wired to 0000_0000)
    * We **cannot** poke these from the M33 to boot into aarch64 mode right away!!!
    * So we need to load an aarch32 boot stub to do this then request a warm reset and WFI
* Initial aarch32 reset vector is boot ROM at 0000_0000 (visible only to CPU1)
* So how do we get the CPU1 running from the M33??
    * We can burn an image to flash (via the M33) and boot it?
    * Or be evil and warm reboot it via CoreSight?
    * Warm reset: RMR_EL3 = 3
* Ideally: we want image in SYSRAM (0x2000_0000)

## Debug stuff

* Port 3901 is A complex gdbserver
* Port 3902 is M33 gdb server
* Port 3903 is m0+ gdb server
