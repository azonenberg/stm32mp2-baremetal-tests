.section .vector
_vectors:
	b	_start	//reset vector

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables used by startup
.data
.align	8
__startupInitDone:
	.quad 0


.text

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset vector

.globl _start
_start:

	//Clear GPRs x0-x30
	mov		x0, 0
	mov		x1, 0
	mov		x2, 0
	mov		x3, 0
	mov		x4, 0
	mov		x5, 0
	mov		x6, 0
	mov		x7, 0
	mov		x8, 0
	mov		x9, 0
	mov		x10, 0
	mov		x11, 0
	mov		x12, 0
	mov		x13, 0
	mov		x14, 0
	mov		x15, 0
	mov		x16, 0
	mov		x17, 0
	mov		x18, 0
	mov		x19, 0
	mov		x20, 0
	mov		x21, 0
	mov		x22, 0
	mov		x23, 0
	mov		x24, 0
	mov		x25, 0
	mov		x26, 0
	mov		x27, 0
	mov		x28, 0
	mov		x29, 0
	mov		x30, 0

	//Seems like ROM code enables SMP already, so nothing needed for that

	//L1 is not active yet so we need to turn that on still
	//ROM code invalidates the L1 on reset by default, so we don't have to do that ourself, just enable it

	//TODO: initialize FPU and clear its registers
	//TODO: clear NEON/vector registers

	//TODO: bring up main CPU PLL vs running at whatever freq we do in startup code (this should probably be done in C++)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Figure out which core we're on

IdentifyCores:

	//Get the core number and go to core 1 path if we're on core 1
	mrs		x0, MPIDR_EL1
	and		x0, x0, 0xff
	cbnz	x0, InitializeCore1

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Core 0 initialization (fall through from identifyCores)

InitializeCore0:

	//Initialize stack pointer. apparently aarch64 doesn't let you ldr directly to sp?
	ldr		x0, =__core0stackstart
	mov		sp, x0

	//TODO: call global constructors

	//Tell core 1 that we're ready
	ldr		x0, =__startupInitDone
	mov		x1, #1
	str		x1, [x0]

	//Call application code
	bl		InitHook_core0
	bl		MainLoop_core0

	//If application main loop ever terminates, hang here
core0MainLoopTerminated:
	b		core0MainLoopTerminated

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Core 1 initialization

InitializeCore1:

	//Initialize stack pointer
	ldr		x0, =__core1stackstart
	mov		sp, x0

	//Spin until core 0 finishes initializing
	ldr		x0, =__startupInitDone
core1InitLoop:
	ldr		x1, [x0]
	cbz		x1, core1InitLoop

	//Call application code
	bl		InitHook_core1
	bl		MainLoop_core1

	//If application main loop ever terminates, hang here
core1MainLoopTerminated:
	b		core1MainLoopTerminated
