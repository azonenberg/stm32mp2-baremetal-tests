.section .vector
_vectors:
	b	_start	//reset vector

	//padding at start of text segment just so we can tell where dtors end more easily
.text
	nop
	nop
	nop
	nop

//Reset vector
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

	//Zero stack pointer for now. We'll initialize it properly later, once we have figured out which core we're on
	mov		sp, x0

	//Seems like ROM code enables SMP already, so nothing needed for that

	//L1 is not active yet so we need to turn that on
	//ROM code invalidates the L1 on reset by default, so we don't have to do that ourself

	//TODO: initialize FPU and clear its registers
	//TODO: clear NEON/vector registers

	//TODO: bring up main CPU PLL vs running at whatever freq we do in startup code (this should probably be done in C++)

identifyCores:

	//Get the core number and go to core 1 path if we're on core 1
	mrs		x0, MPIDR_EL1
	and		x0, x0, 0xff
	cbnz	x0, core1init

	//fall through to core 0 init path if not
core0init:

	//Initialize stack pointer. apparently aarch64 doesn't let you ldr directly to sp?
	ldr		x0, pstack0
	mov		sp, x0

	//Infinite loop
core0loop:
	b		core0loop

//if we
core1init:
	//Initialize stack pointer
	ldr		x0, pstack1
	mov		sp, x0

	//Infinite loop
core1loop:
	b	core1loop

//Data pointers
	.align 8
pstack0:
	.quad	__core0stackstart
pstack1:
	.quad	__core1stackstart
