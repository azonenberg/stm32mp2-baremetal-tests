.syntax unified
.thumb

//Vector table
.section .vector
.global _vectors
_vectors:
	.word 0x2001fffc	//initial stack pointer: last valid address in RETRAM
	.word _start		//reset vector
	.word 0x00000000	//NMI vector: not used
	.word 0x00000000	//hardfault vector: not used
	.word 0x00000000	//memmanage vector: not used
	.word 0x00000000	//busfault vector: not used
	.word 0x00000000	//usagefault vector: not used
	.word 0x00000000	//reserved vector 7
	.word 0x00000000	//reserved vector 8
	.word 0x00000000	//reserved vector 9
	.word 0x00000000	//reserved vector 10
	.word 0x00000000	//svcall: not used
	.word 0x00000000	//debugmon: not used
	.word 0x00000000	//reserved vector 13
	.word 0x00000000	//pendsv: not used
	.word 0x00000000	//systick: not used
	.word 0x00000000	//irq0: not used

//Entry point
.text
.globl _start
.thumb_func
_start:

	//Enable GPIOJ: RCC_GPIOJCFGR = 6, barrier to make sure this commits before we try to use GPIOJ
	ldr	r0, =0x54200550
	mov	r1, 6
	str	r1, [r0]
	dsb

	//Set PJ7 as output: GPIOJ_MODER = 0xffff7fff
	ldr r0, =0x542d0000
	ldr r1, =0xffff7fff
	str	r1, [r0]

	//Turn on the LED: GPIOJ_ODR = 0x80
	ldr	r0, =0x542d0014
	ldr r1, =0x80
	str r1, [r0]
	dsb

//Infinite loop
infloop:
	b	infloop
