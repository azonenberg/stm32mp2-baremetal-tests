.section .vector
_vectors:
	b	_start	//reset vector

	//Placeholders
	nop
	nop
	nop
	nop

	//text segment
.text
	nop

//Entry point: infinite loop
.globl _start
_start:
	b	_start
