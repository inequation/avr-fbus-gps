.nolist
.include "tn2313def.inc"
.list

.cseg

.org	0
	rjmp	reset

.org	OVF1addr
	rjmp	timer_overflow

.org	URXCaddr
	rjmp	RX_complete

.org	UTXCaddr
	rjmp	TX_complete


reset:
	; set port B pins 2-7 as inputs, pins 0-1 as outputs
	ldi		r16, 0x03
	out		ddrb, r16
	; set PB0 and PB1 to high, i.e. do not enable any port
	ldi		r16, 0x03
halt:
	nop
	rjmp	halt

timer_overflow:

RX_complete:

TX_complete:
