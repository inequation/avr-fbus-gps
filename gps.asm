; AVR GSM-based (Nokia FBus) GPS locator
; Written by Leszek Godlewski <lg@inequation.org>

.nolist
.include "tn2313def.inc"
.list

; assume 3.6864MHz quartz

; =============================================================================
; Constants
; -----------------------------------------------------------------------------

; This constant sets the timer0 prescaler. Refer to the TCCR0B register
; documentation for your MCU.
;.equ	PRESCALER0	=	(1 << CS02 | 0 << CS01 | 1 << CS00)	; 1024 divisor
.equ	PRESCALER0	=	(0 << CS02 | 0 << CS01 | 1 << CS00)	; no prescaling, for debugging purposes

; This constant sets the timer1 prescaler. Refer to the TCCR1B register
; documentation for your MCU.
;.equ	PRESCALER1	=	(1 << CS12 | 0 << CS11 | 1 << CS10)	; 1024 divisor
.equ	PRESCALER1	=	(0 << CS12 | 0 << CS11 | 1 << CS10)	; no prescaling, for debugging purposes

; This constant sets how many overflows the MCU will wait before polling the
; GPS for the first time; a value of 3 makes it wait 54,61s (a GPS cold start
; usually takes no less than 40 seconds).
; One timer1 overflow takes 18.204s (3.6864MHz / 1024 = 3600Hz -> timer is
; ticked this many times per second; it takes 65536 / 3600Hz = 18.204s to
; overflow the timer).
.equ	WAIT_PERIOD	=	3

; UBRR setting for 9600 baud (assuming the 3.6864 quartz!)..
.equ	UBRR_9600	=	23
; UBRR setting for 115200 baud.
.equ	UBRR_115k2	=	1

; Port B value for enabling FBus comms - low on PB.0, high on PB.1.
.equ	ENABLE_FBUS	=	0x02
; Port B value for enabling GPS comms - high on PB.0, low on PB.1.
.equ	ENABLE_GPS	=	0x01

; =============================================================================
; Macros
; -----------------------------------------------------------------------------

; A macro to reset the Sleep Enable bit.
.macro		sleep_disable
	in		r16, MCUCR
	andi	r16, ~(1 << SE)
	out		MCUCR, r16
.endmacro

; A macro to copy r17 = @1 bytes from program memory address Z = @0 to X.
.macro		copy_PM
	ldi		ZH, high(@0 << 1)
	ldi		ZL, low(@0 << 1)
	ldi		r17, @1
	rcall	PM_read
.endmacro

; A macro to copy r17 = @1 bytes from EEPROM address r18 = @0 to X.
.macro		copy_EEPROM
	ldi		r18, @0
	ldi		r17, @1
	rcall	EEPROM_read
.endmacro

; =============================================================================
; Code
; -----------------------------------------------------------------------------

.cseg

; =============================================================================
; Interrupt handlers
; -----------------------------------------------------------------------------

.org	0
	rjmp	reset

;.org	OVF1addr
;	rjmp	timer_overflow

.org	URXCaddr
	rjmp	RX_complete

; =============================================================================
; Program space constants
; -----------------------------------------------------------------------------

; FBus SMS frame start. The 0xFF byte needs to be filled with valid frame
; length after assembling the entire frame.
tpl_SMS_frame_start:
	.db 0x1E, 0x00, 0x0C, 0x02, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x01, 0x02, 0x00

; Transport Protocol Data Unit descriptor. Byte 3 is set to indicate 8-bit data
; as opposed to the default 7-bit GSM alphabet to save us the hassle of
; encoding (see GSM 03.40).
; Byte 4 is to be filled with user data (actual message) length.
tpl_SMS_TPDU:
	.db	0x15, 0x00, 0x00, 0x04, 0xFF

; Validity Period descriptor.
tpl_SMS_VP:
	.db 0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

tpl_SMS_report_online:
	.db "avr-fbus-gps online"

; =============================================================================
; Reset handler
; -----------------------------------------------------------------------------

reset:
	ldi		r16, 1
	ldi		r17, 2
	cp		r16, r17
	; set port B pins 2-7 as inputs, pins 0-1 as outputs
	ldi		r16, 0x03
	out		DDRB, r16
	; set PB0 and PB1 to high, i.e. don't enable any of the UART devices
	ldi		r16, 0x03
	out		PORTB, r16

	; initialize the stack pointer - set it to the a part of the register file
	; to save on SRAM, we're not getting the stack any deeper than 3 levels
	; anyway
	ldi		r16, 0x08
	out		SPL, r16

	; wait a single timer0 cycle for the GPS to start up, too
	ldi		r16, PRESCALER0
	out		TCCR0B, r16
	; wait four cycles so that we don't read a zero
	nop
	nop
	nop
	nop
wait:
	in		r16, TCNT0
	cpi		r16, 0
	brne	wait
	; switch timer0 off
	ldi		r16, 0
	out		TCCR0B, r16

	; set the baud rate to 9600
	ldi		r16, UBRR_9600
	out		UBRRL, r16
	ldi		r16, 0
	out		UBRRH, r16

	; enable receiver and transmitter
	ldi		r16, (1 << RXEN) | (1 << TXEN)
	out		UCSRB, r16
	; default UART settings are exactly what we need (8 data bits, no parity, 1
	; stop bit), so no additional initialization is needed

	; switch to FBus (low on PB.2)
	ldi		r16, ENABLE_FBUS
	out		PORTB, r16

	; initialize FBus packet sequence number
	ldi		r25, 0

	; report in to the "nexus" - send an SMS to the registered number
	; assemble the appropriate FBus frame
	rcall	FBus_start
	; write the report message into the frame
	copy_PM	tpl_SMS_report_online, 19	
	; finish off the frame and fill in all length bytes and calculate checksums
	rcall	FBus_finish

	; synchronize the phone's UART
	rcall	FBus_sync

	; start the overflow counter
	ldi		r17, WAIT_PERIOD

	; enable timer interrupts, and stuff
;	ldi		r16, PRESCALER1
;	out		TCCR1B, r16
;	ldi		r16, (1 << TOIE1)
;	out		TIMSK, r16

	; global interrupt enable
;	sei

; =============================================================================
; Idle loop (go back to sleep after waking to handle an interrupt)
; -----------------------------------------------------------------------------

	; we've configured everything, now go to idle state and save power
idle:
	in		r16, MCUCR
	ori		r16, (1 << SE)	; set Sleep Enable
	out		MCUCR, r16
	sleep
	rjmp	idle

; =============================================================================
; UART receive complete interrupt handler
; -----------------------------------------------------------------------------

RX_complete:
	sleep_disable
	reti

; =============================================================================
; Utility procedures
; -----------------------------------------------------------------------------

; Reads the byte at r18 from EEPROM into r16.
EEPROM_read:
	; wait for completion of previous write
	sbic	EECR, EEPE
	rjmp 	EEPROM_read
	; set up address (r18) in address register
	out 	EEAR, r18
	; start eeprom read by writing EERE
	sbi 	EECR, EERE
	; read data from data register
	in		r16, EEDR
	st		X+, r16
	inc		r18
	dec		r17
	brne	EEPROM_read
ret

; Reads r17 bytes from Z into X.
PM_read:
	lpm		r16, Z+
	st		X+, r16
	dec		r17
	brne	PM_read
ret

; Transmits a byte from r16 via UART.
UART_Tx:
	; wait for empty buffer
	sbis	UCSRA, UDRE
	rjmp	UART_Tx
	out		UDR, r16
ret

; =============================================================================
; FBus utility procedures
; -----------------------------------------------------------------------------

; FBus synchronization procedure - sends 128 0x55 characters over UART.
FBus_sync:
	ldi		r16, 0x55
	ldi		r17, 128
loop_sync:
	rcall	UART_Tx
	dec		r17
	brne	loop_sync
ret

; Starts the FBus SMS frame.
FBus_start:
	ldi		XL,	low(SMS_frame_start)
	ldi		XH, high(SMS_frame_start)
	; start with the frame start
	copy_PM	tpl_SMS_frame_start, 12
	; read the SMS message centre from EEPROM into the frame
	copy_EEPROM	0x00, 12
	; write the Transport Protocol Data Unit descriptor into the frame
	copy_PM	tpl_SMS_TPDU, 5
	; read the destination number from EEPROM into the frame
	copy_EEPROM	0x0C, 12
	; write the Validity Period descriptor into the frame
	copy_PM	tpl_SMS_VP, 7
ret

; Processes the entire frame, calculating frame and message lengths, checksums
; and adjusting padding bytes.
FBus_finish:
	; write in the number of packets left to transmit (always 0)
	ldi		r16, 0
	st		X+, r16
	; write in the sequence number
	st		X+, r25
	inc		r25
	andi	r25, 0x07

	; get the length of the entire frame; no need to work on the high bytes,
	; we only have 128 bytes starting at 0x60, so the last valid address is
	; 0xDF, which obviously fits in the low byte
	mov		r20, XL
	subi	r20, low(SMS_frame_start)
	; subtract FBus header length to get the frame length
	subi	r20, 6

	; add a padding byte, if necessary
	mov		r16, XL
	andi	r16, 0x01
	breq	skip
	ldi		r16, 0
	st		X+, r16
skip:

	; write the frame length
	ldi		YL, low(SMS_frame_start)
	ldi		YH, high(SMS_frame_start)
	adiw	YL, 5
	st		Y, r20
	; calculate and write the message content length
	subi	r20, (6 + 12 + 5 + 12 + 7 + 2)
	adiw	YL, (6 + 12 + 5)
	st		Y, r20
	
	; FBus checksum: r18 - high byte, r17 - low byte
	ldi		r18, 0
	ldi		r17, 0
	ldi		YL, low(SMS_frame_start)
	ldi		YH, high(SMS_frame_start)
checksum:
	ld		r16, Y+
	eor		r18, r16
	ld		r16, Y+
	eor		r17, r16
	cp		YL, XL
	brlo	checksum
	; store the checksum
	st		X+, r18
	st		X+, r17
ret

; =============================================================================
; Data segment - SRAM region designations
; -----------------------------------------------------------------------------

.dseg

.org SRAM_START

SMS_frame_start:	.byte	12
SMS_SMSC_number:	.byte	12
SMS_TPDU:			.byte	5
SMS_dest_number:	.byte	12
SMS_VP:				.byte	7
SMS_user_data:

.eseg

.db 0x07, 0x91, 0x84, 0x06, 0x92, 0x15, 0x11, 0xF1, 0x00, 0x00, 0x00, 0x00
.db 0x0B, 0x91, 0x84, 0x97, 0x11, 0x86, 0x89, 0xF0, 0x00, 0x00, 0x00, 0x00
