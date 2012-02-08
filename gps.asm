; AVR GSM-based (Nokia FBus) GPS locator
; Written by Leszek Godlewski <lg@inequation.org>

.nolist
.include "tn2313def.inc"
.list

; This ROM is written for an ATtiny2313 running on a 3.6864MHz quartz.

; =============================================================================
; Register assignments
; -----------------------------------------------------------------------------

; FBus packet sequence number register.
.def	FPSR		=	r25
; GPS parsing state register.
.def	GPSSR		=	r24
; GPS parsing counter
.def	GPSCNT		=	r23
; GPS helper register
.def	GPSHLP		=	r22
; GPS fix availability register
.def	GPSFIX		=	r21

; =============================================================================
; Constants
; -----------------------------------------------------------------------------

; UBRR setting for 9600 baud (assuming the 3.6864 quartz!)..
.equ	UBRR_9600	=	23
; UBRR setting for 115200 baud.
.equ	UBRR_115k2	=	1

; Port B value for enabling FBus comms - low on PB.0, high on PB.1.
.equ	ENABLE_FBUS	=	0x02
; Port B value for enabling GPS comms - high on PB.0, low on PB.1.
.equ	ENABLE_GPS	=	0x01

; GPS parsing state: awaiting frame start. All input is ignored until a '$'
; character is received.
.equ	GPSS_IDLE		=	0x00
; GPS parsing state: decoding frame type.
.equ	GPSS_DECODING	=	0x01
; GPS parsing state: GSA sentence, mode 1.
.equ	GPSS_GSA_MODE_1	=	0x02
; GPS parsing state: GSA sentence, mode 2. We don't care about anything past.
.equ	GPSS_GSA_MODE_2	=	0x03
; GPS parsing state: RMC sentence, UTC timestamp.
.equ	GPSS_RMC_TIME	=	0x04
; GPS parsing state: RMC sentence, data validity status.
.equ	GPSS_RMC_VALID	=	0x05
; GPS parsing state: RMC sentence, coordinates.
.equ	GPSS_RMC_COORDS	=	0x06
; GPS parsing state: RMC sentence, ground speed.
.equ	GPSS_RMC_SPEED	=	0x07
; GPS parsing state: RMC sentence, course. We don't care about anything past.
.equ	GPSS_RMC_COURSE	=	0x08
; All other NMEA sentences are ignored.

; =============================================================================
; Macros
; -----------------------------------------------------------------------------

; A macro to clear the Sleep Enable bit.
.macro		sleep_disable
	in		r16, MCUCR
	andi	r16, ~(1 << SE)
	out		MCUCR, r16
.endmacro

; A macro to set the Sleep Enable bit.
.macro		sleep_enable
	in		r16, MCUCR
	ori		r16, (1 << SE)
	out		MCUCR, r16
.endmacro

; A macro to enable the UART Rx complete interrupt
.macro		RxC_enable
	in		r16, UCSRB
	ori		r16, (1 << RXCIE)
	out		UCSRB, r16
.endmacro

; A macro to disable the UART Rx complete interrupt
.macro		RxC_disable
	in		r16, UCSRB
	andi	r16, ~(1 << RXCIE)
	out		UCSRB, r16
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

; A macro that forces an SMS commit if the buffer is about to overflow
.macro		check_SMS_buffer
	cpi		XL, SMS_footer
	brsh	GPS_commit
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
	.db	0x11, 0x00, 0x00, 0xF4, 0xFF

; Validity Period descriptor.
tpl_SMS_VP:
	.db 0xA9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

tpl_SMS_report_online:
	.db "avr-fbus-gps online"

; =============================================================================
; Reset handler
; -----------------------------------------------------------------------------

reset:
	; no GPS fix yet
	ldi		GPSFIX, 0

	; set port B pins 2-7 as inputs, pins 0-1 as outputs
	ldi		r16, 0x03
	out		DDRB, r16
	; set PB0 and PB1 to high, i.e. don't enable any of the UART devices
	ldi		r16, 0x03
	out		PORTB, r16

	; initialize the stack pointer - set it to the a part of the register file
	; to save on SRAM, we're not getting the stack any deeper than 3 levels
	; anyway
	ldi		r16, RAMEND;0x08
	out		SPL, r16

	; switch to FBus
	rcall	UART_to_FBus

	; enable receiver and transmitter
	ldi		r16, (1 << RXEN) | (1 << TXEN)
	out		UCSRB, r16
	; default UART settings are exactly what we need (8 data bits, no parity, 1
	; stop bit), so no additional initialization is needed

	; initialize FBus packet sequence number
	ldi		FPSR, 0x40

	; report in to the "nexus" - send an SMS to the registered number
	; assemble the appropriate FBus frame
	rcall	FBus_start
	; write the report message into the frame
	copy_PM	tpl_SMS_report_online, 19	
	; finish off the frame and fill in all length bytes and calculate checksums
	rcall	FBus_finish

	; synchronize the phone's UART
	rcall	FBus_sync
	; send in the message
	rcall	FBus_send

	; switch to GPS comms and go to sleep, waiting for GPS updates
	ldi		GPSSR, GPSS_IDLE
	rcall	UART_to_GPS

	; global interrupt enable
	sei

; =============================================================================
; Idle loop (go back to sleep after waking to handle an interrupt)
; -----------------------------------------------------------------------------

	; we've configured everything, now go to idle state to save power
idle:
	sleep_enable
	sleep
	rjmp	idle

; =============================================================================
; UART receive complete interrupt handler
; -----------------------------------------------------------------------------

RX_complete:
	; disable sleeping for stability
	sleep_disable

	; disable interrupts
	cli

	; start by reading the awaiting byte
	rcall	UART_Rx
	; state machine
	cpi		GPSSR, GPSS_IDLE
	breq	GPS_idle
	cpi		GPSSR, GPSS_DECODING
	breq	GPS_decoding
	cpi		GPSSR, GPSS_GSA_MODE_1
	breq	GPS_GSA_mode_1
	cpi		GPSSR, GPSS_GSA_MODE_2
	breq	GPS_GSA_mode_2
	cpi		GPSSR, GPSS_RMC_TIME
	breq	GPS_RMC_time
	cpi		GPSSR, GPSS_RMC_VALID
	breq	GPS_RMC_validity
	cpi		GPSSR, GPSS_RMC_COORDS
	breq	GPS_RMC_coords
	cpi		GPSSR, GPSS_RMC_SPEED
	breq	GPS_RMC_speed
	cpi		GPSSR, GPSS_RMC_COURSE
	breq	GPS_RMC_course_proxy

GPS_idle:
	; if the character is not '$', ignore it altogether
	cpi		r16, '$'
	brne	RX_skip_proxy
	; otherwise skip to sentence decoding state
	ldi		GPSCNT, 0
	ldi		GPSSR, GPSS_DECODING
	rjmp	RX_skip

GPS_decoding:
	; there's nothing interestng until character #4, which identifies the
	; sentence unambiguously
	inc		GPSCNT
	cpi		GPSCNT, 4
	brne	GPS_decoding_cmp
	; save off the character identifying the sentence
	mov		GPSHLP, r16
	rjmp	RX_skip
GPS_decoding_cmp:
	; if it's a comma, it's time to switch states
	cpi		r16, ','
	brne	RX_skip_proxy
	cpi		GPSHLP, 'S'
	breq	GPS_decoding_GSA
	cpi		GPSHLP, 'M'
	breq	GPS_decoding_RMC
	; otherwise it's a sentence we don't care about
	ldi		GPSCNT, 0
	ldi		GPSSR, GPSS_IDLE
	rjmp	RX_skip
GPS_decoding_GSA:
	ldi		GPSCNT, 0
	ldi		GPSSR, GPSS_GSA_MODE_1
	rjmp	RX_skip
GPS_decoding_RMC:
	ldi		GPSCNT, 0
	ldi		GPSSR, GPSS_RMC_TIME
	; set the write pointer at the beginning of the SMS data frame so that we
	; can begin writing at will
	ldi		XL,	low(SMS_user_data)
	ldi		XH, high(SMS_user_data)
	rjmp	RX_skip

GPS_GSA_mode_1:
	; don't care until a comma comes in
	cpi		r16, ','
	brne	RX_skip
	ldi		GPSSR, GPSS_GSA_MODE_2
	rjmp	RX_skip

GPS_GSA_mode_2:
	cpi		r16, ','
	brne	GPS_GSA_mode_2_fix
	ldi		GPSSR, GPSS_GSA_MODE_2
	rjmp	RX_skip
GPS_GSA_mode_2_fix:
	; the mode is either 1, 2 or 3; 1 means no fix 
	subi	r16, '1'
	mov		GPSFIX, r16
	; we don't care about the remainder of the frame, switch state back to idle
	ldi		GPSCNT, 0
	ldi		GPSSR, GPSS_IDLE
	rjmp	RX_skip

; Proxy label, this one is too far for branch instructions addressing.
RX_skip_proxy:
	rjmp	RX_skip
; Proxy label, this one is too far for branch instructions addressing.
GPS_RMC_course_proxy:
	rjmp	GPS_RMC_course

GPS_RMC_time:
	; commit if buffer already full
	;check_SMS_buffer
	st		X+, r16
	cpi		r16, ','
	brne	RX_skip
	ldi		GPSSR, GPSS_RMC_VALID
	rjmp	RX_skip

GPS_RMC_validity:
	; don't care until a comma comes in
	cpi		r16, ','
	brne	RX_skip
	ldi		GPSSR, GPSS_RMC_COORDS
	rjmp	RX_skip

GPS_RMC_coords:
	; commit if buffer already full
	;check_SMS_buffer
	st		X+, r16
	cpi		r16, ','
	brne	RX_skip
	; switch state after writing 4 fields (4 commas)
	inc		GPSCNT
	cpi		GPSCNT,	4
	brne	RX_skip
	ldi		GPSCNT, 0
	ldi		GPSSR, GPSS_RMC_SPEED
	rjmp	RX_skip

GPS_RMC_speed:
	; commit if buffer already full
	check_SMS_buffer
	st		X+, r16
	cpi		r16, ','
	brne	RX_skip
	ldi		GPSSR, GPSS_RMC_COURSE
	rjmp	RX_skip

GPS_RMC_course:
	; commit if buffer already full
	check_SMS_buffer
	; first check for the trailing comma
	cpi		r16, ','
	; we've got all the data we wanted, send it away
	breq	GPS_commit
	st		X+, r16
	rjmp	RX_skip

GPS_commit:
	; is there a fix available?
	cpi		GPSFIX, 0
	breq	RX_skip
	; yes, there is! finish the SMS frame and send it!
	rjmp	FBus_finish
	; switch to FBus comms
	rcall	UART_to_FBus
	rcall	FBus_sync
	rcall	FBus_send
	; restore GPS comms
	rcall	UART_to_GPS
	; revert GPS state
	ldi		GPSCNT, 0
	ldi		GPSSR, GPSS_IDLE
	rjmp	RX_skip

RX_skip:
	; reenable interrupts
	sei
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
	;rjmp	UART_Tx
	out		UDR, r16
ret

; Reads a byte into r16 from UART.
UART_Rx:
	; wait for full buffer
	sbis	UCSRA, RXC
	rjmp	UART_Rx
	in 		r16, UDR
ret

; Switches UART to FBus comms mode.
UART_to_FBus:
	; set low on PB.1
	ldi		r16, ENABLE_FBUS
	out		PORTB, r16
	; set the baud rate to 115k2
	ldi		r16, UBRR_115k2
	out		UBRRL, r16
	ldi		r16, 0
	out		UBRRH, r16
	; disable the Rx complete interrupt
	in		r16, UCSRB
	andi	r16, ~(1 << RXCIE)
	out		UCSRB, r16
ret

; Switches UART to GPS comms mode.
UART_to_GPS:
	; set low on PB.0
	ldi		r16, ENABLE_GPS
	out		PORTB, r16
	; set the baud rate to 9600
	ldi		r16, UBRR_9600
	out		UBRRL, r16
	ldi		r16, 0
	out		UBRRH, r16
	; enable the Rx complete interrupt
	in		r16, UCSRB
	ori		r16, (1 << RXCIE)
	out		UCSRB, r16
ret

; =============================================================================
; FBus procedures
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
	; write in the number of packets left to transmit (always 1)
	ldi		r16, 1
	st		X+, r16
	; write in the sequence number
	st		X+, FPSR
	inc		FPSR
	andi	FPSR, 0x07
	ori		FPSR, 0x60

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

; Sends the contents of SRAM from the beginning to X - 1 over UART.
FBus_send:
	ldi		YL, low(SMS_frame_start)
	ldi		YH, high(SMS_frame_start)
send_loop:
	ld		r16, Y+
	rcall	UART_Tx
	cp		YL, XL
	brlo	send_loop
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

; Reserve space for the FBus footer.
.org (RAMEND - 4)
SMS_footer:

.eseg

.db 0x07, 0x91, 0x84, 0x06, 0x92, 0x15, 0x11, 0xF1, 0x00, 0x00, 0x00, 0x00
.db 0x0B, 0x91, 0x84, 0x97, 0x11, 0x86, 0x89, 0xF0, 0x00, 0x00, 0x00, 0x00
