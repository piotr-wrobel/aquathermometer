;---------------------------------------------------------------------------;
; Software implemented UART module                                          ;
; (C)ChaN, 2005 (http://elm-chan.org/)                                      ;
; tuned by pvg (2015)														;
;---------------------------------------------------------------------------;
; Bit rate settings:
;
;            250kHz .5MHz 1MHz  2MHz  4MHz  6MHz  8MHz  10MHz  12MHz  16MHz  20MHz
;   2.4kbps      33    68  138     -     -     -     -      -      -      -      -
;   4.8kbps      15    33   68   138     -     -     -      -      -      -      -
;   9.6kbps       7    15   33    68   138   208     -      -      -      -      -
;  19.2kbps       -     7   15    33    68   102   138    173    208      -      -
;  38.4kbps       -     -    7    15    33    50    68     85    102    138    172
;  57.6kbps       -     -    -     -    21    33    44     56     68     91    114
; 115.2kbps       -     -    -     -     -     7    21     27     33     44     56

.nolist
#include <avr/io.h>
.list

#define	BPS	7	/* Bit delay. (see above table) */
#define	BIDIR	1	/* 0:Separated Tx/Rx, 1:Shared Tx/Rx */

#define OUT_PIN PB0
#define IN_PIN PB0
#define PORT_SUART PORTB
#define DDR_SUART DDRB



#define	DIR_OUT		sbi _SFR_IO_ADDR(DDR_SUART), OUT_PIN	/* Pin direction OUT */
#define	DIR_IN		cbi _SFR_IO_ADDR(DDR_SUART), IN_PIN		/* Pin direction IN */
#define	OUT_0		cbi _SFR_IO_ADDR(PORT_SUART), OUT_PIN	/* Output 0 */
#define	OUT_1		sbi _SFR_IO_ADDR(PORT_SUART), OUT_PIN	/* Output 1 */
#define	SKIP_IN_0	sbic _SFR_IO_ADDR(PINB), IN_PIN			/* Skip if 0 */
#define	SKIP_IN_1	sbis _SFR_IO_ADDR(PINB), IN_PIN			/* Skip if 1 */



#ifdef SPM_PAGESIZE
.macro	_LPMI	reg
	lpm	\reg, Z+
.endm
.macro	_MOVW	dh,dl, sh,sl
	movw	\dl, \sl
.endm
#else
.macro	_LPMI	reg
	lpm
	mov	\reg, r0
	adiw	ZL, 1
.endm
.macro	_MOVW	dh,dl, sh,sl
	mov	\dl, \sl
	mov	\dh, \sh
.endm
#endif



;---------------------------------------------------------------------------;
; Transmit a byte in serial format of N81
;
;Prototype: void xmit (uint8_t data);
;Size: 16 words

.global xmit
.func xmit
xmit:
	DIR_OUT
	OUT_1
#if BIDIR
	ldi	r23, BPS-1	;Pre-idle time for bidirectional data line
5:	dec	r23     	;
	brne	5b		;/
#endif
	in	r0, _SFR_IO_ADDR(SREG)	;Save flags
	cli			;Start critical section
	com	r24		;C = start bit
	ldi	r25, 10		;Bit counter
1:	ldi	r23, BPS-1	;----- Bit transferring loop
2:	dec	r23     	;Wait for a bit time
	brne	2b		;/
	brcs	3f		;MISO = bit to be sent
	OUT_1			;
3:	brcc	4f		;
	OUT_0			;/
4:	lsr	r24     	;Get next bit into C
	dec	r25     	;All bits sent?
	brne	1b	     	;  no, coutinue
#if BIDIR
	DIR_IN
#endif
	out	_SFR_IO_ADDR(SREG), r0	;End of critical section
	sei
	ret
.endfunc



;---------------------------------------------------------------------------;
; Receive a byte
;
;Prototype: uint8_t rcvr (void);
;Size: 19 words

.global rcvr
.func rcvr
rcvr:
	in	r0, _SFR_IO_ADDR(SREG)	;Save flags
	cli			;Start critical section
	ldi	r24, 0x80	;Receiving shift reg

1:	SKIP_IN_1		;Wait for idle
	rjmp	1b
2:	SKIP_IN_0		;Wait for start bit
	rjmp	2b
	ldi	r25, BPS/2	;Wait for half bit time
3:	dec	r25
	brne	3b

4:	ldi	r25, BPS	;----- Bit receiving loop
5:	dec	r25     	;Wait for a bit time
	brne	5b		;/
	lsr	r24     	;Next bit
	SKIP_IN_0		;Get a data bit into r24.7
	ori	r24, 0x80
	brcc	4b	     	;All bits received?  no, continue

	out	_SFR_IO_ADDR(SREG), r0	;End of critical section
	sei
	ret
.endfunc
