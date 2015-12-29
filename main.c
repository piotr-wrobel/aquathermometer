/*------------------------------------------------*/
/* A test program for software UART       */
/*------------------------------------------------*/



#include <avr/io.h>
#include <avr/pgmspace.h>
//#include <util/delay.h>
//#include <avr/interrupt.h>
#include "suart.h"

#define	SYSCLK		8000000UL

const char mess1[] PROGMEM="Witaj w programie testowym, wybierz komende (1,2,3)\r\n";
const char mess2[] PROGMEM="\r\nKomenda nieznana ! sprobuj ponownie\r\n";
const char mess3[] PROGMEM="\r\nWybrales komende 1\r\n";
const char mess4[] PROGMEM="\r\nWybrales komende 2\r\n";
const char mess5[] PROGMEM="\r\nWybrales komende 3\r\n";

// // Wylaczenie WatchDoga
// #ifdef WDIF 
    // static void __init3( 
        // void ) 
        // __attribute__ (( section( ".init3" ), naked, used )); 
    // static void __init3( 
        // void ) 
    // { 
        // /* wyłączenie watchdoga (w tych mikrokontrolerach, w których watchdog 
         // * ma możliwość generowania przerwania pozostaje on też aktywny po 
         // * resecie) */ 
        // MCUSR = 0; 
        // WDTCR = 1 << WDCE | 1 << WDE; 
        // WDTCR = 0; 
    // } 
// #endif

int main (void)
{
	const char *s;
	char c;
	
	/* Send message */
	s = mess1;

	for (;;) {
		c = pgm_read_byte(s++);
		if (!c) break;
		xmit(c);
	}

	/* Receives data and echos it in incremented data */
	for(;;) {
		c = rcvr();
		switch(c)
		{
			case '1':
				s = mess3;
			break;
			case '2':
				s = mess4;
			break;
			case '3':
				s = mess5;
			break;
			default:
				s = mess2;
		}
		for (;;) {
			c = pgm_read_byte(s++);
			if (!c) break;
			xmit(c);
		}
	}

}

