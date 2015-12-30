/*-------------------------------------------------*/
/* Termometr AQUA - Attiny85   code by pvglab      */
/*-------------------------------------------------*/



#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "dallas_one_wire.h"
#include "suart.h"

#define	SYSCLK		8000000UL

#define ASCII_SPACE 0x20
#define ASCII_ZERO 0x30

//DS18B20 commands
#define CONVERT_T_COMMAND 0x44
#define READ_SCRATCHPAD_COMMAND 0xBE

const char mess1[] PROGMEM="\r\n\nPomiar temperatury\r\n";
const char mess2[] PROGMEM="Szukam czujnika...\r\n";
const char mess_ok[] PROGMEM="OK: czujnik znaleziony\r\n\n";
const char mess_err1[] PROGMEM="ERR: brak czujnika\r\n";
const char mess_err2[] PROGMEM="ERR: wiecej niz jeden czujnik\r\n";
const char mess_err3[] PROGMEM="ERR: blad komunikacji z czujnikiem\r\n";
const char mess_koniec[] PROGMEM="Koniec programu\r\n";
const char mess_wynik1[] PROGMEM="Wynik pomiaru: ";
const char mess_wynik2[] PROGMEM=" C\r";



// Ustawienia Watchdog'a
#ifdef WDIF 
    static void __init3( 
        void ) 
        __attribute__ (( section( ".init3" ), naked, used )); 
    static void __init3( 
        void ) 
    { 
        /* wyłączenie watchdoga (w tych mikrokontrolerach, w których watchdog 
         * ma możliwość generowania przerwania pozostaje on też aktywny po 
         * resecie) */ 
        MCUSR = 0; 
        WDTCR = (1 << WDCE) | (1 << WDE); //Time sequence to enable Watchdog Changes !
        WDTCR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1); //Ustawienie przerwania co 4 sekundy 
    } 
#endif

DALLAS_IDENTIFIER_LIST_t *onewires;
uint8_t  wynik_szukania_onewire, messageBuf[10],temperatura[3] ={0xDD,ASCII_SPACE,ASCII_SPACE}; //Tu przewchowywana bedzie odczytana temperatura, poczatkowo -99-

static void pgm_xmit(const char *s);
static void mierzTemperature(DALLAS_IDENTIFIER_LIST_t *onewires);
static void odczytajTemperature(DALLAS_IDENTIFIER_LIST_t *onewires,uint8_t *messageBuf,uint8_t *temperatura);

int main (void)
{
	pgm_xmit(mess1);
	pgm_xmit(mess2);
	
	wynik_szukania_onewire=dallas_search_identifiers();
	if(wynik_szukania_onewire) //szukamy urzadzen - 0 gdy przeszukanie udane
	{
		
		pgm_xmit(mess_err1);
		goto KONIEC;
	}
	onewires=get_identifier_list();
	if(onewires->num_devices!=1) //Ma byc tylko jedno urządzenie póki co :)
	{
		pgm_xmit(mess_err2);
		goto KONIEC;
	}	
	pgm_xmit(mess_ok);
	while(1)
	{
		mierzTemperature(onewires); //Zlecamy pomiar temperatury
		_delay_ms(1000); //Czekamy na pomiar
		odczytajTemperature(onewires, messageBuf, temperatura);
		_delay_ms(4000); //Czekamy
		
	}
KONIEC:
	pgm_xmit(mess_koniec);
	while(1);
}

ISR(WDT_vect)
{
	xmit('!');
	xmit('\n');	
}

static void pgm_xmit(const char *s)
{
	char c;
	while(c = pgm_read_byte(s++))
		xmit(c);
}

static void mierzTemperature(DALLAS_IDENTIFIER_LIST_t *onewires)
{
	dallas_match_rom(&(onewires->identifiers[0])); //Wybieramy pierwsze urzadzenie
	dallas_write_byte(CONVERT_T_COMMAND); //Zlecamy pomiar temperatury
}

static void odczytajTemperature(DALLAS_IDENTIFIER_LIST_t *onewires,uint8_t *messageBuf,uint8_t *temperatura)
{
	dallas_match_rom(&(onewires->identifiers[0])); //Wybieramy pierwsze urzadzenie
	dallas_write_byte(READ_SCRATCHPAD_COMMAND); //Zlecamy odczyt temperatury
	dallas_read_buffer(messageBuf,2);
	if(!dallas_reset())
	{
		pgm_xmit(mess_err3); // brak komunikacj iz czujnikiem temperatury
		return;
	}
	temperatura[0]=((messageBuf[1] & 0x07)<<4)|messageBuf[0]>>4; // Tutaj wartosci calkowite temp bez znaku - 7 bitow
	temperatura[1]= (messageBuf[1]&0xF0)|(messageBuf[0]&0x0F); // 4 starsze bity to znak, mlodsze 4 to wartosc po przecinku
	char znak='+';
	if(temperatura[1]&0xF0) //temperatura ujemna
	{
		temperatura[0]= ~temperatura[0];
		temperatura[1]= ~temperatura[1];
		znak='-';
	}
	
	pgm_xmit(mess_wynik1);
	xmit(znak);
	xmit((temperatura[0]/10)+ASCII_ZERO);
	xmit((temperatura[0]%10)+ASCII_ZERO);
	xmit(',');
	xmit((((uint16_t)temperatura[1]*625)/1000)+ASCII_ZERO);
	pgm_xmit(mess_wynik2);
}


