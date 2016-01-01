/*-------------------------------------------------*/
/* Termometr AQUA - Attiny85   code by pvglab      */
/*-------------------------------------------------*/



#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "dallas_one_wire.h"
#include "suart.h"

#define	SYSCLK		1000000UL

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
static void __init3( 
	void ) 
	__attribute__ (( section( ".init3" ), naked, used )); 
static void __init3( 
	void ) 
{ 
	MCUSR = 0; 
	WDTCR = (1 << WDCE) | (1 << WDE); //Time sequence to enable Watchdog Changes !
	WDTCR = (1 << WDIE) | (1 << WDP3);// | (1 << WDP0); //Ustawienie przerwania co 4 sekund 
} 

DALLAS_IDENTIFIER_LIST_t *onewires;
uint8_t  wynik_szukania_onewire, messageBuf[10],temperatura[3] ={0x00,0x00,0x00}; //Tu przewchowywana bedzie odczytana temperatura, poczatkowo -99-
uint8_t wybudzenie=0;

static void pgm_xmit(const char *s);
void sleep(void);
static void mierzTemperature(DALLAS_IDENTIFIER_LIST_t *onewires);
static void odczytajTemperature(DALLAS_IDENTIFIER_LIST_t *onewires,uint8_t *messageBuf,uint8_t *temperatura);
static void pokazTemperature(uint8_t *temperatura);


int main (void)
{
	// DDRB |= (1<<PB1); //wyjscie do zasilania 18B20
	// PORTB |= (1<<PB1);
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
		if(!wybudzenie)
			mierzTemperature(onewires); //Zlecamy pomiar temperatury
		xmit(wybudzenie+ASCII_ZERO);
		if(wybudzenie==1)
			odczytajTemperature(onewires, messageBuf, temperatura);
		pokazTemperature(temperatura);
		if(++wybudzenie>5)
			wybudzenie=0;
		sleep();
	}
KONIEC:
	pgm_xmit(mess_koniec);
	while(1);
}

ISR(WDT_vect)
{
}

static void pgm_xmit(const char *s)
{
	char c;
	while(c = pgm_read_byte(s++))
		xmit(c);
}

void sleep(void)
{
	cli();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	power_all_disable();
	DDRB=0x00;
	PORTB=0x00;
	// power_usi_disable()
	// power_adc_disable()
	// power_timer0_disable()
	// power_timer1_disable(); // OR ? power_all_disable() !
	sleep_enable();
	sei(); //first instruction after SEI is guaranteed to execute before any interrupt
	sleep_cpu();
	/* The program will continue from here. */
	/* First thing to do is disable sleep. */
	sleep_disable();
	power_all_enable();
	// DDRB |= (1<<PB1); //wyjscie do zasilania 18B20
	// PORTB |= (1<<PB1);
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
	temperatura[2]='+';
	if(temperatura[1]&0xF0) //temperatura ujemna
	{
		temperatura[0]= ~temperatura[0];
		temperatura[1]= ~temperatura[1];
		temperatura[2]='-';
	}
}

static void pokazTemperature(uint8_t *temperatura)
{
	pgm_xmit(mess_wynik1);
	xmit(temperatura[2]);
	xmit((temperatura[0]/10)+ASCII_ZERO);
	xmit((temperatura[0]%10)+ASCII_ZERO);
	xmit(',');
	xmit((((uint16_t)temperatura[1]*625)/1000)+ASCII_ZERO);
	pgm_xmit(mess_wynik2);	
}
