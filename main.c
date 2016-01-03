/*-------------------------------------------------*/
/* Termometr AQUA - Attiny85   code by pvglab      */
/*-------------------------------------------------*/

#define DEBUG

#define TEMP_MINIMALNA 23
#define TEMP_MAKSYMALNA 27
#define BATERIA_MINIMUM 2600

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "dallas_one_wire.h"

#ifdef DEBUG
#include "suart.h"
#endif
#define	SYSCLK		500000UL

#define ASCII_SPACE 0x20
#define ASCII_ZERO 0x30

#define PORT PORTB
#define DDR DDRB
#define RED PB2
#define GREEN PB1
#define YELLOW PB0

//DS18B20 commands
#define CONVERT_T_COMMAND 0x44
#define READ_SCRATCHPAD_COMMAND 0xBE

#ifdef DEBUG
const char mess1[] PROGMEM="\r\n\nPomiar temperatury\r\n";
const char mess2[] PROGMEM="Szukam czujnika...\r\n";
const char mess_ok[] PROGMEM="OK: czujnik znaleziony\r\n\n";
const char mess_err1[] PROGMEM="ERR: brak czujnika\r\n";
const char mess_err2[] PROGMEM="ERR: wiecej niz jeden czujnik\r\n";
const char mess_err3[] PROGMEM="ERR: blad komunikacji z czujnikiem\r\n";
const char mess_koniec[] PROGMEM="Koniec programu\r\n";
const char mess_wynik1[] PROGMEM="Wynik pomiaru: ";
const char mess_wynik2[] PROGMEM=" C\r";
const char mess_bateria1[] PROGMEM="Napiecie baterii: ";
const char mess_bateria2[] PROGMEM="mV\r\n";
#endif


// Ustawienia Watchdog'a
static void __init3( 
	void ) 
	__attribute__ (( section( ".init3" ), naked, used )); 
static void __init3( 
	void ) 
{ 
	MCUSR = 0; 
	WDTCR = (1 << WDCE) | (1 << WDE); //Time sequence to enable Watchdog Changes !
#ifdef DEBUG
	WDTCR = (1 << WDIE) | (1 << WDP3); //Ustawienie przerwania co 4 sekundy
#else
	WDTCR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); //Ustawienie przerwania co 8 sekund
#endif
	//Ustawienie zegara na 500KHz
	cli();
	CLKPR = 0b10000000; //Prescaler change enable !
	CLKPR = 0b00000100; // Set prescaler to 16
	sei();
} 

DALLAS_IDENTIFIER_LIST_t *onewires;
uint8_t  wynik_szukania_onewire, messageBuf[10],temperatura[4] ={0x00,0x00,0x00,0x00}; //Tu przewchowywana bedzie odczytana temperatura, poczatkowo -99-
uint8_t wybudzenie=0;
uint16_t napiecie_baterii;
#ifdef DEBUG
static void pgm_xmit(const char *s);
#endif
void alert(uint8_t DIODA);
void sleep(void);
static void mierzTemperature(DALLAS_IDENTIFIER_LIST_t *onewires);
static void odczytajTemperature(DALLAS_IDENTIFIER_LIST_t *onewires,uint8_t *messageBuf,uint8_t *temperatura);
static void pokazTemperature(uint8_t *temperatura);
uint16_t getVCC(void);
void pokaz_VCC(uint16_t napiecie_baterii);


int main (void)
{
	DDRB |= (1<<YELLOW) | (1<<GREEN) | (1<<RED); //wyjscie do LED
	PORTB &= ~((1<<YELLOW) | (1<<GREEN) | (1<<RED)); //Po restarcie mrugniemy wszystkimi diodami testowo
	_delay_ms(50);
	PORTB |= (1<<YELLOW) | (1<<GREEN) | (1<<RED);
#ifdef DEBUG	
	pgm_xmit(mess1);
	pgm_xmit(mess2);
#endif	
	wynik_szukania_onewire=dallas_search_identifiers();
	if(wynik_szukania_onewire) //szukamy urzadzen - 0 gdy przeszukanie udane
	{
#ifdef DEBUG		
		pgm_xmit(mess_err1);
#endif
		goto KONIEC;
	}
	onewires=get_identifier_list();
	if(onewires->num_devices!=1) //Ma byc tylko jedno urządzenie póki co :)
	{
#ifdef DEBUG
		pgm_xmit(mess_err2);
#endif
		goto KONIEC;
	}	
#ifdef DEBUG
	pgm_xmit(mess_ok);
#endif
	while(1)
	{
		if(!wybudzenie)
			mierzTemperature(onewires); //Zlecamy pomiar temperatury
#ifdef DEBUG		
		xmit(wybudzenie+ASCII_ZERO);
#endif
		if(wybudzenie==1)
		{
			odczytajTemperature(onewires, messageBuf, temperatura);
			napiecie_baterii=getVCC();
#ifdef DEBUG
			pokaz_VCC(napiecie_baterii);
#endif
			if(napiecie_baterii < BATERIA_MINIMUM)
				temperatura[3]=2; //Wykorzystamy ta tablice, by przekazac info i wyczerpanej baterii
		}
		pokazTemperature(temperatura);
#ifdef DEBUG
		if(++wybudzenie>2)
#else
		if(++wybudzenie>37)
#endif
			wybudzenie=0;
		sleep();
	}
KONIEC:
#ifdef DEBUG
	pgm_xmit(mess_koniec);
#endif
	alert(YELLOW);	
	while(1);
}

ISR(WDT_vect)
{
}

#ifdef DEBUG
static void pgm_xmit(const char *s)
{
	char c;
	while(c = pgm_read_byte(s++))
		xmit(c);
}
#endif

void alert(uint8_t DIODA)
{
	PORTB &= ~(1<<DIODA);
	_delay_ms(50);
	PORTB |= (1<<DIODA);
	_delay_ms(100);
	PORTB &= ~(1<<DIODA);
	_delay_ms(50);
	PORTB |= (1<<DIODA);	
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
	DDRB |= (1<<YELLOW) | (1<<GREEN) | (1<<RED); //wyjscie do LED
	PORTB |= (1<<YELLOW) | (1<<GREEN) | (1<<RED);
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
#ifdef DEBUG
		pgm_xmit(mess_err3); // brak komunikacji z czujnikiem temperatury
#endif
		temperatura[3]=1;
		return;
	}
	temperatura[3]=0;
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
#ifdef DEBUG	
	pgm_xmit(mess_wynik1);
	xmit(temperatura[2]);
	xmit((temperatura[0]/10)+ASCII_ZERO);
	xmit((temperatura[0]%10)+ASCII_ZERO);
	xmit(',');
	xmit((((uint16_t)temperatura[1]*625)/1000)+ASCII_ZERO);
	pgm_xmit(mess_wynik2);
#endif
	if(temperatura[3])
	{
		if(temperatura[3]>1)
			alert(RED);
		else
			alert(YELLOW);
		return;
	}
	if(temperatura[0] < TEMP_MINIMALNA)
		PORTB &= ~(1<<YELLOW);
	if(temperatura[0] >= TEMP_MINIMALNA && temperatura[0] < TEMP_MAKSYMALNA )
		PORTB &= ~(1<<GREEN);
	if(temperatura[0] >= TEMP_MAKSYMALNA)
		PORTB &= ~(1<<RED);
	_delay_ms(50);
	PORTB |= (1<<YELLOW) | (1<<GREEN) | (1<<RED);
}

uint16_t getVCC(void) 
{
	ADCSRA |= (1<<ADEN); //Włączenie przetwornika AD
	ADCSRA |= (1<<ADPS1) | (1<<ADPS0); //Preskaler 8 dla przetwornika AD (przy 0.5MHz clk)
	ADMUX = (1<<MUX3) | (1<<MUX2); // For ATtiny85
	_delay_ms(2); // Wait for Vref to settle
	ADCSRA |= (1<<ADSC); // Convert
	while (ADCSRA & (1<<ADSC));
	uint8_t low = ADCL;
	uint16_t val = ((ADCH&0x03) << 8) | low;
	//discard previous result
	ADCSRA |= (1<<ADSC); // Convert
	while (ADCSRA & (1<<ADSC));
	low = ADCL;
	val = ((ADCH&0x03) << 8) | low;
	ADCSRA &= ~(1<<ADEN); //Wyłączenie przetwornika AD
	return (uint16_t)(((uint32_t)1024 * 1100) / val);
}

void pokaz_VCC(uint16_t napiecie_baterii)
{
	pgm_xmit(mess_bateria1);
	xmit((napiecie_baterii/1000)+ASCII_ZERO);
	xmit(((napiecie_baterii%1000)/100)+ASCII_ZERO);
	xmit(((napiecie_baterii%100)/10)+ASCII_ZERO);
	xmit(ASCII_ZERO); //jednostki sobie darujemy :)
	pgm_xmit(mess_bateria2);	
	
}
