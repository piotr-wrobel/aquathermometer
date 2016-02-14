/*-------------------------------------------------*/
/* Termometr AQUA - Attiny45/85   code by pvglab      */
/*-------------------------------------------------*/

//#define DEBUG
//#define UART
#ifdef DEBUG //W trybie DEBUG obnizony dolny prog, by latwiej debugowac PWM na zielonej LED
	#define TEMP_MINIMALNA 20
#else
	#define TEMP_MINIMALNA 23
#endif
#define TEMP_MAKSYMALNA 27
#define BATERIA_MINIMUM 2600

#define BATERIA_WYCZERPANA 2
#define AWARIA_CZUJNIKA 1
#define OK 0

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "dallas_one_wire.h"

#ifdef UART
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

#define PHOTOTRANSISTOR PB4

//DS18B20 commands
#define CONVERT_T_COMMAND 0x44
#define READ_SCRATCHPAD_COMMAND 0xBE

#ifdef UART
const char mess1[] PROGMEM="\r\n\nTermometr\r\n";
const char mess2[] PROGMEM="Szukam czujnika...\r\n";
#ifdef DEBUG
const char mess3[] PROGMEM="licznik(0x00FF):licznik(0x0004)\r\n";
#else
const char mess3[] PROGMEM="licznik(0x00FF):licznik(0x0025)\r\n";
#endif
const char mess_ok[] PROGMEM="OK: czujnik znaleziony\r\n\n";
const char mess_err1[] PROGMEM="ERR: brak czujnika\r\n";
const char mess_err2[] PROGMEM="ERR: wiecej niz jeden czujnik\r\n";
const char mess_err3[] PROGMEM="ERR: blad komunikacji z czujnikiem\r\n";
const char mess_koniec[] PROGMEM="Koniec programu\r\n";
const char mess_wynik1[] PROGMEM="Wynik pomiaru: ";
const char mess_wynik2[] PROGMEM=" C\r";
const char mess_bateria1[] PROGMEM="Napiecie baterii: ";
const char mess_bateria2[] PROGMEM="mV\r\n";
const char mess_swiatlo1[] PROGMEM="Poziom swiatla: ";
const char mess_swiatlo2[] PROGMEM="/1024\r\n; ";
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
	WDTCR = (1 << WDE) | (1 << WDIE) | (1 << WDP3); //Ustawienie przerwania co 4 sekundy, wlaczenie WD
#else
	WDTCR = (1 << WDE) | (1 << WDIE) | (1 << WDP3) | (1 << WDP0); //Ustawienie przerwania co 8 sekund, wlaczenie WD
#endif
	//Ustawienie zegara na 500KHz
	cli();
	CLKPR = 0b10000000; //Prescaler change enable !
	CLKPR = 0b00000100; // Set prescaler to 16
	sei();
} 

DALLAS_IDENTIFIER_LIST_t *onewires;
uint8_t  wynik_szukania_onewire, messageBuf[10],temperatura[4] ={0x00,0x00,0x00,0x00}; //Tu przewchowywana bedzie odczytana temperatura, poczatkowo -99-
char napis[5];
uint8_t wybudzenie=0, wybudzenie_2=0;
uint16_t napiecie_baterii,poziom_swiatla=0;
#ifdef UART
static void pgm_xmit(const char *s);
static void  string_xmit(char *s);
static void UARTuitoa(uint16_t liczba, char *string);
#endif
void alert(uint8_t DIODA);
void sleep(void);
static void mierzTemperature(DALLAS_IDENTIFIER_LIST_t *onewires);
static void odczytajTemperature(DALLAS_IDENTIFIER_LIST_t *onewires,uint8_t *messageBuf,uint8_t *temperatura);
static void pokazTemperature(uint8_t *temperatura, uint16_t poziom_swiatla);
uint16_t getVCC(void);
uint16_t getLightLevel(void);
#ifdef UART
void pokazLiczbe(uint16_t liczba);
void pokaz_VCC(uint16_t napiecie_baterii);
void pokazPoziomSwiatla(uint16_t poziom_swiatla);
#endif

int main (void)
{
	DDR |= (1<<YELLOW) | (1<<GREEN) | (1<<RED); //wyjscie do LED
	PORT &= ~((1<<YELLOW) | (1<<GREEN) | (1<<RED)); //Po restarcie mrugniemy wszystkimi diodami testowo
	_delay_ms(50);
	PORT |= (1<<YELLOW) | (1<<GREEN) | (1<<RED);
#ifdef UART	
	pgm_xmit(mess1);
	pgm_xmit(mess3);
	pgm_xmit(mess2);
#endif	
	wynik_szukania_onewire=dallas_search_identifiers();
	if(wynik_szukania_onewire) //szukamy urzadzen - 0 gdy przeszukanie udane
	{
#ifdef UART		
		pgm_xmit(mess_err1);
#endif
		goto KONIEC;
	}
	onewires=get_identifier_list();
	if(onewires->num_devices!=1) //Ma byc tylko jedno urządzenie póki co :)
	{
#ifdef UART
		pgm_xmit(mess_err2);
#endif
		goto KONIEC;
	}	
#ifdef UART
	pgm_xmit(mess_ok);
#endif
	while(1)
	{
		if(!wybudzenie)
			mierzTemperature(onewires); //Zlecamy pomiar temperatury
#ifdef UART		
		UARTuitoa(wybudzenie_2,napis);
		string_xmit(napis);
		xmit(':');
		UARTuitoa(wybudzenie,napis);
		string_xmit(napis);
		xmit('>');
#endif
		if(wybudzenie==1)
		{
			odczytajTemperature(onewires, messageBuf, temperatura);
			if(!wybudzenie_2) //Pomiar napiecia tylko przy przepełnieniu drugiego licznika
			{
				napiecie_baterii=getVCC();
#ifdef UART
				pokaz_VCC(napiecie_baterii);
#endif
				if(napiecie_baterii < BATERIA_MINIMUM)
					temperatura[3]=BATERIA_WYCZERPANA; //Wykorzystamy ta tablice, by przekazac info i wyczerpanej baterii
			}
			if(temperatura[3]==OK) //Jesli z bateria i czujnikiem wszystko ok, to zmierzymy poziom swiatla
				poziom_swiatla=getLightLevel();
#ifdef UART
			pokazPoziomSwiatla(poziom_swiatla);
#endif
		}
		pokazTemperature(temperatura,poziom_swiatla);
#ifdef DEBUG
		if(++wybudzenie>4) //Pierwszy licznik liczy do 20 sekund (5 x 4 sekundy WD)
#else
		if(++wybudzenie>37) //Pierwszy licznik liczy do 5 minut (38 x 8 sekund WD)
#endif
		{	
			wybudzenie=0;
			wybudzenie_2++; //Drugi licznik to 256 cykli pierwszego licznika, czyli odpowiednio
		}					// 85 minut lub około 21 godzin
		
		WDTCR |= (1 << WDIE); //Odnowienie przerwania, czyli odroczenie resetu :)
		sleep();
	}
KONIEC:
#ifdef UART
	pgm_xmit(mess_koniec);
#endif
	alert(YELLOW);	
	while(1);
}

ISR(WDT_vect){}

#ifdef UART
static void pgm_xmit(const char *s)
{
	char c;
	while(c = pgm_read_byte(s++))
		xmit(c);
}

static void  string_xmit(char *s)
{
    uint8_t cnt = 0;
	while (s[cnt])
		xmit(s[cnt++]);
}

static void UARTuitoa(uint16_t liczba, char *string)
{
	uint8_t nibble=0,pozycja;
	for(pozycja=0;pozycja<4;pozycja++)
	{
		nibble=(liczba>>12);
		liczba=(liczba<<4);
		if(nibble <= 9)
			nibble+=48;
		else
			nibble+=55;
		string[pozycja]=nibble;
	}
	string[pozycja]=0;
}
#endif

void alert(uint8_t DIODA)
{
	PORT &= ~(1<<DIODA);
	_delay_ms(50);
	PORT |= (1<<DIODA);
	_delay_ms(100);
	PORT &= ~(1<<DIODA);
	_delay_ms(50);
	PORT |= (1<<DIODA);	
}

void sleep(void)
{
	cli();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	power_all_disable();
	DDR=0x00;
	PORT=0x00;
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
	PORT |= (1<<YELLOW) | (1<<GREEN) | (1<<RED);
	DDR |= (1<<YELLOW) | (1<<GREEN) | (1<<RED); //wyjscie do LED
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
#ifdef UART
		pgm_xmit(mess_err3); // brak komunikacji z czujnikiem temperatury
#endif
		if(temperatura[3] != BATERIA_WYCZERPANA) //Tylko gdy bateria jeszcze sprawna, ma sens komunikowac awarie czujnika
			temperatura[3]= AWARIA_CZUJNIKA;
		return;
	}
	if(temperatura[3] != BATERIA_WYCZERPANA)
		temperatura[3]= OK;
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

static void pokazTemperature(uint8_t *temperatura, uint16_t poziom_swiatla)
{
#ifdef UART	
	pgm_xmit(mess_wynik1);
	xmit(temperatura[2]);
	xmit((temperatura[0]/10)+ASCII_ZERO);
	xmit((temperatura[0]%10)+ASCII_ZERO);
	xmit(',');
	xmit((((uint16_t)temperatura[1]*625)/1000)+ASCII_ZERO);
	pgm_xmit(mess_wynik2);
#endif
	if(temperatura[3] != OK)
	{
		if(temperatura[3] == BATERIA_WYCZERPANA)
			alert(RED);
		else
			alert(YELLOW);
		return;
	}
	if(temperatura[0] >= TEMP_MINIMALNA && temperatura[0] < TEMP_MAKSYMALNA )
	{
		//Temperatura jest optymalna, wezmiemy pod uwage poziom swiatla by nie razic zbytnio diodą w nocy :)
		//PORT &= ~(1<<GREEN);
		TCCR0A = 1<<COM0B1 | /*1<<COM0B0 | */ 1<<WGM01 | 1<<WGM00; //Fast PWM on OC0B, clear on compare - set on bottom
		OCR0B = (poziom_swiatla>>2) & 0xFE; //Obcinamy 2 najmlodsze bity wyniku by miec 8 bitow z pomiaru 
		// i trzeci zerujemy, aby zielona dioda nigdy nie byla calkiem wygaszona
		TCCR0B = (1 << CS00); // start timer, no prescale
		_delay_ms(50);
		TCCR0B=0; //Wylaczenie PWM
		TCCR0A=0;
		PORT |= (1<<YELLOW) | (1<<GREEN) | (1<<RED); //Wylaczamy wszystkie LED
		return;
		
	}
	if(temperatura[0] < TEMP_MINIMALNA)
		PORT &= ~(1<<YELLOW);
	if(temperatura[0] >= TEMP_MAKSYMALNA)
		PORT &= ~(1<<RED);
	_delay_ms(50);
	PORT |= (1<<YELLOW) | (1<<GREEN) | (1<<RED); //Wylaczamy wszystkie LED
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
uint16_t getLightLevel(void) 
{
	PORT |= (1<<PHOTOTRANSISTOR); //Zasilanie na fototranzystor przez R podciagajacy na porcie
	ADCSRA |= (1<<ADEN); //Włączenie przetwornika AD
	ADCSRA |= (1<<ADPS1) | (1<<ADPS0); //Preskaler 8 dla przetwornika AD (przy 0.5MHz clk)
	ADMUX = (1<<MUX1); // Vcc as Vref and connect ADC2(PB4)
	ADCSRA |= (1<<ADSC); // Convert
	_delay_ms(1);
	while (ADCSRA & (1<<ADSC));
	uint8_t low = ADCL;
	uint16_t val = ((ADCH&0x03) << 8) | low;
	//discard previous result
	ADCSRA |= (1<<ADSC); // Convert
	while (ADCSRA & (1<<ADSC));
	low = ADCL;
	val = ((ADCH&0x03) << 8) | low;
	_delay_ms(1);
	ADCSRA |= (1<<ADSC); // Convert
	while (ADCSRA & (1<<ADSC));
	low = ADCL;
	val += ((ADCH&0x03) << 8) | low;
	_delay_ms(1);
	ADCSRA |= (1<<ADSC); // Convert
	while (ADCSRA & (1<<ADSC));
	low = ADCL;
	val += ((ADCH&0x03) << 8) | low;
	
	ADCSRA &= ~(1<<ADEN); //Wyłączenie przetwornika AD
	PORT &= ~(1<<PHOTOTRANSISTOR); //Odlaczamy fototranzystor
	return val/3;
}
#ifdef UART
void pokazLiczbe(uint16_t liczba)
{
	xmit((liczba/1000)+ASCII_ZERO);
	xmit(((liczba%1000)/100)+ASCII_ZERO);
	xmit(((liczba%100)/10)+ASCII_ZERO);
	xmit((liczba%10)+ASCII_ZERO);
}
void pokaz_VCC(uint16_t napiecie_baterii)
{
	pgm_xmit(mess_bateria1);
	pokazLiczbe(napiecie_baterii);
	pgm_xmit(mess_bateria2);	
	
}
void pokazPoziomSwiatla(uint16_t poziom_swiatla)
{
	pgm_xmit(mess_swiatlo1);
	pokazLiczbe(poziom_swiatla);
	pgm_xmit(mess_swiatlo2);	
}
#endif

