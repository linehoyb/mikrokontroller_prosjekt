/*
 * ADC_modul_prosjekt.c
 *
 * Created: 21.04.2021 17:57:35
 * Author : lineh
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) // UBRR value

#define THERM_PIN 0X00 // Thermistor connected to AREF, GND and A0
#define POT_PIN 0x01 // Pot.meter connected to AREF, GND and A1

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <math.h>

uint8_t time_test = 180; // Test-variable to see if timer working

void ADC_init(void)
{
	ADMUX |= (1<<REFS0); // VCC as voltage reference
	ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); //Enables ADC feature, prescaler = 128 --> ADCfreq = 125kHz
	// (Selecting prescaler: freq. needed: 50kHz - 200kHz. CLKfreq: 16MHz)
}

void USART_init(void)
{
	UBRR0 = BAUD_PRESCALE; // Sets UBBR according to system clock and desired baudrate
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Turn on the transmission and reception circuitry
}

void timer_init(void)
{
	/*An interrupt happens every 1s*/
	OCR1A = 0x3D08;

	TCCR1B |= (1 << WGM12);	// Mode 4, CTC on OCR1A
	TIMSK1 |= (1 << OCIE1A); 	//Set interrupt on compare match
	TCCR1B |= (1 << CS12) | (1 << CS10); // set prescaler to 1024 and start the timer
	
	sei(); // enable interrupts
}

void transmitByte(uint8_t data)
{
	/* Wait for empty transmit buffer */
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = data;                                            /* send data */
}

void printString(const char myString[]) 
{
	uint8_t i = 0;
	while (myString[i]) {
		transmitByte(myString[i]);
		i++;
	}
}

uint16_t read_ADC(uint8_t ADCchannel)
{
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F); //select ADC channel with safety mask
	ADCSRA |= (1<<ADSC); //single conversion mode

	while( ADCSRA & (1<<ADSC) ); // wait until ADC conversion is complete
	return ADC;
}

void print_value(uint16_t number)
{
	/*
	* Prints 3-digit numbers to the terminal
	* Because the terminal is interpeting these values as an ASCI code we add '0', or 48, plus a digit
	*/
	transmitByte('0' + ((number/ 100) % 10)); // Hundreds
	transmitByte('0' + ((number/ 10) % 10)); // Tens
	transmitByte('0' + (number % 10)); // Ones
	printString("\r");
	_delay_ms(50);
}

float ADC_to_seconds(uint16_t adc_number)
{
	/*Returns the timer value in seconds*/
	const float MAX_SECONDS = 180;
	return ((float)adc_number/1023)*MAX_SECONDS;
}

float ADC_to_celcius(uint16_t adc_number)
{
	/*Returns the temperature in celcius*/
	const float T_25 = 298.15; // ref.temp in kelvin (25C)
	const float R_25 = 10000; // ref. resistance thermistor
	const float BETA = 3950; // material constant
	const float ADC_MAX = 1023; 
	const float VCC = 5.0;
	
	float V_0 = ((float)adc_number*VCC) / ADC_MAX; // voltage over thermistor
	float R_0 = V_0*R_25 / (VCC-V_0); // thermistor resistance
	float celcius = (1 / ((1/T_25) + (1/BETA)*log(R_0 / R_25))) - 273.15;
	return celcius;
}

int main(void)
{	
	ADC_init();
	USART_init();
	timer_init();
	
	while (1)
	{
		volatile uint16_t thermistor_value = read_ADC(THERM_PIN);
		float temperature = ADC_to_celcius(thermistor_value);
		printString("Temperature: ");
		print_value(temperature);
		
		volatile uint16_t pot_value = read_ADC(POT_PIN);
		float seconds = ADC_to_seconds(pot_value);
		printString("Set time at: ");
		print_value(seconds);
		
		printString("------\r");
		_delay_ms(1000);
	}
	//	return 0;
}

ISR (TIMER1_COMPA_vect) // action to be done every 1 sec
{
	if (time_test == 0) time_test = 180; // resets timer (just for testing)
	else time_test--; // Subtracts 1 from the timer value
	
	printString("Time left: ");
	print_value(time_test);
}