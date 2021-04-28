/*
 * prosjektfil_mikro.c
 *
 * Created: 21.04.2021 11:38:20
 * Author : lineh
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) // UBRR value

// -------- Pin defines --------- //
#define THERM_PIN 0x00 // Thermistor connected to AREF, GND and A0
#define POT_PIN 0x01 // Pot.meter connected to AREF, GND and A1

#define START_PIN PB3 // Start button PB3
#define PAUSE_PIN PB4 // Pause button PB4
#define START_PORT PORTB
#define PAUSE_PORT PORTB

#define RED_LED 0x02 // Red led on PD2
#define YELLOW_LED 0x03 // Yellow led on PD3
#define GREEN_LED 0x00 // Green led on PB0

#define LCD_Port PORTD			//Define LCD Port
#define LCD_DPin  DDRD			//Define 4-Bit Pins (PD4-PD7 at PORT D)
#define RSPIN PD0			//RS Pin
#define ENPIN PD1 			//E Pin


// -------- Libraries --------- //
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <math.h>

// -------- Inits --------- //
#define DEBOUNCE_TIME  1000                            /* microseconds */
uint8_t time_test = 180;			// Test-variable to see if timer working
uint8_t buttonWasPressed=0;                                 /* state */
int runtime;			 //Timer for LCD

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

void init_buzzer (void)
{
	DDRB = (1<<DDB2);	// PB2 or OC1B output pin
	TCCR0A = (1<<COM0A0)|(1<<WGM01);	// Toggle OC1B on compare match and CTC mode  with OCR1A top (mode 4)
	TCCR0B = (1<<CS02)|(1<<CS00);	// 1024x prescaler
	OCR1B =	13;	// Top value to give 1200hz freq
}

void LCD_Init (void)
{
	_delay_ms(15);		//Wait before LCD activation
	LCD_Action(0x02);	//4-Bit Control
	LCD_Action(0x28);       //Control Matrix @ 4-Bit
	LCD_Action(0x0c);       //Disable Cursor
	LCD_Action(0x06);       //Move Cursor
	LCD_Action(0x01);       //Clean LCD
	_delay_ms(2);
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

void buzzer()
{
	for (int i = 3; i > 0; --i)
	{
		DDRB = (1<<DDB2);
		_delay_ms(1000);
		DDRB = (0<<DDB2);
		_delay_ms(500);
	}
}

uint8_t debounce(uint8_t button_pin) {
	if (bit_is_clear(PINB, button_pin)) {      /* button is pressed now */
		_delay_us(DEBOUNCE_TIME);
		if (bit_is_clear(PINB, button_pin)) {            /* still pressed */
			return (1);
		}
	}
	return 0;
}

void button_interupt()
{
	if (debounce(START_PORT)) {                        /* debounced button press */
		if (buttonWasPressed == 0) {     /* but wasn't last time through */
				PINB |= (1 << PINB0);                        /* do whatever */
				buttonWasPressed = 1;                      /* update the state */
		}
	}
	else {                                /* button is not pressed now */
		buttonWasPressed = 0;                        /* update the state */
	}

}                                                  /* End event loop */

void LCD_Action( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0);
	LCD_Port &= ~ (1<<RSPIN);
	LCD_Port |= (1<<ENPIN);
	_delay_us(1);
	LCD_Port &= ~ (1<<ENPIN);
	_delay_us(200);
	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);
	LCD_Port |= (1<<ENPIN);
	_delay_us(1);
	LCD_Port &= ~ (1<<ENPIN);
	_delay_ms(2);
}

void LCD_Clear()
{
	LCD_Action (0x01);		//Clear LCD
	_delay_ms(2);			//Wait to clean LCD
	LCD_Action (0x80);		//Move to Position Line 1, Position 1
}

void LCD_Print (char *str)
{
	int i;
	for(i=0; str[i]!=0; i++)
	{
		LCD_Port = (LCD_Port & 0x0F) | (str[i] & 0xF0);
		LCD_Port |= (1<<RSPIN);
		LCD_Port|= (1<<ENPIN);
		_delay_us(1);
		LCD_Port &= ~ (1<<ENPIN);
		_delay_us(200);
		LCD_Port = (LCD_Port & 0x0F) | (str[i] << 4);
		LCD_Port |= (1<<ENPIN);
		_delay_us(1);
		LCD_Port &= ~ (1<<ENPIN);
		_delay_ms(2);
	}
}


void LCD_Printpos (char row, char pos, char *str) //Write on a specific location
{
	if (row == 0 && pos<16)
	LCD_Action((pos & 0x0F)|0x80);
	else if (row == 1 && pos<16)
	LCD_Action((pos & 0x0F)|0xC0);
	LCD_Print(str);
}

int main(void)
{	
	ADC_init();
	USART_init();
	timer_init();
	
	PORTB |= (1<<PAUSE_PORT) | (1<<START_PORT); // Internal pull-up for buttons
	DDRD |= 0xFF; // LDC and LED outputs
	DDRB |= (1<<DDB0); // green LED
	
	PORTD |= (1<<PORTD2) | (1<<PORTD3);
	PORTB |= (1<<PORTB0);
	
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
		
		button_interupt();
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