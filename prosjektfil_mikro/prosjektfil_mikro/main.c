/*
 * prosjektfil_mikro.c
 * This program sets a timer up to 3 minutes, and counts down. 
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) // UBRR value

// -------- Pin defines --------- //
#define THERM_PIN 0x00 // Thermistor connected to AREF, GND and A0
#define POT_PIN 0x01 // Pot.meter connected to AREF, GND and A1

#define BUZZER_PIN PB3 // Buzzer PB3
#define START_PIN PB2 // Start button PB2
#define PAUSE_PIN PC2 // Pause button PB4, interrupt: PCINT4
#define START_PORT PORTB
#define PAUSE_PORT PORTC
#define BUZZER_PORT PORTB


#define R_Y_LED_PORT PORTD
#define G_LED_PORT PORTB

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
uint8_t buttonWasPressed = 0;                                 /* state */
uint8_t timer_running = 0;
uint8_t start_pressed = 0;
uint8_t seconds = 0;
int runtime;			 //Timer for LCD

const uint8_t RED_LED = 2; // Red led on PD2
const uint8_t YELLOW_LED = 3; // Yellow led on PD3
const uint8_t GREEN_LED = 0; // Green led on PB0

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
	//OCR1B = 13; //buzzer test

	TCCR1B |= (1 << WGM12);	// Mode 4, CTC on OCR1A
	TIMSK1 |= (1 << OCIE1A); 	//Set interrupt on compare match
	TCCR1B |= (1 << CS12) | (1 << CS10); // set prescaler to 1024 and start the timer
}


void buzzer_init (void)
{
	TCCR2A = (1<<COM2A0)|(1<<WGM21);    // Toggle OC1B on compare match and CTC mode  with OCR1A top (mode 4)
	TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);    // 1024x prescaler
	OCR2A =    13;    // Top value to give 1200hz freq
}

// void button_init(void){ //orginal button
// 	PORTB |= (1<<PORTB2) | (1<<PORTB4); // Internal pull-up for buttons
// 	PCMSK0 |= (1<<PCINT4); // PB4 set as input for interrupt
// 	PCICR |= (1<<PCIE0);  // PCI0 vector,  interrupt 0 enabled 
// 	EICRA |= (1<<ISC01); //falling edge INT0 generates an interrupt request
// }

void button_init(void)
{
	PORTC |= (1<<PORTC2); // Internal pull-up for pause button
	PORTB |= (1<<PORTB2); // Internal pull-up for start button
	PCMSK1 |= (1<<PCINT10); // A2 set as input for interrupt
	PCICR |= (1<<PCIE1);  // PCI1 vector,  interrupt 1 enabled
	EICRA |= (1<<ISC11); //falling edge INT1 generates an interrupt request
	sei();
}

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

void LCD_init (void)
{
	_delay_ms(15);		//Wait before LCD activation
	LCD_Action(0x02);	//4-Bit Control
	LCD_Action(0x28);       //Control Matrix @ 4-Bit
	LCD_Action(0x0c);       //Disable Cursor
	LCD_Action(0x06);       //Move Cursor
	LCD_Action(0x01);       //Clean LCD
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

void transmitByte(uint8_t data)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);  //Wait for empty transmit buffer 
	UDR0 = data; // Send data
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
	uint8_t hundreds = (number/ 100) % 10;
	uint8_t tens = (number/ 10) % 10;
	uint8_t ones = number % 10;
	
	if(hundreds > 0) transmitByte('0' + hundreds);
	if(hundreds > 0 || tens > 0) transmitByte('0' + tens);
	transmitByte('0' + ones); // Ones
	printString("\r");
	_delay_ms(10);
}

uint8_t ADC_to_seconds(uint16_t adc_number)
{
	/*Returns the timer value in seconds*/
	const float MAX_SECONDS = 180;
	float decimal_result = ((float)adc_number/1023)*MAX_SECONDS;
	return (uint8_t)decimal_result;
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
		DDRB = (1<<DDB3);  // PB3 or OCR2A output pin
		_delay_ms(1000);
		DDRB = (0<<DDB3);
		_delay_ms(500);
	}
}

uint8_t debounce(uint8_t button_pin) {
	if (bit_is_clear(PINB, button_pin)) {      /* button is pressed now */
		_delay_us(DEBOUNCE_TIME);
		if (bit_is_clear(PINB, button_pin)) {            /* still pressed */
			return 1;
		}
	}
	return 0;
}

uint8_t get_button_status(uint8_t button)
{
	if (debounce(button)) {                        /* debounced button press */
		if (buttonWasPressed == 0) {     /* but wasn't last time through */
			buttonWasPressed = 1;                      /* update the state */
			return 1;
		}
	}
	else {                                /* button is not pressed now */
		buttonWasPressed = 0;                        /* update the state */
		return 0;
	}
}  
void yellow_LED_on(){
	R_Y_LED_PORT |= (1<<YELLOW_LED);
	R_Y_LED_PORT &= ~(1<<RED_LED);
	G_LED_PORT &= ~(1<<GREEN_LED);
}

void green_LED_on(){
	G_LED_PORT |= (1<<GREEN_LED);
	R_Y_LED_PORT &= ~(1<<RED_LED);
	R_Y_LED_PORT &= ~(1<<YELLOW_LED);
}

void red_LED_on(){
	R_Y_LED_PORT |= (1<<RED_LED);
	R_Y_LED_PORT &= ~(1<<YELLOW_LED);
	G_LED_PORT &= ~(1<<GREEN_LED);
}


int main(void)
{	
	ADC_init();
	USART_init();
	timer_init();
	buzzer_init();
	LCD_init();
	button_init();
	
	LCD_Print("Hello");
	_delay_ms(2000);
	
	DDRD |= 0xFF; // LDC and LED outputs
	DDRB |= (1<<DDB0); // green LED output
	green_LED_on();
	_delay_ms(500);
	yellow_LED_on();
	_delay_ms(500);
	red_LED_on();
	_delay_ms(500);
		
	while (1)
	{
		while (timer_running == 0 && start_pressed == 0)
		{
			cli(); // Disable interrupts
			volatile uint16_t pot_value = read_ADC(POT_PIN);
			seconds = ADC_to_seconds(pot_value);
			printString("Set time at: ");
			print_value(seconds);
			
			start_pressed = get_button_status(START_PIN);
			
			if (start_pressed)
			{
				timer_running = 1;
				green_LED_on();
				sei();	// enable interrupts, also starts the countdown
			}
		}
		
// 		volatile uint16_t thermistor_value = read_ADC(THERM_PIN);
// 		float temperature = ADC_to_celcius(thermistor_value);
// 		printString("Temperature: ");
// 		print_value(temperature);
// 		printString("------\r");
// 		_delay_ms(1000);
		
	}
	return 0;
}

ISR (TIMER1_COMPA_vect) // action to be done every 1 sec
{
	 seconds--; // Subtracts 1 from the timer value
	if (seconds == 0) 
	{
		printString("All done!");
		red_LED_on();
		buzzer();
		timer_running = 0;
		start_pressed = 0;
	}
	else
	{
		printString("Time left: ");
		print_value(seconds);
	}
}

ISR (PCINT1_vect)
{
	printString("PAUSE!");
	yellow_LED_on();
	while(get_button_status(START_PIN) == 0) {} // wait for start button to be pressed
	green_LED_on();
}