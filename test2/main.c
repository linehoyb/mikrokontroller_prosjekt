/*
4-Bit 16x2 LCD Example
More information at www.aeq-web.com

Forfatter:
Ida Frøhoel
Line Høybakk
Sander Bjarke Ringheim
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
#define PAUSE_PORT PORTB
#define BUZZER_PORT PORTB

#define RED_LED PD2 // Red led on PD2
#define YELLOW_LED PD3 // Yellow led on PD3
#define GREEN_LED PB0 // Green led on PB0
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
uint8_t seconds;
uint8_t temperature;
volatile uint16_t thermistor_value;

void ADC_init(void)
{
	ADMUX |= (1<<REFS0); // VCC as voltage reference
	ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); //Enables ADC feature, prescaler = 128 --> ADCfreq = 125kHz
	// (Selecting prescaler: freq. needed: 50kHz - 200kHz. CLKfreq: 16MHz)
}

void timer_init(void)
{
	/*An interrupt happens every 1s*/
	OCR1A = 0x3D08;

	TCCR1B |= (1 << WGM12);	// Mode 4, CTC on OCR1A
	TIMSK1 |= (1 << OCIE1A); 	//Set interrupt on compare match
	TCCR1B |= (1 << CS12) | (1 << CS10); // set prescaler to 1024 and start the timer
}

void button_init(void)
{
	PORTC |= (1<<PORTC2); // Internal pull-up for pause button
	PORTB |= (1<<PORTB2); // Internal pull-up for start button
	PCMSK1 |= (1<<PCINT10); // A2 set as input for interrupt
	PCICR |= (1<<PCIE1);  // PCI1 vector,  interrupt 1 enabled
	EICRA |= (1<<ISC11); //falling edge INT1 generates an interrupt request
}

void buzzer_init (void)
{
	TCCR2A = (1<<COM2A0)|(1<<WGM21);    // Toggle OC2A on compare match and CTC mode  with OCR2A top (mode 4)
	TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);    // 1024x prescaler
	OCR2A =    13;    // Top value to give 1200hz freq
}

void LCD_Init (void)
{
	LCD_DPin = 0xFF;		//Control LCD Pins (D4-D7)
	_delay_ms(15);		//Wait before LCD activation
	LCD_Action(0x02);	//4-Bit Control
	LCD_Action(0x28);       //Control Matrix @ 4-Bit
	LCD_Action(0x0c);       //Disable Cursor
	LCD_Action(0x06);       //Move Cursor
	LCD_Action(0x01);       //Clean LCD
	_delay_ms(2);
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

void LCD_Printpos (char row, char pos, char *str)
{ //Write on a specific location
	if (row == 0 && pos<16)
	LCD_Action((pos & 0x0F)|0x80);
	else if (row == 1 && pos<16)
	LCD_Action((pos & 0x0F)|0xC0);
	LCD_Print(str);
}

uint16_t read_ADC(uint8_t ADCchannel)
{
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F); //select ADC channel with safety mask
	ADCSRA |= (1<<ADSC); //single conversion mode

	while( ADCSRA & (1<<ADSC) ); // wait until ADC conversion is complete
	return ADC;
}

uint8_t ADC_to_seconds(uint16_t adc_number)
{
	/*Returns the timer value in seconds*/
	const float MAX_SECONDS = 180;
	float decimal_result = ((float)adc_number/1023)*MAX_SECONDS;
	return (uint8_t)decimal_result;
}

uint8_t ADC_to_celcius(uint16_t adc_number)
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
	return (uint8_t)celcius;
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


int main()
{	//initializing first
	ADC_init();
	timer_init();
	button_init();
	LCD_Init(); //Activate LCD
	buzzer_init();
	DDRB |= (1<<GREEN_LED);
	
	/*Start sequence. Shows "set potmeter" on LCD and has a countdown with the three leds*/
	
	LCD_Print("Set potmeter");	//Begin writing at Line 1, Position 1
	_delay_ms(1500);
	
	green_LED_on();
	_delay_ms(1000);
	yellow_LED_on();
	_delay_ms(1000);
	red_LED_on();
	_delay_ms(1000);
	LCD_Clear();

	while(1) 
	{
		while (timer_running == 0 && start_pressed == 0)
		{
			cli(); // Disable interrupts
			
			/* starts ADC_convertion of potmeter and tempsensor */
			volatile uint16_t pot_value = read_ADC(POT_PIN);
			seconds = ADC_to_seconds(pot_value);
			
			thermistor_value = read_ADC(THERM_PIN);
			temperature = ADC_to_celcius(thermistor_value);
			
			_delay_ms(200);
			
			/* Print secons ans temperature*/
			char sec [8]; //makes the seconds into char
			itoa(seconds, sec, 10); //makes it possible to print on the LCD
			LCD_Action(0x80); //sets the cursor on the LCD to row 1, place 1
			LCD_Print(sec); //print seconds
			_delay_ms(200);
			LCD_Clear(); //clears LCD
			
			char temp [8]; //makes temperature into char
			itoa(temperature, temp, 10); //makes it possible to print on LCD
			LCD_Action(0xC0);	//sets cursor on LCD to row 2, position 1 
			LCD_Print("Temperature:"); //prints the "temperature" on the position set by action 
			LCD_Printpos(2 , 14, temp); //prints the temperature on position 2, 14. 
			
			start_pressed = get_button_status(START_PIN);
// 			
			if (start_pressed)
			{
				timer_running = 1;
				green_LED_on();
				sei();	// enable interrupts, also starts the countdown
			}
		} //end while
		/* Reads the termistor value and converts it to temperature*/
		thermistor_value = read_ADC(THERM_PIN);
		temperature = ADC_to_celcius(thermistor_value);
		_delay_ms(500);
		
	} //end while(1)
} //end int main

ISR (TIMER1_COMPA_vect) // action to be done every 1 sec
{
	seconds--; // Subtracts 1 from the timer value
	if (seconds == 0) //finished sequence
	{	LCD_Clear();
		red_LED_on();
		LCD_Print("Finished");
		buzzer();
		_delay_ms(3000);
		LCD_Clear();
		timer_running = 0;
		start_pressed = 0;
		
	}
	else
	{   /* When the timer counts down ,the LCD prints the countdown of seconds and the temperature */
		LCD_Clear();
		char sec [8];
		itoa(seconds, sec, 10);
//		LCD_Print("Time left");
		LCD_Action(0x80); //sets the cursor to 1, 1
		LCD_Print(sec); //prints seconds
		_delay_ms(100);
		
		char temp [8];
		itoa(temperature, temp, 10);
		LCD_Action(0xC0);	//sets the cursor on the LCD to 2, 1 
		LCD_Print("Temperatur:");
		LCD_Printpos(2 , 14, temp); //printstemperature on position 2, 14
		
	}
}


ISR (PCINT1_vect)
{	/* Pause sequence with interrupt. prints "pause"*/
	yellow_LED_on();
	LCD_Clear();
	LCD_Print("Pause!");
	while(get_button_status(START_PIN) == 0) {} // wait for start button to be pressed
	green_LED_on();
}