/*
 * prosjekt_timer.c
 *
 * Created: 22.04.2021 14:09:31
 * Author : sande
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pinDefines.h"
#define DEBOUNCE_TIME  1000                            /* microseconds */




void Buzzer()
{
		/* Oppgave 1 */
	DDRB = (1<<DDB2);	// PB2 or OC1B output pin
	TCCR0A = (1<<COM0A0)|(1<<WGM01);	// Toggle OC1B on compare match and CTC mode  with OCR1A top (mode 4)
	TCCR0B = (1<<CS02)|(1<<CS00);	// 1024x prescaler	
	OCR1B =	13;	// Top value to give 1200hz freq
}


void Stop()
{
	//PORTB = (1<<PB0);	//PB0 Red led output pin high
	for (int i = 3; i > 0; --i)
	{
		Buzzer();
		_delay_ms(1000);
		DDRB = (0<<DDB2);
		_delay_ms(500);
	}	
}

uint8_t debounce(void) {
	if (bit_is_clear(BUTTON_PIN, BUTTON)) {      /* button is pressed now */
		_delay_us(DEBOUNCE_TIME);
		if (bit_is_clear(BUTTON_PIN, BUTTON)) {            /* still pressed */
			return (1);
		}
	}
	return 0;
}

void button_interupt()
{
	// -------- Inits --------- //
	uint8_t buttonWasPressed=0;                                 /* state */
	BUTTON_PORT |= (1 << BUTTON);     /* enable the pullup on the button */
	LED_DDR = (1 << LED0);                      /* set up LED for output */

	// ------ Event loop ------ //
	while (1) {
		if (debounce()) {                        /* debounced button press */
			if (buttonWasPressed == 0) {     /* but wasn't last time through */
				LED_PORT ^= (1 << LED0);                        /* do whatever */
				buttonWasPressed = 1;                      /* update the state */
			}
		}
		else {                                /* button is not pressed now */
			buttonWasPressed = 0;                        /* update the state */
		}

		}                                                  /* End event loop */
		return 0;                            /* This line is never reached */
}                                                 /* End event loop */	 	




int main(void)
{
	DDRB = (1<<DDB0);
	PORTB |= (1<<PORTB0);
	Stop();
	
    while (1) 
    {
	
    }
}
