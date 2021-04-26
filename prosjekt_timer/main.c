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



void Buzzer()
{
		/* Oppgave 1 */
	DDRB = (1<<DDB1);	// PB1 or OC1A output pin
	TCCR1A = (1<<COM1A0);	// Toggle OC1A on compare match
	TCCR1B = (1<<CS12)|(1<<CS10)|(1<<WGM12);	// 1024x prescaler and CTC mode with OCR1A top (mode 4)
	/*
	(16000000/1024)*(500/1000)= 7813
	*/
	OCR1A =	13;	// Top value to give 500ms period
}

void Stop()
{
	//PORTB = (1<<PB0);	//PB0 Red led output pin high
	for (int i = 3; i > 0; --i)
	{
		Buzzer();
		_delay_ms(1000);
		DDRB = (0<<DDB1);
		_delay_ms(500);
	}	
}


int main(void)
{
	DDRB = (1<<DDB0);
	PORTB |= (1<<PORTB0);
	Stop();
	
    while (1) 
    {
	
    }
}
