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


void init_buzzer (void)
{
	DDRB = (1<<DDB3);	// PB3 or OCR2A output pin
	TCCR2A = (1<<COM2A0)|(1<<WGM21);	// Toggle OC1B on compare match and CTC mode  with OCR1A top (mode 4)
	TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);	// 1024x prescaler
	OCR2A =	13;	// Top value to give 1200hz freq
}

void Buzzer()
{	
	for (int i = 3; i > 0; --i)
	{
		DDRB = (1<<DDB3);
		_delay_ms(1000);
		DDRB = (0<<DDB3);
		_delay_ms(500);
		}
}




int main(void)
{
	init_buzzer();
	Buzzer();
	
    while (1) 
    {
	
    }
}
