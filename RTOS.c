/*
 * RTOS.c
 *
 * Created: 16/07/2013 2:50:53 PM
 *  Author: Rob & Nick
 */ 

#define F_CPU 11059200
#include "os.h"
#include <avr/io.h>
#include <util/delay.h>

void blinkLEDs(void)
{
	for (;;)
	{
		_delay_ms(500);
		PORTB = ~PORTB;
		Task_Next();
	}
}

extern void r_main(void)
{
	Task_Create_Period(blinkLEDs, 0, 200, 200, 1);
}