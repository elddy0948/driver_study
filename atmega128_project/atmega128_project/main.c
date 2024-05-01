/*
 * atmega128_project.c
 *
 * Created: 2024-05-02 오전 1:25:14
 * Author : HoJoon
 */ 

#include <avr/io.h>
#include "Drivers/Inc/atmega128_driver.h"

int main(void)
{
	IO_SET_OUTPUT(DDRC, 0);
	
	IO_WRITE_PIN(PORTC, 0, HIGH);
    /* Replace with your application code */
	IO_WRITE_PIN(PORTC, 0, LOW);
    while (1) 
    {
    }
}

