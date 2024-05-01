/*
 * atmega128_io.h
 *
 * Created: 2024-05-02 오전 1:52:22
 *  Author: HoJoon
 */ 


#ifndef ATMEGA128_IO_H_
#define ATMEGA128_IO_H_

#include "atmega128_driver.h"

#define HIGH	1
#define LOW		0

#define OUTPUT	1
#define INPUT	0

#define IO_SET_OUTPUT(DDR, PIN)		(DDR |= (1 << PIN))
#define IO_SET_INPUT(DDR, PIN)			(DDR &= ~(1 << PIN))

#define IO_WRITE_PIN(PORT, PIN, DATA)	(PORT |= (DATA << PIN))
#define IO_WRITE_PORT(PORT, DATA)		(PORT = DATA)

#endif /* ATMEGA128_IO_H_ */