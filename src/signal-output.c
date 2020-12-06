#include <stdlib.h>
#include <avr/io.h>
#include "signal-output.h"

#define SIGS1_RED_PB  PB2
#define SIGS1_YEL_PB  PB1
#define SIGS1_GRN_PB  PB0

#define SIGS2_RED_PD  PD5
#define SIGS2_YEL_PD  PD4
#define SIGS2_GRN_PD  PD3

#define SIGS3_RED_PC  PC0
#define SIGS3_YEL_PD  PD7
#define SIGS3_GRN_PD  PD6


#define SIGS4_RED_PC  PC3
#define SIGS4_YEL_PC  PC2
#define SIGS4_GRN_PC  PC1


void setSignalS1(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGS1_RED_PB):0) | (yellow?_BV(SIGS1_YEL_PB):0) | (green?_BV(SIGS1_GRN_PB):0));
	if (ca)
		PORTB = (PORTB | (_BV(SIGS1_RED_PB) | _BV(SIGS1_YEL_PB) | _BV(SIGS1_GRN_PB))) & ~mask;
	else
		PORTB = (PORTB & ~(_BV(SIGS1_RED_PB) | _BV(SIGS1_YEL_PB) | _BV(SIGS1_GRN_PB))) | mask;
}


void setSignalS2(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGS2_RED_PD):0) | (yellow?_BV(SIGS2_YEL_PD):0) | (green?_BV(SIGS2_GRN_PD):0));
	if (ca)
		PORTD = (PORTD | (_BV(SIGS2_RED_PD) | _BV(SIGS2_YEL_PD) | _BV(SIGS2_GRN_PD))) & ~mask;
	else
		PORTD = (PORTD & ~(_BV(SIGS2_RED_PD) | _BV(SIGS2_YEL_PD) | _BV(SIGS2_GRN_PD))) | mask;
}


void setSignalS3(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t maskc = (red?_BV(SIGS3_RED_PC):0);
	uint8_t maskd = (yellow?_BV(SIGS3_YEL_PD):0) | (green?_BV(SIGS3_GRN_PD):0);
	
	if (ca)
	{
		PORTC = (PORTC | (_BV(SIGS3_RED_PC))) & ~maskc;
		PORTD = (PORTD | (_BV(SIGS3_YEL_PD) | _BV(SIGS3_GRN_PD))) & ~maskd;
	}
	else
	{
		PORTC = (PORTC & ~(_BV(SIGS3_RED_PC))) | maskc;
		PORTD = (PORTD & ~(_BV(SIGS3_YEL_PD) | _BV(SIGS3_GRN_PD))) | maskd;
	}
}


void setSignalS4(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGS4_RED_PC):0) | (yellow?_BV(SIGS4_YEL_PC):0) | (green?_BV(SIGS4_GRN_PC):0));
	if (ca)
		PORTC = (PORTC | (_BV(SIGS4_RED_PC) | _BV(SIGS4_YEL_PC) | _BV(SIGS4_GRN_PC))) & ~mask;
	else
		PORTC = (PORTC & ~(_BV(SIGS4_RED_PC) | _BV(SIGS4_YEL_PC) | _BV(SIGS4_GRN_PC))) | mask;
}

void initSignalOutputs()
{

	DDRC |= _BV(SIGS4_GRN_PC) | _BV(SIGS4_YEL_PC) | _BV(SIGS4_RED_PC) | _BV(SIGS3_RED_PC);
	DDRD |= _BV(SIGS3_YEL_PD) | _BV(SIGS3_GRN_PD) | _BV(SIGS2_RED_PD) | _BV(SIGS2_YEL_PD) | _BV(SIGS2_GRN_PD);
	DDRB |= _BV(SIGS1_GRN_PB) | _BV(SIGS1_YEL_PB) | _BV(SIGS1_RED_PB);
}
