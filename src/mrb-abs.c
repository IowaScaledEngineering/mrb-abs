/*************************************************************************
Title:    MRBus ABS Signal Module - Based on MRB-GIO Hardware
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2016 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "mrbus.h"

uint8_t mrbus_dev_addr = 0;
uint8_t pkt_count = 0;


#define EE_E1_OCC_ADDR     0x10
#define EE_E1_APRCH_ADDR   0x11
#define EE_E1_APRCH2_ADDR  0x12
#define EE_E1_TMBL_ADDR    0x13
#define EE_W1_OCC_ADDR     0x14
#define EE_W1_APRCH_ADDR   0x15
#define EE_W1_APRCH2_ADDR  0x16
#define EE_W1_TMBL_ADDR    0x17

#define EE_E2_OCC_ADDR     0x18
#define EE_E2_APRCH_ADDR   0x19
#define EE_E2_APRCH2_ADDR  0x1A
#define EE_E2_TMBL_ADDR    0x1B
#define EE_W2_OCC_ADDR     0x1C
#define EE_W2_APRCH_ADDR   0x1D
#define EE_W2_APRCH2_ADDR  0x1E
#define EE_W2_TMBL_ADDR    0x1F

#define EE_E1_OCC_PKT      0x20
#define EE_E1_APRCH_PKT    0x21
#define EE_E1_APRCH2_PKT   0x22
#define EE_E1_TMBL_PKT     0x23
#define EE_W1_OCC_PKT      0x24
#define EE_W1_APRCH_PKT    0x25
#define EE_W1_APRCH2_PKT   0x26
#define EE_W1_TMBL_PKT     0x27

#define EE_E2_OCC_PKT      0x28
#define EE_E2_APRCH_PKT    0x29
#define EE_E2_APRCH2_PKT   0x2A
#define EE_E2_TMBL_PKT     0x2B
#define EE_W2_OCC_PKT      0x2C
#define EE_W2_APRCH_PKT    0x2D
#define EE_W2_APRCH2_PKT   0x2E
#define EE_W2_TMBL_PKT     0x2F

#define EE_E1_OCC_BITBYTE     0x30
#define EE_E1_APRCH_BITBYTE   0x31
#define EE_E1_APRCH2_BITBYTE  0x32
#define EE_E1_TMBL_BITBYTE    0x33
#define EE_W1_OCC_BITBYTE     0x34
#define EE_W1_APRCH_BITBYTE   0x35
#define EE_W1_APRCH2_BITBYTE  0x36
#define EE_W1_TMBL_BITBYTE    0x37

#define EE_E2_OCC_BITBYTE     0x38
#define EE_E2_APRCH_BITBYTE   0x39
#define EE_E2_APRCH2_BITBYTE  0x3A
#define EE_E2_TMBL_BITBYTE    0x3B
#define EE_W2_OCC_BITBYTE     0x3C
#define EE_W2_APRCH_BITBYTE   0x3D
#define EE_W2_APRCH2_BITBYTE  0x3E
#define EE_W2_TMBL_BITBYTE    0x3F

#define OCCUPANCY_E_OCC  0x01
#define OCCUPANCY_E_ADV  0x02
#define OCCUPANCY_E_ADV2 0x04
#define OCCUPANCY_E_TMBL 0x08
#define OCCUPANCY_W_OCC  0x10
#define OCCUPANCY_W_ADV  0x20
#define OCCUPANCY_W_ADV2 0x40
#define OCCUPANCY_W_TMBL 0x80


// ******** Start 100 Hz Timer - Very Accurate Version

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;

volatile uint8_t eventFlags = 0;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCCR1A = 0;
	TCCR1B = _BV(CS11) | _BV(CS10);
	TCCR1C = 0;
	TIMSK1 = _BV(TOIE1);
	ticks = 0;
	decisecs = 0;
}

#define EVENT_1HZ_BLINK  0x04

ISR(TIMER1_OVF_vect)
{
	static uint8_t hzTimer = 0;

	TCNT1 += 0xF3CB;
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}
	if (hzTimer++ >= 5)
	{
		hzTimer = 0;
		eventFlags ^= EVENT_1HZ_BLINK;
	}
	
}

// **** Bus Voltage Monitor

volatile uint8_t busVoltage=0;

ISR(ADC_vect)
{
	static uint16_t busVoltageAccum=0;
	static uint8_t busVoltageCount=0;

	busVoltageAccum += ADC;
	if (++busVoltageCount >= 64)
	{
		busVoltageAccum = busVoltageAccum / 64;
        //At this point, we're at (Vbus/6) / 5 * 1024
        //So multiply by 300, divide by 1024, or multiply by 150 and divide by 512
        busVoltage = ((uint32_t)busVoltageAccum * 150) / 512;
		busVoltageAccum = 0;
		busVoltageCount = 0;
	}
}

uint8_t occupancyStatus[2] = {0,0};

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;


	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 16;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = MRBUS_VERSION_WIRED;
		txBuffer[7]  = 0; // Software Revision
		txBuffer[8]  = 0; // Software Revision
		txBuffer[9]  = 0; // Software Revision
		txBuffer[10]  = 0; // Hardware Major Revision
		txBuffer[11]  = 0; // Hardware Minor Revision
		txBuffer[12] = 'A';
		txBuffer[13] = 'B';
		txBuffer[14] = 'S';
		txBuffer[15] = ' ';
		mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}


	for (i=0; i<(EE_E1_OCC_PKT - EE_E1_OCC_ADDR); i++)
	{
		if (rxBuffer[MRBUS_PKT_SRC] == eeprom_read_byte((uint8_t*)(i+EE_E1_OCC_ADDR)))
		{
			if (rxBuffer[MRBUS_PKT_TYPE] == eeprom_read_byte((uint8_t*)(i+EE_E1_OCC_PKT)))
			{
				uint8_t byteNum = eeprom_read_byte((uint8_t*)(i+EE_E1_OCC_BITBYTE));
				uint8_t bitNum = (byteNum>>5) & 0x07;
				byteNum &= 0x1F;
	
				if (rxBuffer[byteNum] & (1<<bitNum))
					occupancyStatus[i/8] |= 1<<(i%8);
				else
					occupancyStatus[i/8] &= ~(1<<(i%8));
			}
		}
	}

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
}

void init(void)
{
	// FIXME:  Do any initialization you need to do here.
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	

	pkt_count = 0;

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);
	
	// FIXME: This line assures that update_decisecs is 2 seconds or less
	// You probably don't want this, but it prevents new developers from wondering
	// why their new node doesn't transmit (uninitialized eeprom will make the update
	// interval 64k decisecs, or about 110 hours)  You'll probably want to make this
	// something more sane for your node type, or remove it entirely.
	update_decisecs = min(20, update_decisecs);



// Uncomment this block to set up the ADC to continuously monitor the bus voltage using a 3:1 divider tied into the ADC7 input
// You also need to uncomment the ADC ISR near the top of the file
	// Setup ADC
	ADMUX  = 0x47;  // AVCC reference; ADC7 input
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	DIDR0  = 0x00;  // No digitals were harmed in the making of this ADC

	busVoltage = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

}

#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 16

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

typedef enum
{
	ASPECT_OFF          = 0,
	ASPECT_GREEN        = 1,
	ASPECT_YELLOW       = 2,
	ASPECT_FL_YELLOW    = 3,
	ASPECT_RED          = 4,
	ASPECT_FL_GREEN     = 5,
	ASPECT_FL_RED       = 6,
	ASPECT_LUNAR        = 7
} SignalAspect;

typedef enum 
{
	INDICATION_STOP               = 0,
	INDICATION_APPROACH           = 1,
	INDICATION_APPROACH_DIVERGING = 2,
	INDICATION_ADVANCE_APPROACH   = 3,
	INDICATION_CLEAR              = 4
} CodelineStatus;

#define SIGA1_RED_PB  PB2
#define SIGA1_YEL_PB  PB1
#define SIGA1_GRN_PB  PB0

void setSignalA1(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGA1_RED_PB):0) | (yellow?_BV(SIGA1_YEL_PB):0) | (green?_BV(SIGA1_GRN_PB):0));
	if (ca)
		PORTB = (PORTB | (_BV(SIGA1_RED_PB) | _BV(SIGA1_YEL_PB) | _BV(SIGA1_GRN_PB))) & ~mask;
	else
		PORTB = (PORTB & ~(_BV(SIGA1_RED_PB) | _BV(SIGA1_YEL_PB) | _BV(SIGA1_GRN_PB))) | mask;
}

#define SIGB1_RED_PD  PD5
#define SIGB1_YEL_PD  PD4
#define SIGB1_GRN_PD  PD3

void setSignalB1(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGB1_RED_PD):0) | (yellow?_BV(SIGB1_YEL_PD):0) | (green?_BV(SIGB1_GRN_PD):0));
	if (ca)
		PORTD = (PORTD | (_BV(SIGB2_RED_PD) | _BV(SIGB1_YEL_PD) | _BV(SIGB1_GRN_PD))) & ~mask;
	else
		PORTD = (PORTD & ~(_BV(SIGB1_RED_PD) | _BV(SIGB1_YEL_PD) | _BV(SIGB1_GRN_PD))) | mask;
}

#define SIGA2_RED_PC  PC0
#define SIGA2_YEL_PD  PD7
#define SIGA2_GRN_PD  PD6

void setSignalA2(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t maskc = (red?_BV(SIGA2_RED_PC):0);
	uint8_t maskd = (yellow?_BV(SIGA2_YEL_PD):0) | (green?_BV(SIGA2_GRN_PD):0);
	
	if (ca)
	{
		PORTC = (PORTC | (_BV(SIGA2_RED_PC))) & ~maskc;
		PORTD = (PORTD | (_BV(SIGA2_YEL_PD) | _BV(SIGA2_GRN_PD))) & ~maskd;
	}
	else
	{
		PORTC = (PORTC & ~(_BV(SIGA2_RED_PC))) | maskc;
		PORTD = (PORTD & ~(_BV(SIGA2_YEL_PD) | _BV(SIGA2_GRN_PD))) | maskd;
	}
}

#define SIGB2_RED_PC  PC3
#define SIGB2_YEL_PC  PC2
#define SIGB2_GRN_PC  PC1

void setSignalB1(uint8_t red, uint8_t yellow, uint8_t green, uint8_t ca)
{
	uint8_t mask = ((red?_BV(SIGB2_RED_PD):0) | (yellow?_BV(SIGB2_YEL_PD):0) | (green?_BV(SIGB2_GRN_PD):0));
	if (ca)
		PORTC = (PORTC | (_BV(SIGB2_RED_PD) | _BV(SIGB2_YEL_PD) | _BV(SIGB2_GRN_PD))) & ~mask;
	else
		PORTC = (PORTC & ~(_BV(SIGB2_RED_PD) | _BV(SIGB2_YEL_PD) | _BV(SIGB2_GRN_PD))) | mask;
}


int main(void)
{
	SignalAspect signalAspectE[2] = { ASPECT_RED, ASPECT_RED };
	SignalAspect signalAspectW[2] = { ASPECT_RED, ASPECT_RED };	
	CodelineStatus codelineE[2] = { INDICATION_STOP, INDICATION_STOP };
	CodelineStatus codelineW[2] = { INDICATION_STOP, INDICATION_STOP };
	
	uint8_t commonAnode = 0;
	
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();

	sei();	

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
			
		// FIXME: Do any module-specific behaviours here in the loop.
		
		if (decisecs >= update_decisecs && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 9;
			txBuffer[5] = 'S';
			txBuffer[6] = 0;
			txBuffer[7] = 0;
			txBuffer[8] = busVoltage;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}	


		// Convert bits from various occupancy packets into codeline status
		// Fixme - might as well read aspects from EEPROM
		for (i=0; i<2; i++)
		{
			if (occupancyStatus[i] & (OCCUPANCY_E_OCC | OCCUPANCY_E_TMBL))
				codelineE[i] = INDICATION_STOP;
			else if (occupancyStatus[i] & OCCUPANCY_E_ADV)
				codelineE[i] = INDICATION_APPROACH;
			else if (occupancyStatus[i] & OCCUPANCY_E_ADV2)
				codelineE[i] = INDICATION_ADVANCE_APPROACH;
			else 
				codelineE[i] = INDICATION_CLEAR;

			if (occupancyStatus[i] & (OCCUPANCY_W_OCC | OCCUPANCY_W_TMBL))
				codelineW[i] = INDICATION_STOP;
			else if (occupancyStatus[i] & OCCUPANCY_W_ADV)
				codelineW[i] = INDICATION_APPROACH;
			else if (occupancyStatus[i] & OCCUPANCY_W_ADV2)
				codelineW[i] = INDICATION_ADVANCE_APPROACH;
			else 
				codelineW[i] = INDICATION_CLEAR;
		}			

		// Convert codeline status to aspects
		for (i=0; i<2; i++)
		{
			switch(codelineE[i])
			{
				case INDICATION_CLEAR:
					signalAspectE[i] = ASPECT_GREEN;
					break;

				case INDICATION_APPROACH:
					signalAspectE[i] = ASPECT_YELLOW;
					break;

				case INDICATION_ADVANCE_APPROACH:
					signalAspectE[i] = ASPECT_FL_YELLOW;
					break;

				default:
				case INDICATION_STOP:
					signalAspectE[i] = ASPECT_RED;
					break;
			}

			switch(codelineW[i])
			{
				case INDICATION_CLEAR:
					signalAspectW[i] = ASPECT_GREEN;
					break;

				case INDICATION_APPROACH:
					signalAspectW[i] = ASPECT_YELLOW;
					break;

				case INDICATION_ADVANCE_APPROACH:
					signalAspectW[i] = ASPECT_FL_YELLOW;
					break;

				default:
				case INDICATION_STOP:
					signalAspectW[i] = ASPECT_RED;
					break;
			}
		}
	
		// Convert aspect to bit output
		switch(signalAspectE[0])
		{
			case ASPECT_RED:
				setSignalA1(1,0,0, commonAnode);
				break;
			case ASPECT_YELLOW:
				setSignalA1(0,1,0, commonAnode);				
				break;
			case ASPECT_GREEN:
				setSignalA1(0,0,1, commonAnode);				
				break;
			case ASPECT_FL_RED:
				setSignalA1((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
				break;
			case ASPECT_FL_YELLOW:
				setSignalA1(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
				break;
			case ASPECT_FL_GREEN:
				setSignalA1(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
				break;
			case ASPECT_OFF:
			default:
				setSignalA1(0,0,0, commonAnode);
				break;
		}

		switch(signalAspectW[0])
		{
			case ASPECT_RED:
				setSignalB1(1,0,0, commonAnode);
				break;
			case ASPECT_YELLOW:
				setSignalB1(0,1,0, commonAnode);				
				break;
			case ASPECT_GREEN:
				setSignalB1(0,0,1, commonAnode);				
				break;
			case ASPECT_FL_RED:
				setSignalB1((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
				break;
			case ASPECT_FL_YELLOW:
				setSignalB1(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
				break;
			case ASPECT_FL_GREEN:
				setSignalB1(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
				break;
			case ASPECT_OFF:
			default:
				setSignalB1(0,0,0, commonAnode);
				break;
		}

		switch(signalAspectE[1])
		{
			case ASPECT_RED:
				setSignalA2(1,0,0, commonAnode);
				break;
			case ASPECT_YELLOW:
				setSignalA2(0,1,0, commonAnode);				
				break;
			case ASPECT_GREEN:
				setSignalA2(0,0,1, commonAnode);				
				break;
			case ASPECT_FL_RED:
				setSignalA2((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
				break;
			case ASPECT_FL_YELLOW:
				setSignalA2(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
				break;
			case ASPECT_FL_GREEN:
				setSignalA2(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
				break;
			case ASPECT_OFF:
			default:
				setSignalA2(0,0,0, commonAnode);
				break;
		}

		switch(signalAspectW[1])
		{
			case ASPECT_RED:
				setSignalB2(1,0,0, commonAnode);
				break;
			case ASPECT_YELLOW:
				setSignalB2(0,1,0, commonAnode);				
				break;
			case ASPECT_GREEN:
				setSignalB2(0,0,1, commonAnode);				
				break;
			case ASPECT_FL_RED:
				setSignalB2((eventFlags & EVENT_1HZ_BLINK),0,0, commonAnode);
				break;
			case ASPECT_FL_YELLOW:
				setSignalB2(0,(eventFlags & EVENT_1HZ_BLINK),0, commonAnode);				
				break;
			case ASPECT_FL_GREEN:
				setSignalB2(0,0,(eventFlags ^= EVENT_1HZ_BLINK), commonAnode);				
				break;
			case ASPECT_OFF:
			default:
				setSignalB2(0,0,0, commonAnode);
				break;
		}
		

		if (mrbusPktQueueDepth(&mrbusTxQueue))
		{
			uint8_t fail = mrbusTransmit();

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit to avoid hammering the bus
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			if (fail)
			{
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
			}
		}
	}
}



