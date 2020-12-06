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
#include "signal-output.h"

uint8_t mrbus_dev_addr = 0;
uint8_t pkt_count = 0;


#define EE_S1_OCC_ADDR     0x10
#define EE_S1_APRCH_ADDR   0x11
#define EE_S1_APRCH2_ADDR  0x12
#define EE_S1_TMBL_ADDR    0x13
#define EE_S2_OCC_ADDR     0x14
#define EE_S2_APRCH_ADDR   0x15
#define EE_S2_APRCH2_ADDR  0x16
#define EE_S2_TMBL_ADDR    0x17
#define EE_S3_OCC_ADDR     0x18
#define EE_S3_APRCH_ADDR   0x19
#define EE_S3_APRCH2_ADDR  0x1A
#define EE_S3_TMBL_ADDR    0x1B
#define EE_S4_OCC_ADDR     0x1C
#define EE_S4_APRCH_ADDR   0x1D
#define EE_S4_APRCH2_ADDR  0x1E
#define EE_S4_TMBL_ADDR    0x1F

#define EE_S1_OCC_PKT      0x20
#define EE_S1_APRCH_PKT    0x21
#define EE_S1_APRCH2_PKT   0x22
#define EE_S1_TMBL_PKT     0x23
#define EE_S2_OCC_PKT      0x24
#define EE_S2_APRCH_PKT    0x25
#define EE_S2_APRCH2_PKT   0x26
#define EE_S2_TMBL_PKT     0x27
#define EE_S3_OCC_PKT      0x28
#define EE_S3_APRCH_PKT    0x29
#define EE_S3_APRCH2_PKT   0x2A
#define EE_S3_TMBL_PKT     0x2B
#define EE_S4_OCC_PKT      0x2C
#define EE_S4_APRCH_PKT    0x2D
#define EE_S4_APRCH2_PKT   0x2E
#define EE_S4_TMBL_PKT     0x2F

#define EE_S1_OCC_BITBYTE     0x30
#define EE_S1_APRCH_BITBYTE   0x31
#define EE_S1_APRCH2_BITBYTE  0x32
#define EE_S1_TMBL_BITBYTE    0x33
#define EE_S2_OCC_BITBYTE     0x34
#define EE_S2_APRCH_BITBYTE   0x35
#define EE_S2_APRCH2_BITBYTE  0x36
#define EE_S2_TMBL_BITBYTE    0x37
#define EE_S3_OCC_BITBYTE     0x38
#define EE_S3_APRCH_BITBYTE   0x39
#define EE_S3_APRCH2_BITBYTE  0x3A
#define EE_S3_TMBL_BITBYTE    0x3B
#define EE_S4_OCC_BITBYTE     0x3C
#define EE_S4_APRCH_BITBYTE   0x3D
#define EE_S4_APRCH2_BITBYTE  0x3E
#define EE_S4_TMBL_BITBYTE    0x3F

#define EE_S1_CONFIG          0x40
#define EE_S2_CONFIG          0x41
#define EE_S3_CONFIG          0x42
#define EE_S4_CONFIG          0x43


#define OCCUPANCY_OCC  0x01
#define OCCUPANCY_ADV  0x02
#define OCCUPANCY_ADV2 0x04
#define OCCUPANCY_TMBL 0x08


volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;

volatile uint8_t eventFlags = 0;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

#define EVENT_1HZ_BLINK  0x04

ISR(TIMER0_COMPA_vect)
{
	static uint8_t hzTimer = 0;
	static uint8_t ticks=0;

	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}

	if (hzTimer++ >= 50)
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

uint8_t occupancyStatus[4] = {0,0,0,0};

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


	for (i=0; i<(EE_S1_OCC_PKT - EE_S1_OCC_ADDR); i++)
	{
		if (rxBuffer[MRBUS_PKT_SRC] == eeprom_read_byte((uint8_t*)(i+EE_S1_OCC_ADDR)))
		{
			if (rxBuffer[MRBUS_PKT_TYPE] == eeprom_read_byte((uint8_t*)(i+EE_S1_OCC_PKT)))
			{
				uint8_t byteNum = eeprom_read_byte((uint8_t*)(i+EE_S1_OCC_BITBYTE));
				uint8_t bitNum = (byteNum>>5) & 0x07;
				byteNum &= 0x1F;
	
				if (rxBuffer[byteNum] & (1<<bitNum))
					occupancyStatus[i/4] |= 1<<(i%4);
				else
					occupancyStatus[i/4] &= ~(1<<(i%4));
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
	MCUSR = 0;
	wdt_reset();
	wdt_enable(WDTO_1S);

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

	update_decisecs = min(20, max(1, update_decisecs));



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

#define NUM_SIGNALS  4

int main(void)
{
	SignalAspect signalAspect[NUM_SIGNALS] = { ASPECT_RED, ASPECT_RED, ASPECT_RED, ASPECT_RED };	
	CodelineStatus codeline[NUM_SIGNALS] = { INDICATION_STOP, INDICATION_STOP, INDICATION_STOP, INDICATION_STOP };
	SignalOutputFunc signalOutputFuncs[NUM_SIGNALS] = { &setSignalS1, &setSignalS2, &setSignalS3, &setSignalS4 };
	uint8_t signalOptions[NUM_SIGNALS] = {0,0,0,0};
	uint8_t i;
	
	// Application initialization
	init();

	// Initialize a 100 Hz timer.  See the definition for this function - you can
	// remove it if you don't use it.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbusInit();

	for(i=0; i<NUM_SIGNALS; i++)
		signalOptions[i] = eeprom_read_byte((uint8_t*)(i+EE_S1_CONFIG));

	initSignalOutputs();

	sei();	

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();
			
		// Convert bits from various occupancy packets into codeline status
		for (i=0; i<4; i++)
		{
			if (occupancyStatus[i] & (OCCUPANCY_OCC | OCCUPANCY_TMBL))
				codeline[i] = INDICATION_STOP;
			else if (occupancyStatus[i] & OCCUPANCY_ADV)
				codeline[i] = INDICATION_APPROACH;
			else if (occupancyStatus[i] & OCCUPANCY_ADV2)
				codeline[i] = INDICATION_ADVANCE_APPROACH;
			else 
				codeline[i] = INDICATION_CLEAR;
		}			

		// Convert codeline status to aspects
		for (i=0; i<4; i++)
		{
			switch(codeline[i])
			{
				case INDICATION_CLEAR:
					signalAspect[i] = ASPECT_GREEN;
					break;

				case INDICATION_APPROACH:
					signalAspect[i] = ASPECT_YELLOW;
					break;

				case INDICATION_ADVANCE_APPROACH:
					signalAspect[i] = ASPECT_FL_YELLOW;
					break;

				default:
				case INDICATION_STOP:
					signalAspect[i] = ASPECT_RED;
					break;
			}
		}


		for(i=0; i<4; i++)
		{
			SignalOutputFunc setSignal = signalOutputFuncs[i];			
			// Convert aspect to bit output
			switch(signalAspect[i])
			{
				case ASPECT_RED:
					setSignal(1,0,0, signalOptions[i]);
					break;
				case ASPECT_YELLOW:
					setSignal(0,1,0, signalOptions[i]);				
					break;
				case ASPECT_GREEN:
					setSignal(0,0,1, signalOptions[i]);				
					break;
				case ASPECT_FL_RED:
					setSignal((eventFlags & EVENT_1HZ_BLINK),0,0, signalOptions[i]);
					break;
				case ASPECT_FL_YELLOW:
					setSignal(0,(eventFlags & EVENT_1HZ_BLINK),0, signalOptions[i]);				
					break;
				case ASPECT_FL_GREEN:
					setSignal(0,0,(eventFlags ^= EVENT_1HZ_BLINK), signalOptions[i]);				
					break;
				case ASPECT_OFF:
				default:
					setSignal(0,0,0, signalOptions[i]);
					break;
			}
		}
		
		if (decisecs >= update_decisecs && !(mrbusPktQueueFull(&mrbusTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 11;
			txBuffer[5] = 'S';
			txBuffer[6] = signalAspect[0];
			txBuffer[7] = signalAspect[1];
			txBuffer[8] = signalAspect[2];
			txBuffer[9] = signalAspect[3];
			txBuffer[10] = busVoltage;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
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



