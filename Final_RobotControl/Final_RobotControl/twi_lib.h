

// Modified for personal project by Keelin Wheeler in March 2017:
//  This code is a modified version of the library at http://playground.arduino.cc/ComponentLib/Ps2mouse
//  The details of this code need not be discussed or commented in much detail,
//    what the code does is provide the TWI protocol for communicating with the MPU9250 (IMU)


/*
  twi.h - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef TWI_H
#define TWI_H

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define TWI_FREQ 400000L
#define TWI_BUFFER_LENGTH 32

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

#define TW_READ   1
#define TW_WRITE  0

#define TW_START  0x08
#define TW_REP_START  0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
#define TW_MT_DATA_NACK 0x30
#define TW_MT_ARB_LOST  0x38
#define TW_MR_SLA_ACK  0x40
#define TW_MR_SLA_NACK  0x48
#define TW_MR_DATA_ACK  0x50
#define TW_MR_DATA_NACK  0x58
#define TW_SR_SLA_ACK  0x60
#define TW_SR_ARB_LOST_SLA_ACK  0x68
#define TW_SR_GCALL_ACK  0x70
#define TW_SR_ARB_LOST_GCALL_ACK  0x78
#define TW_SR_DATA_ACK  0x80
#define TW_SR_DATA_NACK  0x88
#define TW_SR_GCALL_DATA_ACK  0x90
#define TW_SR_GCALL_DATA_NACK  0x98
#define TW_SR_STOP  0xA0
#define TW_ST_SLA_ACK  0xA8
#define TW_ST_ARB_LOST_SLA_ACK  0xB0
#define TW_ST_DATA_ACK  0xB8
#define TW_ST_DATA_NACK  0xC0
#define TW_ST_LAST_DATA  0xC8
#define TW_NO_INFO  0xF8
#define TW_BUS_ERROR  0x00


void twi_init(void);
void twi_disable(void);
void twi_setAddress(uint8_t);
void twi_setFrequency(uint32_t);
uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t, uint8_t);
uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t, uint8_t);
uint8_t twi_transmit(const uint8_t*, uint8_t);
void twi_attachSlaveRxEvent( void (*)(uint8_t*, int) );
void twi_attachSlaveTxEvent( void (*)(void) );
void twi_reply(uint8_t);
void twi_stop(void);
void twi_releaseBus(void);

#endif
