/*
 * dw1000, driver for decawave dw1000 UVB chip
 *  Copyright (C) 2014  Kim Albertsson
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser Public License for more details.
 *  
 *  You should have received a copy of the GNU Lesser Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DW_RANGE_H
#define DW_RANGE_H

#include <stdint.h>
#include "contiki.h"
#include "dwFrameTypes.h"

typedef enum
{
	IDLE = 0,
	RECEIVING,
	BLINK_RECEIVED,
	RANGE_INIT_RECEIVED,
	POLL_RECEIVED,
	RESPONSE_RECEIVED,
	FINAL_RECEIVED,
	MEASUREMENT_ADDED,
	ERROR
} ranging_state_t;

typedef struct
{
	uint32_t num_packets_rx;
	uint32_t num_packets_tx;
	uint32_t num_errors;
} packet_stats_t;

packet_stats_t packet_stats;

// Utility
void printArray( uint8_t * pData, uint32_t len );

// Send
void sendBlink();
void sendRangeInit( uint64_t destAddr, uint16_t delay, uint64_t dx_timestamp );
void sendPoll( uint64_t destAddr, uint64_t * pTsp );
void sendResponse( uint64_t destAddr, uint64_t * pTsr, uint64_t dx_timestamp );
void sendFinal( uint64_t destAddr, uint64_t tsp, uint64_t trr, uint64_t * pTsf, uint64_t dx_timestamp );

// Receive
uint32_t receiveBlink( uint64_t * pDestAddr );
uint32_t receiveRangeInit( uint64_t * pDestAddr, uint16_t * pResponseDelay );
uint32_t receivePoll( uint64_t * pTrp );
uint32_t receiveResponse( uint64_t * pTrr );
uint32_t receiveFinal( 	uint64_t * pTsp, uint64_t * pTrr, uint64_t * pTsf, uint64_t * pTrf );

// Frames
uint32_t parseFrameBlink( uint8_t * pData, uint32_t dataLen, dw_frame_blink_t * pMsg );
uint32_t parseFrameRange( uint8_t * pData, uint32_t dataLen, dw_frame_range_t * pMsg );

// Messages
uint32_t parseMessagePoll(      uint8_t * pData, uint32_t dataLen, dw_message_poll_t         * pMsg );
uint32_t parseMessageResponse(  uint8_t * pData, uint32_t dataLen, dw_message_response_t     * pMsg );
uint32_t parseMessageFinal(     uint8_t * pData, uint32_t dataLen, dw_message_final_t        * pMsg );
uint32_t parseMessageRangeInit( uint8_t * pData, uint32_t dataLen, dw_message_range_init_t * pMsg );

// Interrupt handler
struct process * interrupt_handler_callback;
struct process dw_interrupt_callback_proc;

#endif