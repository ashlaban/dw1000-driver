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

/**
 * \file
 * 	Demo implementation of ranging for the Decawave dw1000 chip.
 * \author
 *	Kim Albertsson
 */

 #include "dw-range.h"

#include <stdio.h> /* For printf() */
#include <stdint.h>
#include <stddef.h>
 #include "string.h"

#include "udelay.h"

#include "dw1000-base.h"
#include "dwFrameTypes.h"

extern dw1000_base_driver dw1000;

PROCESS(dw_interrupt_callback_proc, "Interrupt callback (dw1000)");

void
printArray( uint8_t * pData, uint32_t len )
{
	if (len == 0) return;
	int i;
	for (i = 0; i<len-1; ++i)
	{
		printf("%hx, ", *pData++ );
	}
	printf("%hx\n", *pData );
}
/*---------------------------------------------------------------------------*/
void
sendBlink()
{
	// printf("Sending blink.\n");
	dw_frame_blink_t blinkFrame;
	dw_init_frame_blink( &blinkFrame );
	
	// Configure transmission
	dw1000_tx_conf_t tx_conf;
	tx_conf.data_len = DW_FRAME_BLINK_LEN;
	tx_conf.is_delayed = 0;
	dw_conf_tx( &tx_conf );

	// Transmit
	uint32_t  n_data_segments = 1;
	uint32_t  p_data_len[1] = {DW_FRAME_BLINK_LEN};
	uint8_t * pp_data[1];
	pp_data[0] = (uint8_t *)&blinkFrame;
	dw_transmit_multiple_data( pp_data, p_data_len, n_data_segments, DW_TRANCEIVE_SYNC );
	//printf("Blink sent.\n");
}
/*---------------------------------------------------------------------------*/
void
sendRangeInit( uint64_t destAddr, uint16_t delay, uint64_t dx_timestamp )
{
	//printf("Sending ranging init...\n");
	uint16_t tag_id       = ((dw_get_device_id() & 0xFF) << 8) | (destAddr & 0xFF);

	dw_frame_range_t        frameRange;
	dw_message_range_init_t msgRangeInit;
	dw_init_frame_range( &frameRange, destAddr );
	dw_init_message_range_init( &msgRangeInit, tag_id, delay );

	// Configure transmission
	dw1000_tx_conf_t tx_conf;
	tx_conf.data_len = DW_FRAME_RANGE_LEN + DW_MSG_RANGE_INIT_LEN;
	tx_conf.is_delayed   = 1;
	tx_conf.dx_timestamp = dx_timestamp;
	dw_conf_tx( &tx_conf );

	// Transmit
	uint32_t   n_data_segments = 2;
	uint32_t   data_len[2] = { DW_FRAME_RANGE_LEN, DW_MSG_RANGE_INIT_LEN };
	uint8_t  * pp_data[2];
	pp_data[0] = (uint8_t *)&frameRange;
	pp_data[1] = (uint8_t *)&msgRangeInit;
	dw_transmit_multiple_data( pp_data, data_len, n_data_segments, DW_TRANCEIVE_SYNC );

	// printArray( (uint8_t *)&frameRange  , DW_FRAME_RANGE_LEN      );
	// printArray( (uint8_t *)&msgRangeInit, DW_MSG_RANGING_INIT_LEN );
	//printf("Range init sent.\n");
}
/*---------------------------------------------------------------------------*/
void
sendPoll( uint64_t destAddr, uint64_t * pTsp )
{
	// printf("Sending poll.\n");
	dw_frame_range_t  frameRange;
	dw_message_poll_t msgPoll;
	dw_init_frame_range( &frameRange, destAddr );
	dw_init_message_poll( &msgPoll );

	// Configure
	dw1000_tx_conf_t tx_conf;
	tx_conf.data_len   = DW_FRAME_RANGE_LEN + DW_MSG_POLL_LEN;
	tx_conf.is_delayed = 0;
	dw_conf_tx( &tx_conf );
	
	// Transmit
	uint32_t   n_data_segments = 2;
	uint32_t   data_len[2] = { DW_FRAME_RANGE_LEN, DW_MSG_POLL_LEN };
	uint8_t  * pp_data[2];
	pp_data[0] = (uint8_t *)&frameRange;
	pp_data[1] = (uint8_t *)&msgPoll;
	dw_transmit_multiple_data( pp_data, data_len, n_data_segments, DW_TRANCEIVE_SYNC );
	*pTsp = dw_get_tx_timestamp();
}
/*---------------------------------------------------------------------------*/
void
sendResponse( uint64_t destAddr, uint64_t * pTsr, uint64_t dx_timestamp )
{
	// printf("Sending response...\n");
	dw_frame_range_t      frameRange;
	dw_message_response_t msgResponse;
	dw_init_frame_range( &frameRange, destAddr );
	dw_init_message_response( &msgResponse );
	*pTsr = (dx_timestamp & DX_TIMESTAMP_CLEAR_LOW_9) + dw_get_antenna_delay();

	// Configure
	dw1000_tx_conf_t tx_conf;
	tx_conf.data_len = DW_FRAME_RANGE_LEN + DW_MSG_RESPONSE_LEN;
	tx_conf.is_delayed   = 1;
	tx_conf.dx_timestamp = dx_timestamp;
	dw_conf_tx( &tx_conf );

	// Transmit
	uint32_t   n_data_segments = 2;
	uint32_t   data_len[2] = { DW_FRAME_RANGE_LEN, DW_MSG_RESPONSE_LEN };
	uint8_t  * pp_data[2];
	pp_data[0] = (uint8_t *)&frameRange;
	pp_data[1] = (uint8_t *)&msgResponse;
	dw_transmit_multiple_data( pp_data, data_len, n_data_segments, DW_TRANCEIVE_SYNC );

	// printArray( (uint8_t *)&frameRange  , DW_FRAME_RANGE_LEN );
	// printArray( (uint8_t *)&msgResponse, DW_MSG_RESPONSE_LEN );
	//printf("Response sent.\n");
}
/*---------------------------------------------------------------------------*/
void
sendFinal( uint64_t destAddr, uint64_t tsp, uint64_t trr, uint64_t * pTsf, uint64_t dx_timestamp )
{
	//printf("Sending final.\n");
	*pTsf = (dx_timestamp & DX_TIMESTAMP_CLEAR_LOW_9) + dw_get_antenna_delay();

	dw_frame_range_t  frameRange;
	dw_message_final_t msgFinal;
	dw_init_frame_range( &frameRange, destAddr );
	dw_init_message_final( &msgFinal, tsp, trr, *pTsf );

	// Configure
	dw1000_tx_conf_t tx_conf;
	tx_conf.data_len = DW_FRAME_RANGE_LEN + DW_MSG_FINAL_LEN;
	tx_conf.is_delayed   = 1;
	tx_conf.dx_timestamp = dx_timestamp;
	dw_conf_tx( &tx_conf );

	// Transmit
	uint32_t   n_data_segments = 2;
	uint32_t   data_len[2] = { DW_FRAME_RANGE_LEN, DW_MSG_FINAL_LEN };
	uint8_t  * pp_data[2];
	pp_data[0] = (uint8_t *)&frameRange;
	pp_data[1] = (uint8_t *)&msgFinal;
	dw_transmit_multiple_data( pp_data, data_len, n_data_segments, DW_TRANCEIVE_SYNC );

	//printf("Final sent.\n");
}
/*---------------------------------------------------------------------------*/
uint32_t
parseFrameBlink( uint8_t * pData, uint32_t dataLen, dw_frame_blink_t * pMsg )
{
	//printf("DataLen  : %i\n", dataLen);
	//printf("Should be: %i\n", DW_FRAME_BLINK_LEN + DW_FRAME_FCS_LEN);
	if (pData == NULL || pMsg  == NULL || dataLen != DW_FRAME_BLINK_LEN + DW_FRAME_FCS_LEN) return 1;

	pMsg->frame_control = *((uint8_t  *)(pData+0 ));
	pMsg->seq_no        = *((uint8_t  *)(pData+1 ));
	pMsg->fcs           = *((uint16_t *)(pData+10));

	// Fault is generated if commented out code is run. memcpy line is equivalent.
	//pMsg->tag_id        = *((uint64_t *)(pData+2 ));
	memcpy( &pMsg->tag_id, pData+2, 8 );

	if (pMsg->frame_control != 0xC5) { return 1; }

	return 0;
}
/*---------------------------------------------------------------------------*/
uint32_t
parseFrameRange( uint8_t * pData, uint32_t dataLen, dw_frame_range_t * pMsg )
{
	if (pData == NULL) return 1;
	if (pMsg  == NULL) return 1;

	pMsg->frame_control = *((uint16_t *)(pData+0 ));
	pMsg->seq_no        = *((uint8_t  *)(pData+2 ));
	pMsg->pan_id        = *((uint16_t *)(pData+3 ));
	pMsg->pPayload      =   (uint8_t  *)(pData+21);
	pMsg->fcs           = *((uint16_t *)(pData+dataLen-2));
	// TODO: Add user payload
	
	// Can't do unaligned memory accesses, use memcpy instead
	// pMsg->dest_addr     = *((uint64_t *)(pData+5 ));
	// pMsg->src_addr      = *((uint64_t *)(pData+13));
	memcpy( &pMsg->dest_addr, pData+5 , 8);
	memcpy( &pMsg->src_addr , pData+13, 8);

	if (pMsg->frame_control != 0xCC41) { return 1; }

	return 0;
}
/*---------------------------------------------------------------------------*/
uint32_t
parseMessagePoll( uint8_t * pData, uint32_t dataLen, dw_message_poll_t * pMsg )
{
	if (pData == NULL) return 1;
	if (pMsg  == NULL) return 1;

	pMsg->function_code     = *((uint8_t  *)(pData+0));
	// TODO: Add user payload

	if (pMsg->function_code != 0x21) { return 1; }

	return 0;
}
/*---------------------------------------------------------------------------*/
uint32_t
parseMessageResponse( uint8_t * pData, uint32_t dataLen, dw_message_response_t * pMsg )
{
	if (pData == NULL) return 1;
	if (pMsg  == NULL) return 1;

	pMsg->function_code  = *((uint8_t  *)(pData+0));
	pMsg->activity       = *((uint8_t  *)(pData+1));
	pMsg->activity_param = *((uint16_t *)(pData+2));
	// TODO: Add user payload

	if (pMsg->function_code != 0x10) { return 1; }

	return 0;
}
/*---------------------------------------------------------------------------*/
uint32_t
parseMessageFinal( uint8_t * pData, uint32_t dataLen, dw_message_final_t * pMsg )
{
	if (pData == NULL) return 1;
	if (pMsg  == NULL) return 1;

	pMsg->function_code = *((uint8_t  *)(pData+0));
	memcpy( pMsg->poll_msg_tx_timestamp , pData+1 , 5 );
	memcpy( pMsg->resp_msg_rx_timestamp , pData+6 , 5 );
	memcpy( pMsg->final_msg_tx_timestamp, pData+11, 5 );
	// TODO: Add user payload

	if (pMsg->function_code != 0x29) { return 1; }

	return 0;
}
/*---------------------------------------------------------------------------*/
uint32_t
parseMessageRangeInit( uint8_t * pData, uint32_t dataLen, dw_message_range_init_t * pMsg )
{
	if (pData == NULL) return 1;
	if (pMsg  == NULL) return 1;

	pMsg->function_code     = *((uint8_t  *)(pData+0));
	pMsg->tag_addr          = *((uint16_t *)(pData+1));
	pMsg->response_delay_ms = *((uint16_t *)(pData+3));

	if (pMsg->function_code != 0x20) { return 1; }

	return 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw_interrupt_callback_proc, ev, data)
{
	PROCESS_BEGIN();
	while( 1 )
	{
		PROCESS_WAIT_EVENT();
		//printf(" ==== HANDLING EVENT! ==== \n");
		if (ev != PROCESS_EVENT_POLL) { continue; }

		static ranging_state_t st;
		uint32_t status_reg = dw_read_reg_32( DW_REG_SYS_STATUS, 4 );
		//printf("status_reg: %x\n", status_reg);

		// TODO: This should be wrapped in a while loop so that all pending 
		// interrupts will be handled.
		// while (status_reg != 0)
		// {
		if (status_reg & DW_TXFRS_MASK)
		{
			/*
		 	 * TX - Frame Sent
		 	 */
			++packet_stats.num_packets_tx;
			dw1000.state = DW_STATE_IDLE;

			// Clear TXFRS interrupt.
			uint32_t clear_interrupt_mask = DW_TXFRS_MASK;
			dw_write_reg( DW_REG_SYS_STATUS, 4, (uint8_t *)&clear_interrupt_mask );
		} 
		else if (status_reg & DW_RXDFR_MASK)
		{
			/*
		 	 * RX - Frame Ready
		 	 */
		 	
			++packet_stats.num_packets_rx;
			dw_get_rx_buffer(); /* updates dw1000.state */
			
			// Parsing of the receive buffer to decide what packet is incomming.
			// Perhaps this is suited better outside the interrupt handle process.
			if      (dw1000.p_receive_buffer[0] == 0xC5                                       ) st = BLINK_RECEIVED;
			else if (dw1000.p_receive_buffer[0] == 0x41 && dw1000.p_receive_buffer[21] == 0x20) st = RANGE_INIT_RECEIVED ;
			else if (dw1000.p_receive_buffer[0] == 0x41 && dw1000.p_receive_buffer[21] == 0x21) st = POLL_RECEIVED ;
			else if (dw1000.p_receive_buffer[0] == 0x41 && dw1000.p_receive_buffer[21] == 0x10) st = RESPONSE_RECEIVED ;
			else if (dw1000.p_receive_buffer[0] == 0x41 && dw1000.p_receive_buffer[21] == 0x29) st = FINAL_RECEIVED;
			if (dw1000.state == DW_STATE_ERROR) st = ERROR;

			// printArray(dw1000.p_receive_buffer, dw1000.receive_buffer_len);

			//TODO: Reset RXDFR interrupt, this is done in get_rx buffer right now. Perhaps inappropriate.
			process_post(interrupt_handler_callback, PROCESS_EVENT_CONTINUE, &st);
		}
		else if (status_reg & DW_RXRFTO_MASK)
		{
			/*
		 	 * RX - Receive Frame Timeout
		 	 */
			dw1000.state = DW_STATE_ERROR;
			dw1000.error_code = DW_ERROR_TIMEOUT;
			st = ERROR;
			
			// Reset RXTFO interrupt
			uint32_t clear_interrupt_mask = DW_RXRFTO_MASK;
			dw_write_reg( DW_REG_SYS_STATUS, 4, (uint8_t *)&clear_interrupt_mask );

			// Continue work process.
			process_post(interrupt_handler_callback, PROCESS_EVENT_CONTINUE, &st);
		}
		else
		{
			/*
		 	 * All other events treated as errors.
		 	 */
		 	
			++packet_stats.num_errors;

			// Error handling
			dw1000.state = DW_STATE_ERROR;
			st = ERROR;
			
			// printf("status_reg: %x\n", status_reg);

			// Reset all events.
			const uint64_t one_mask = 0xFFFFFFFFFFFFFFFFULL;
			dw_write_reg( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&one_mask );
			process_post(interrupt_handler_callback, PROCESS_EVENT_CONTINUE, &st);
		}

		// printf("status_reg: %x\n", status_reg);
		// printf("state: %x\n", st);
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/