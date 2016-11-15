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

#include "dwFrameTypes.h"

#include "dw1000-base.h"
#include "string.h" // memcpy()


/**
 * \brief Takes a frame argument and fills it with default data.
 */
void
dw_init_frame_blink( dw_frame_blink_t * blinkFrame )
{
	blinkFrame->frame_control = 0xC5;
	blinkFrame->seq_no        = dw_get_seq_no();
	blinkFrame->tag_id        = dw_get_device_id();
}

/**
 * \brief Takes a frame argument and fills it with default data.
 */
void
dw_init_frame_range( dw_frame_range_t * rangeFrame, uint64_t destAddr )
{
	rangeFrame->frame_control = 0xCC41;
	rangeFrame->seq_no        = dw_get_seq_no();
	rangeFrame->pan_id        = 0xDECA;
	rangeFrame->dest_addr     = destAddr;
	rangeFrame->src_addr      = dw_get_device_id();
}

/**
 * \brief Takes a frame argument and fills it with default data.
 */
void
dw_init_message_poll( dw_message_poll_t * pMsg )
{
	pMsg->function_code = 0x21;
}

/**
 * \brief Takes a frame argument and fills it with default data.
 */
void
dw_init_message_response( dw_message_response_t * pMsg )
{
	uint8_t activity       = 0x2; // Continue with ranging

	uint8_t activity_param = 0x0000;	// 0x0000 -> will NOT send request report
										// 0x0001 -> will send request report
	
	pMsg->function_code  = 0x10;
	pMsg->activity       = activity 	  & 0x00FF;
	pMsg->activity_param = activity_param & 0xFFFF;
}

/**
 * \brief Takes a frame argument and fills it with default data.
 */
void
dw_init_message_final( 	dw_message_final_t * pMsg,
							uint64_t poll_tx,
							uint64_t resp_rx,
							uint64_t final_tx
						)
{
	pMsg->function_code = 0x29;
	// TODO: Does not work for other endiannessesses
	memcpy( pMsg->poll_msg_tx_timestamp , &poll_tx , 5 );
	memcpy( pMsg->resp_msg_rx_timestamp , &resp_rx , 5 );
	memcpy( pMsg->final_msg_tx_timestamp, &final_tx, 5 );
}

/**
 * \brief Takes a frame argument and fills it with default data.
 */
void
dw_init_message_report( dw_message_report_t * pMsg, uint64_t tof )
{
	pMsg->function_code = 0x2A;
	memcpy( pMsg->tof, &tof, 6 );
}

/**
 * \brief Takes a frame argument and fills it with default data.
 */
void
dw_init_message_range_init( dw_message_range_init_t * pMsg, uint16_t tagAddr, uint16_t respDelay )
{
	pMsg->function_code     = 0x20;
	pMsg->tag_addr          = tagAddr;
	pMsg->response_delay_ms = respDelay;
}