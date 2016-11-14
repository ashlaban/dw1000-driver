/**
* \file
 * 	Decawace uwb demo application
 * \author
 *	Kim Albertsson
 */

// Tested with mulle board 145

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include <stdint.h>
#include <stddef.h>

#include "udelay.h"

#include "dw1000-base.h"
#include "dwFrameTypes.h"
#include "dw-range.h"

#define DEVICE_TIME_UNITS (1.0/499.2e6/128.0)
#define ANTENNA_DELAY ((515.0 / 2.0) * 1e-9 / (1.0/499.2e6/128.0))

typedef enum
{
	UNPAIRED = 0,
	PAIRED
} tag_state_t;

extern struct process * interrupt_handler_callback;

int error;

/*---------------------------------------------------------------------------*/
PROCESS(dw_main_process           , "Main process");
PROCESS(dw_tag_process            , "Tag process (dw1000)");
AUTOSTART_PROCESSES(&dw_main_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw_main_process, ev, data)
{
	PROCESS_BEGIN();

	// Init dw1000
	process_start(&dw_interrupt_callback_proc, NULL);

	dw_init();
	dw_conf_print();
	interrupt_handler_callback = &dw_tag_process;
	process_start(&dw_tag_process, NULL);

	// Start code proper
	while(1)
	{
		PROCESS_WAIT_EVENT();
	}
 
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw_tag_process, ev, data)
{
	PROCESS_BEGIN();

	static uint64_t dest_addr = 0ULL;
	static uint16_t responseDelay;

	static tag_state_t     tag_state     = UNPAIRED;
	static ranging_state_t ranging_state = IDLE;

	printf("Starting Tag\n");
	// Setting antenna delay
	dw_set_antenna_delay( ANTENNA_DELAY );

	// Enable rx timeout
	dw_set_rx_timeout( 0xFFFF );
	printf("Timeout: %f ms.\n", (float)dw_get_rx_timeout()/1000.0f);
	dw_enable_rx_timeout();

	// Configure rx
	static dw1000_rx_conf_t rx_conf;
	rx_conf.is_delayed = 0;
	rx_conf.timeout = 0xFFFF;
	dw_conf_rx( &rx_conf );
	dw_receive( DW_TRANCEIVE_ASYNC );

	static uint64_t tsp;
	static uint64_t trr;
	static uint64_t tsf;
	static uint64_t dx_timestamp;
	static uint32_t msgLen;
	static dw_frame_range_t        frameRange;
	static dw_message_range_init_t msgRangeInit;
	static dw_message_response_t   msgResponse;
	// static struct etimer et;
	// etimer_set(&et, CLOCK_SECOND);
	while (1)
	{	
		PROCESS_WAIT_EVENT();
		// printf("Received Event. %i.\n", ev);
		// printf("Receive  state. %i.\n", ranging_state);
		//else
		{
			ranging_state = *(ranging_state_t *)data;
			switch (ranging_state)
			{
				case RANGE_INIT_RECEIVED:
					// printf("Range init received.\n");
					msgLen = dw1000.receive_buffer_len - DW_FRAME_RANGE_LEN - DW_FRAME_FCS_LEN;
					parseFrameRange(dw1000.p_receive_buffer, dw1000.receive_buffer_len, &frameRange);
					parseMessageRangeInit(frameRange.pPayload, msgLen, &msgRangeInit);

					dest_addr     =  frameRange.dest_addr;
					responseDelay = msgRangeInit.response_delay_ms;

					tag_state = PAIRED;
					break;

				case RESPONSE_RECEIVED:
					//printf("Response Received.\n");
					trr = dw_get_rx_timestamp();
					dx_timestamp = trr + dw_ms_to_device_time( (float)responseDelay );

					msgLen = dw1000.receive_buffer_len - DW_FRAME_RANGE_LEN - DW_FRAME_FCS_LEN;
					parseFrameRange(dw1000.p_receive_buffer, dw1000.receive_buffer_len, &frameRange);
					parseMessageResponse(frameRange.pPayload, msgLen, &msgResponse);
					
					// send final
					sendFinal( dest_addr, tsp, trr, &tsf, dx_timestamp );
					printf("Final sent.\n");
					break;

				case ERROR:
				default:
					//printf("Error.\n");
					break;
			}

			ranging_state = IDLE;
		}		

		if ( ranging_state == IDLE /*ev == PROCESS_EVENT_TIMER*/ )
		{
			// send either poll or blink
			if (tag_state == UNPAIRED && ranging_state == IDLE)
			{
				// send blink
				//printf("Sending Blink.\n");
				sendBlink();
				// Listen for answer
				ranging_state = RECEIVING;
				//rx_conf.is_delayed = 1;
				//rx_conf.dx_timestamp = dw_get_tx_timestamp() + dw_ms_to_device_time(4.8f);
				dw_conf_rx( &rx_conf );
				dw_receive( DW_TRANCEIVE_ASYNC );
			}
			else if (tag_state == PAIRED && ranging_state == IDLE)
			{
				// send poll
				printf("Sending Poll.\n");
				sendPoll( dest_addr, &tsp );

				// Listen for answer
				ranging_state = RECEIVING;
				//rx_conf.is_delayed = 1;
				//rx_conf.dx_timestamp = dw_get_tx_timestamp() + dw_ms_to_device_time(responseDelay-0.2);
				dw_conf_rx( &rx_conf );
				dw_receive( DW_TRANCEIVE_ASYNC );
			}
			// etimer_reset(&et);
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/