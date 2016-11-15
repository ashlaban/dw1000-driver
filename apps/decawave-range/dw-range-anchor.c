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
 * \file 	Decawace uwb demo application
 * \author	Kim Albertsson
 *
 * \note If you get a HPDWARN in status reg you are probably taking to long 
 * time to create a delayed send. Increase MESSAGE_DELAY_MS or optimise 
 * critical path for creating packet.
 */

/*===========================================================================*/
/*================================ Includes =================================*/

#include <stdio.h> /* For printf() */
#include <stdint.h>
#include "string.h"
#include <stddef.h>

#include "udelay.h"

#include "dw1000-base.h"
#include "dwFrameTypes.h"
#include "dw-range.h"

#include "contiki.h"
// #include "contiki-net.h"
// #include "rest-engine.h"

/*===========================================================================*/
/*================================ Defines ==================================*/

#define MESSAGE_DELAY_MS 10.0f
#define DEVICE_TIME_UNITS (1.0/499.2e6/128.0)
#define ANTENNA_DELAY ((515.0 / 2.0) * 1e-9 / (1.0/499.2e6/128.0))

#define MASK_40_BIT 0x000000FFFFFFFFFFULL

/*===========================================================================*/
/*========================== Public Declarations ============================*/

/**
 * \brief Keeps track of all variables needed for a distance measurement.
 */
typedef struct
{
	uint64_t tsp; /**< Poll     sent     time */
	uint64_t trp; /**< Poll     received time */
	uint64_t tsr; /**< Response sent     time */
	uint64_t trr; /**< Response received time */
	uint64_t tsf; /**< Final    sent     time */
	uint64_t trf; /**< Final    received time */
	
	int64_t  ttrt; /**< Round trip time as seen by tag.    */
	int64_t  tart; /**< Round trip time as seen by anchor. */
	int64_t  trt;  /**< Estimated round trip time.         */
	int64_t  tof;  /**< Estimated Time of Flight. [~15.65ps per tick] */
	double   s;    /**< Estimated distance                 */

} distance_meas_t;

typedef struct
{
	uint32_t num_meas;
	uint32_t num_errors;

	// Measurements
	float accumulated_distance;
	float max;
	float min;
	float stddev;

	// Quality
	float rx_power;
	float fp_power;
	float noise_level;
	float fp_ampl;
	
	// Sensors
	float temperature;
	float voltage;
} statistics_t;

extern dw1000_base_driver dw1000;
extern struct process * interrupt_handler_callback;

static distance_meas_t dist_meas;
static statistics_t    stats;

// For coap service
float lastMeasuredDistance = 0.f;
float accumulated_distance = 0.f;

// static void res_get_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
// RESOURCE(res_dw_range, "title=\"Distance: ?len=0..\";rt=\"Text\"", res_get_handler, NULL, NULL, NULL);

/*---------------------------------------------------------------------------*/
PROCESS(dw_main_process           , "Main process");
PROCESS(dw_anchor_process         , "Anchor process (dw1000)");
AUTOSTART_PROCESSES(&dw_main_process);
/*---------------------------------------------------------------------------*/

/*===========================================================================*/
/*============================ Public Functions =============================*/

/**
 * \brief Used to convert from raw measurements to real world distances.
 *
 * \details
 *
 * \todo Present error bounds on this function.
 * 
 * @param  distance raw measurement
 * @return          estimated distance
 */
static float distance_correction( float measurement )
{
	// return distance*0.8719 - 32.8600;
	// return distance*0.9876 - 37.0932;
	// return distance*0.9430 - 72.4511;
	return measurement;
}

void print_statistics( statistics_t * stats )
{
	float avg = stats->accumulated_distance/stats->num_meas;
	printf("========================================\n");
	printf("Measurement no. %i\n", stats->num_meas);
	printf("--- Data -------------------------------\n");
	printf("Avg: %f\n", avg);
	printf("Max: %f\n", stats->max);
	printf("Min: %f\n", stats->min);
	printf("Std dev: %f\n", stats->stddev);
	printf("--- Quality ----------------------------\n");
	stats->rx_power    = dw_get_rx_power();
	stats->fp_power    = dw_get_fp_power();
	stats->noise_level = dw_get_noise_level();
	stats->fp_ampl     = dw_get_fp_ampl();
	printf("rx_power   : %f\n", stats->rx_power);
	printf("fp_power   : %f\n", stats->fp_power);
	printf("noise_level: %f\n", stats->noise_level);
	printf("fp_ampl    : %f\n", stats->fp_ampl);
	printf("--- Sensors ----------------------------\n");
	stats->temperature = dw_get_temperature(DW_ADC_SRC_LATEST);
	stats->voltage     = dw_get_voltage(DW_ADC_SRC_LATEST);
	printf("temperature: %f\n", stats->temperature);
	printf("voltage    : %f\n", stats->voltage);
	printf("--- Diagnostics ------------------------\n");
	printf("num_packets_rx   : %i\n", packet_stats.num_packets_rx);
	printf("num_packets_tx   : %i\n", packet_stats.num_packets_tx);
	printf("num_packet_errors: %i\n", packet_stats.num_errors);
	printf("========================================\n");
}

void reset_statistics( statistics_t * stats )
{
	stats->num_meas = 0;
	stats->accumulated_distance = 0.f;
	stats->max    = -1e9;
	stats->min    =  1e9;
	stats->stddev = 0.f;
	stats->num_errors = 0;
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw_main_process, ev, data)
{
	PROCESS_BEGIN();

	// Init coap
	// rest_init_engine();
	// rest_activate_resource(&res_dw_range, "test/distance");

	// Init dw1000
	process_start(&dw_interrupt_callback_proc, NULL);

	dw_init();
	dw_conf_print();
	interrupt_handler_callback = &dw_anchor_process;
	reset_statistics( &stats );
	process_start(&dw_anchor_process, NULL);

	// Start code proper
	while(1)
	{
		PROCESS_WAIT_EVENT();
	}
 
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw_anchor_process, ev, data)
{
	PROCESS_BEGIN();

	printf("Starting Anchor\n");
	// For precise measurements this should be calibrated per device
	dw_set_antenna_delay( ANTENNA_DELAY );

	static ranging_state_t ranging_state = IDLE;
	static uint64_t destAddr = 0ULL;

	dw_frame_blink_t   frameBlink;
	dw_frame_range_t   frameRangePoll;
	dw_message_poll_t  msgPoll;
	dw_frame_range_t   frameRangeFinal;
	dw_message_final_t msgFinal;
	while( 1 )
	{
		if (ranging_state == IDLE)
		{
			// This would be in the main switch as 
			// case IDLE:
			// if contiki multithreading would have allowed it.
			dw1000_rx_conf_t rx_conf;
			rx_conf.is_delayed = 0;
			rx_conf.timeout = 0xFFFF;
			dw_conf_rx( &rx_conf );	
			dw_receive( DW_TRANCEIVE_ASYNC );
			PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
			ranging_state = *(ranging_state_t *)data;
	
			// printf("State: %i\n", ranging_state);
			// printf("ranging_state: %i\n", ranging_state);
		} 

		uint32_t msgLen;
		uint64_t dx_timestamp;
		switch (ranging_state)
		{
			case MEASUREMENT_ADDED:
				ranging_state = IDLE;
				
				if (stats.num_meas % 100  != 0) { break; }
				print_statistics(&stats);
				if (stats.num_meas != 1000) { break; }

				// For easy calibration of distance_correction().
				// Now you can take one sample and then reset device.
				// Remove loop when calibration is done.
				while(1){};

				// Reset statistics.
				reset_statistics( &stats );
				printf("Reset.\n");

				break;

			case BLINK_RECEIVED:
				
				parseFrameBlink(dw1000.p_receive_buffer, dw1000.receive_buffer_len, &frameBlink);
				destAddr = frameBlink.tag_id;

				dx_timestamp = dw_get_rx_timestamp() + dw_ms_to_device_time( MESSAGE_DELAY_MS );

				sendRangeInit( destAddr, (uint16_t)MESSAGE_DELAY_MS, dx_timestamp );
				printf("Sent ranging init\n");
				ranging_state = IDLE;
				break;

			case POLL_RECEIVED:
				dist_meas.trp = dw_get_rx_timestamp();
				dx_timestamp = dist_meas.trp + dw_ms_to_device_time( MESSAGE_DELAY_MS );
				//printArray(dw1000.p_receive_buffer, dw1000.receive_buffer_len);

				msgLen = dw1000.receive_buffer_len - DW_FRAME_RANGE_LEN - DW_FRAME_FCS_LEN;
				parseFrameRange( dw1000.p_receive_buffer, dw1000.receive_buffer_len, &frameRangePoll );
				parseMessagePoll( frameRangePoll.pPayload, msgLen, &msgPoll );

				// TODO: Keep track of paired tags
				if (destAddr == 0x0ULL)
				{
					destAddr = frameRangePoll.src_addr;
				}

				sendResponse( destAddr, &dist_meas.tsr, dx_timestamp );
				// printArray( dw1000.p_receive_buffer, dw1000.receive_buffer_len );
				ranging_state = IDLE;
				break;

			case FINAL_RECEIVED:
				dist_meas.trf = dw_get_rx_timestamp();

				msgLen = dw1000.receive_buffer_len - DW_FRAME_RANGE_LEN - DW_FRAME_FCS_LEN;
				parseFrameRange( dw1000.p_receive_buffer, dw1000.receive_buffer_len, &frameRangeFinal );
				parseMessageFinal( frameRangeFinal.pPayload, msgLen, &msgFinal );
				
				memcpy( &dist_meas.tsp, msgFinal.poll_msg_tx_timestamp , 5 );
				memcpy( &dist_meas.trr, msgFinal.resp_msg_rx_timestamp , 5 );
				memcpy( &dist_meas.tsf, msgFinal.final_msg_tx_timestamp, 5 );

				uint64_t tsp = dist_meas.tsp;
				uint64_t trp = dist_meas.trp;
				uint64_t tsr = dist_meas.tsr;
				uint64_t trr = dist_meas.trr;
				uint64_t tsf = dist_meas.tsf;
				uint64_t trf = dist_meas.trf;
				dist_meas.ttrt  = ((trr - tsp) & MASK_40_BIT) - ((tsr - trp) & MASK_40_BIT);
				dist_meas.tart  = ((trf - tsr) & MASK_40_BIT) - ((tsf - trr) & MASK_40_BIT);
				dist_meas.trt   = ((dist_meas.tart & MASK_40_BIT) + (dist_meas.ttrt & MASK_40_BIT)) >> 1;
				dist_meas.tof   = dist_meas.trt >> 1;
				dist_meas.s     = 3e8 * 15.65e-12 * (double)dist_meas.tof;
				dist_meas.s     = distance_correction(dist_meas.s);
				if (dist_meas.s > 200.0f)
				{
					/* 
						BUG: sometimes we get a really large value... 
						This is possibly fixed now... (has to do with delayed 
						send ignoring lowest 9 bits.)
					*/
					++stats.num_errors;
					break;
				}

				stats.accumulated_distance += dist_meas.s;
				stats.max = (dist_meas.s > stats.max) ? dist_meas.s : stats.max;
				stats.min = (dist_meas.s < stats.min) ? dist_meas.s : stats.min;
				++stats.num_meas;
				ranging_state = MEASUREMENT_ADDED;
				break;

			case ERROR:
			default   :
				++packet_stats.num_errors; // TODO: bad encapsulation, change to software errors.
				ranging_state = IDLE;
				break;
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
// static void
// res_get_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
// {
// 	/**
// 	 * \note This should be working... But it was a long time ago I tested it 
// 	 * so it probably won't.
// 	 */
	
// 	// Set up response payload.
// 	static const uint32_t length = 21;
// 	static char message[128];
// 	sprintf( message, "Distance to tag: %4f", lastMeasuredDistance );
// 	memcpy(buffer, message, length);

// 	// Set up response packet
// 	REST.set_header_content_type(response, REST.type.TEXT_PLAIN); /* text/plain is the default, hence this option could be omitted. */
// 	REST.set_header_etag(response, (uint8_t *) &length, 1);
// 	REST.set_response_payload(response, buffer, length);
// }