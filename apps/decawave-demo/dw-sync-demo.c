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
 * 	Minimal working example, dw1000 driver.
 * \author
 *	Kim Albertsson
 */

#include "contiki.h"

#include <stdio.h> /* For printf() */
#include <stdint.h>

#include "udelay.h"

#include "dw1000.h"

/*---------------------------------------------------------------------------*/
PROCESS(dw_main_process, "DW main process");
PROCESS(dw_interrupt_callback, "dw1000 interrupt callback");
AUTOSTART_PROCESSES(&dw_main_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw_main_process, ev, data)
{
  PROCESS_BEGIN();

  	// Configures the dw1000, default configuration
	dw_init();
	dw_conf_print();

	while(1)
	{
		// Configure reception
		dw1000_rx_conf_t rx_conf;
		rx_conf.is_delayed = 0;
		rx_conf.timeout = 0xFFFF; // ~65 ms

		// Receive
		dw_conf_rx( &rx_conf );
		dw_receive( DW_TRANCEIVE_SYNC );

		// Configure transmission
		dw1000_tx_conf_t tx_conf;
		tx_conf.data_len = 10;
		tx_conf.is_delayed = 0;
		dw_conf_tx( &tx_conf );

		// Transmit
		static uint8_t counter = 0;
		uint8_t  p_data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
		p_data[0] = counter;
		dw_transmit( p_data, tx_conf.data_len, DW_TRANCEIVE_SYNC );
	}
 
  PROCESS_END();
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

		if (status_reg & DW_TXFRS_MASK)
		{
			/*
		 	 * TX - Frame Sent
		 	 */
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
		 	dw1000.state = DW_STATE_IDLE;
		 	
			dw_get_rx_buffer(); /* updates dw1000.state */
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
		 	dw1000.state = DW_STATE_ERROR;

			// Error handling
			dw1000.state = DW_STATE_ERROR;
			st = ERROR;
			
			// printf("status_reg: %x\n", status_reg);

			// Reset all events.
			const uint64_t one_mask = 0xFFFFFFFFFFFFFFFFULL;
			dw_write_reg( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&one_mask );
			process_post(interrupt_handler_callback, PROCESS_EVENT_CONTINUE, &st);
		}
	}
	PROCESS_END();
}
/*---------------------------------------------------------------------------*/