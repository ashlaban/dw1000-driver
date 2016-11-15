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

#ifndef DW_SPI_H
#define DW_SPI_H

/**
 * \file dw1000-spi.h
 * \author Kim Albertsson
 * \date 2014-Oct-16
 */

/**
 * \ingroup hal
 * \addtogroup spi SPI interface
 *
 * \brief SPI interface used in the dw1000 drivers. Split into tow parts, the
 * platform-specific primitives and the general composite functions. You will
 * usually want to use the composite functions for convenient transfers of data
 * but the primitives are provided should you desire their use.
 *
 * \todo Implement DMA transfer. Useful for transferring large payloads
 * efficiently. Required I think if you want to utilise the 6.8 Mbps link to
 * capacity.
 *
 * \todo Merge the dw1000-spi interface into the dw1000-hal interface. The SPI
 * should be a submodule of the HAL.
 * 
 * @{
 */

#include <stdint.h>

/**
 * \brief Indicates whether the current transfer is part of an SPI transaction 
 * or not.
 * 
 * \details An SPI transaction is a number of logically grouped SPI transfers.
 * For example, a register access on the device consists of writing to the 
 * address of the register followed by reading a number of bytes. If a 
 * transaction is not used in this case the device will forget the previously 
 * committed command.
 */
typedef enum 
{
	DW_SPI_TRANSFER_DONE = 0, /**< Indicates this is last transfer in transaction */
	DW_SPI_TRANSFER_CONT = 1  /**< Indicates that there are more transfers to be done. */
} dw_spi_transfer_flag_t;

/* Primitive SPI functions - These are implemented in a platform-specific file e.g. mulle-spi.c */
void     dw_spi_init(void);
void     dw_spi_write_byte(uint8_t byte, dw_spi_transfer_flag_t continue_transfer);
uint32_t dw_spi_read_byte(void);

/* Composite SPI functions - These high level spi functions are implemented using the primitive ones. Can be found in dw_spi.c */
uint32_t dw_spi_transfer_byte( uint8_t  bytes , dw_spi_transfer_flag_t continue_transfer);
void     dw_spi_read_n_bytes(   uint32_t nBytes, uint8_t * p_data, dw_spi_transfer_flag_t continue_transfer );
void     dw_spi_write_n_bytes(  uint32_t nBytes, uint8_t * p_data, dw_spi_transfer_flag_t continue_transfer );

/** @}*/

#endif