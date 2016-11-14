#include "dw1000-spi.h"

/**
 * \file dw1000-spi.c
 * \author Kim Albertsson
 * \date 2014-Oct-16
 *
 * \brief Implementation of SPI composite functions using the platform-specific
 * primitive functions.
 */

/**
 * \brief Does a simultaneous read and write of a byte to the device. Can carry
 * information in one direction only. It is up to you to keep track of which 
 * direction is relevant.
 *
 * \details Since the SPI protocol is basically a circular shift you need to 
 * perform a write and a read to actually transfer data bestween host and 
 * device.
 * 
 * \param[in]  byte              Byte to write to dw1000.
 * \param[in]  continue_transfer Indicates whether this is the final byte in a 
 *                               transaction or not.
 * \return                       Byte read from device.
 */
uint32_t
dw_spi_transfer_byte(uint8_t byte, dw_spi_transfer_flag_t continue_transfer)
{
	dw_spi_write_byte(byte, continue_transfer);
	return dw_spi_read_byte();
}

/**
 * \brief Reads a set of n_bytes from the device.
 * \param[in] n_bytes           Number of bytes to read.
 * \param[in] pData             Pointer to a byte array of length n_bytes.
 * \param[in] continue_transfer Indicates whether this transfer is part of a 
 *                               transaction or not.
 */
void
dw_spi_read_n_bytes( uint32_t n_bytes, uint8_t * pData, dw_spi_transfer_flag_t continue_transfer )
{
	if (n_bytes == 0) return;

	while( n_bytes-- > 1)
	{
		*pData = dw_spi_transfer_byte(0x0, DW_SPI_TRANSFER_CONT);
		pData++;
	}
	
	// Potentially end spi transaction
	*pData = dw_spi_transfer_byte(0x0, continue_transfer);
}

/**
 * \brief Write a set of n_bytes to the device.
 * \param[in]  n_bytes           Number of bytes to write.
 * \param[out] pData             Pointer to a byte array of length n_bytes.
 * \param[in]  continue_transfer Indicates whether this transfer is part of a 
 *                               transaction or not.
 */
void
dw_spi_write_n_bytes( uint32_t n_bytes, uint8_t * pData, dw_spi_transfer_flag_t continue_transfer )
{
	if (n_bytes == 0) return;

	while( n_bytes-- > 1)
	{
		dw_spi_transfer_byte(*pData, DW_SPI_TRANSFER_CONT);
		pData++;
	}
	
	// Potentially end spi transaction
	dw_spi_transfer_byte(*pData, continue_transfer);
}