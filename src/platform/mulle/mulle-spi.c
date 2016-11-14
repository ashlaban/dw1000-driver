#include "dw1000-spi.h"
#include "MK60N512VMD100.h"

/**
 * \brief Initialise the SPI interface. The mulle SPI uses pins on port E.
 *  - *MOSI     :* PE1 (Port E, pin 1)
 *  - *Clock    :* PE2 (Port E, pin 2)
 *  - *MISO     :* PE3 (Port E, pin 3)
 *  - *Chip sel.:* PE4 (Port E, pin 4)
 */
void
dw_spi_init(void)
{
	SIM_SCGC5  |= SIM_SCGC5_PORTE_MASK; // Enable clock for port E
	PORTE_PCR1 |= 0x0200; /* mosi */
	PORTE_PCR2 |= 0x0200; /* clock */
	PORTE_PCR3 |= 0x0200; /* miso */
	PORTE_PCR4 |= 0x0200; /* chip sel  */

	SIM_SCGC6  |= SIM_SCGC6_SPI1_MASK; // Enable clock for spi1
	SPI1_MCR    = 0x00000001;
	SPI1_MCR    = 0x803F3000;
	SPI1_CTAR0  = 0x38002224;
}

/**
 * \brief Write a single byte to the device. Completes only half the 
 * read/write cycle. 
 * \param byte              Byte to write.
 * \param continue_transfer Indicates whether this write is part of a 
 *                          transaction or not.
 */
void
dw_spi_write_byte(uint8_t byte, dw_spi_transfer_flag_t continue_transfer)
{
	uint32_t chipSel = 0x1;
	uint32_t send = SPI_PUSHR_PCS(chipSel) | SPI_PUSHR_TXDATA(byte);
	
	if (continue_transfer) { send |= SPI_PUSHR_CONT_MASK; }

	SPI1_PUSHR = send;
}

/**
 * \brief Reads a single byte to the device. Completes only half the 
 * read/write cycle.
 * \return A single byte read from device.
 */
uint32_t
dw_spi_read_byte(void)
{
	uint32_t result;

 	while (!(SPI1_SR & SPI_SR_TCF_MASK));
	SPI1_SR |= SPI_SR_TCF_MASK;
	result = SPI1_POPR;
	return result & 0xffff;
}