#include "dw1000-base.h"

/**
 *  \file dw1000-base.c
 *  \author Kim Albertsson
 *  \date 2014-Oct-16
 */

/*===========================================================================*/
/*================================ Includes =================================*/

#include "string.h" // memcpy...
#include "stdio.h"
#include "udelay.h"
#include "math.h"

#include "dw1000-spi.h"
#include "dw1000-hal.h"

/*===========================================================================*/
/*================================ Defines ==================================*/

/**
 * \brief Used to converet a timestamp read from the device using 
 * \ref dw_get_rx_timestamp or \ref dw_get_tx_timestamp to real seconds.
 */
#define DW_MS_TO_DEVICE_TIME_SCALE 62.6566416e6f

/*===========================================================================*/
/*=========================== Private Functions =============================*/

void dw_enable_interrupt( uint32_t mask );

void dw_trxoff();

void dw_init_rx();
void dw_init_tx();

/*===========================================================================*/
/*===========================================================================*/
/**
 * \addtogroup Configuration
 *
 * \brief Configuration of the dw1000 device. Handles both general device
 * configuration and configuration of the transmitter and receiver.
 *
 * @{
 */

/**
 * \brief Initialise the DW1000. Prepare the device and drivers for use and
 * configure the dw1000 with a default configuration.
 *
 * \details The default configuration is as follows:
 * \code
 *     dw1000.conf.prf             = DW_PRF_16_MHZ; 
 *     dw1000.conf.channel         = DW_CHANNEL_5;
 *     dw1000.conf.preamble_length = DW_PREAMBLE_LENGTH_128;
 *     dw1000.conf.preamble_code   = DW_PREAMBLE_CODE_3;
 *     dw1000.conf.pac_size        = DW_PAC_SIZE_8;
 *     dw1000.conf.sfd_type        = DW_SFD_STANDARD;
 *     dw1000.conf.data_rate       = DW_DATA_RATE_850_KBPS;
 * \endcode
 */
void dw_init()
{
	dw1000.state = DW_STATE_INITIALIZING;

	// Init required hardware components
	dw_spi_init();
	dw_hal_init();

	// Init dw1000
	dw_trxoff(); /* Simple reset of device. */

	dw_clear_pending_interrupt( 0x00000007FFFFFFFFULL );
	const uint32_t mask = DW_MTXFRS_MASK
						| DW_MRXDFR_MASK
						| DW_MRXPHE_MASK
						| DW_MRXRFTO_MASK
						| DW_MRXPTO_MASK
						| DW_MRXSFDTO_MASK
						| DW_MRXRFSL_MASK;
	dw_enable_interrupt( mask );

	// Load LDE Code
	// For info, see DW1000 User Manual p. 22
	// TODO: Move this to dw1000_base_conf_t.
	const uint32_t lde1  = 0x0301;
	const uint32_t lde2  = 0x8000;
	const uint32_t lde3  = 0x0200;
	dw_write_subreg(0x36, 0x00, 2, (uint8_t *)&lde1);
	dw_write_subreg(0x2D, 0x06, 2, (uint8_t *)&lde2);
	udelay(250); // Wait at least 150 us
	dw_write_subreg(0x36, 0x00, 2, (uint8_t *)&lde3);
	// // Disable LDE
	// // TODO: Read old value and flip lderun bit
	// //value = dw_read_subreg_64(0x36, 0x04, 4);
	// //printf("Value: %llx \n", value);
	// const uint32_t lderune = 0x81000738;
	// dw_write_subreg(0x36, 0x04, 4, (uint8_t *)&lderune);

	// New init code
	// TODO: Make default configuration optional
	dw1000.conf.prf             = DW_PRF_16_MHZ; 
	dw1000.conf.channel         = DW_CHANNEL_5;
	dw1000.conf.preamble_length = DW_PREAMBLE_LENGTH_128;
	dw1000.conf.preamble_code   = DW_PREAMBLE_CODE_3;
	dw1000.conf.pac_size        = DW_PAC_SIZE_8;
	dw1000.conf.sfd_type        = DW_SFD_STANDARD;
	dw1000.conf.data_rate       = DW_DATA_RATE_850_KBPS;
	dw_conf( &dw1000.conf );

	// Print information about the board
	printf( "Initialising device: %llu\n", dw_get_device_id() );

	dw1000.state = DW_STATE_IDLE;
}

/**
 * \brief Uploads and applies a given configuration to the dw1000. See \ref
 * dw1000_base_conf_t for information about different configuration options.
 *
 * \param[in] dw_conf 	Configuration to be applied.
 */
void dw_conf(dw1000_base_conf_t * dw_conf)
{
	uint32_t sys_cfg_val   = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
	uint32_t tx_fctrl_val  = dw_read_reg_32(DW_REG_TX_FCTRL, 4);
	uint32_t chan_ctrl_val = dw_read_reg_32(DW_REG_CHAN_CTRL, DW_LEN_CHAN_CTRL);
	uint32_t agc_tune1_val;
	const uint32_t agc_tune2_val = 0x2502A907;	/* Always use this */;
	const uint32_t agc_tune3_val = 0x0055;		/* Always use this */;
	uint32_t drx_tune0b_val;
	uint32_t drx_tune1a_val;
	uint32_t drx_tune1b_val;
	uint32_t drx_tune2_val;
	uint32_t drx_tune4h_val;
	uint32_t rf_rxctrl_val;
	uint32_t rf_txctrl_val;
	uint32_t tc_pgdelay_val;
	uint32_t fs_pllcfg_val;
	uint32_t fs_plltune_val;

	// === Configure PRF
	tx_fctrl_val  &= ~DW_TXPRF_MASK;
	chan_ctrl_val &= ~DW_RXPRF_MASK;
	switch (dw_conf->prf)
	{
		case DW_PRF_16_MHZ:
			agc_tune1_val  = 0x8870;
			drx_tune1a_val = 0x0087;
			tx_fctrl_val  |= (0x01 << DW_TXPRF) & DW_TXPRF_MASK;
			chan_ctrl_val |= (0x01 << DW_RXPRF) & DW_RXPRF_MASK;
			break;

		case DW_PRF_64_MHZ:
			agc_tune1_val  = 0x889B;
			drx_tune1a_val = 0x008D;
			tx_fctrl_val  |= (0x02 << DW_TXPRF) & DW_TXPRF_MASK;
			chan_ctrl_val |= (0x02 << DW_RXPRF) & DW_RXPRF_MASK;
			break;
	}

	// === Configure rx/tx channel
	chan_ctrl_val &= ~DW_TXCHAN_MASK;
	chan_ctrl_val &= ~DW_RXCHAN_MASK;

	uint8_t channel = (uint8_t)dw_conf->channel  & 0x1F;
	chan_ctrl_val |= channel;      // tx chan
	chan_ctrl_val |= channel << 5; // rx chan

	switch (dw_conf->channel)
	{
		case DW_CHANNEL_1:
			rf_rxctrl_val  = 0xD8;
			rf_txctrl_val  = 0x00005C40;
			tc_pgdelay_val = 0xC9;
			fs_pllcfg_val  = 0x09000407;
			fs_plltune_val = 0x1E;
			break;
		case DW_CHANNEL_2:
			rf_rxctrl_val  = 0xD8;
			rf_txctrl_val  = 0x00045CA0;
			tc_pgdelay_val = 0xC2;
			fs_pllcfg_val  = 0x08400508;
			fs_plltune_val = 0x26;
			break;
		case DW_CHANNEL_3:
			rf_rxctrl_val  = 0xD8;
			rf_txctrl_val  = 0x00086CC0;
			tc_pgdelay_val = 0xC5;
			fs_pllcfg_val  = 0x08401009;
			fs_plltune_val = 0x5E;
			break;
		case DW_CHANNEL_4:
			rf_rxctrl_val  = 0xBC;
			rf_txctrl_val  = 0x00045C80;
			tc_pgdelay_val = 0x95;
			fs_pllcfg_val  = 0x08400508;
			fs_plltune_val = 0x26;
			break;
		case DW_CHANNEL_5:
			rf_rxctrl_val  = 0xD8;
			rf_txctrl_val  = 0x001E3FE0;
			tc_pgdelay_val = 0xC0;
			fs_pllcfg_val  = 0x0800041D;
			fs_plltune_val = 0xA6;
			break;
		case DW_CHANNEL_7:
			rf_rxctrl_val  = 0xBC;
			rf_txctrl_val  = 0x001E7DE0;
			tc_pgdelay_val = 0x93;
			fs_pllcfg_val  = 0x0800041D;
			fs_plltune_val = 0xA6;
			break;
	}

	// === Configure Preamble length
	tx_fctrl_val  &= ~DW_TXPSR_MASK;
	tx_fctrl_val  &= ~DW_PE_MASK;
	if (dw_conf->preamble_length == DW_PREAMBLE_LENGTH_64)
	{
		drx_tune1b_val = 0x0010;
		drx_tune4h_val = 0x0010;
	}
	else if (dw_conf->preamble_length <= DW_PREAMBLE_LENGTH_1024)
	{
		drx_tune1b_val = 0x0020;
		drx_tune4h_val = 0x0028;
	}
	else if (dw_conf->preamble_length >  DW_PREAMBLE_LENGTH_1024)
	{
		drx_tune1b_val = 0x0064;
		drx_tune4h_val = 0x0028;
	}
	switch (dw_conf->preamble_length)
	{
		case DW_PREAMBLE_LENGTH_64:
			tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x00 << DW_PE)    & DW_PE_MASK;
			break;
		case DW_PREAMBLE_LENGTH_128:
			tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x01 << DW_PE)    & DW_PE_MASK;
			break;
		case DW_PREAMBLE_LENGTH_256:
			tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x02 << DW_PE)    & DW_PE_MASK;
			break;
		case DW_PREAMBLE_LENGTH_512:
			tx_fctrl_val   |= (0x01 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x03 << DW_PE)    & DW_PE_MASK;
			break;
		case DW_PREAMBLE_LENGTH_1024:
			tx_fctrl_val   |= (0x02 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x00 << DW_PE)    & DW_PE_MASK;
			break;
		case DW_PREAMBLE_LENGTH_1536:
			tx_fctrl_val   |= (0x02 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x01 << DW_PE)    & DW_PE_MASK;
			break;
		case DW_PREAMBLE_LENGTH_2048:
			tx_fctrl_val   |= (0x02 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x02 << DW_PE)    & DW_PE_MASK;
			break;
		case DW_PREAMBLE_LENGTH_4096:
			tx_fctrl_val   |= (0x03 << DW_TXPSR) & DW_TXPSR_MASK;
			tx_fctrl_val   |= (0x00 << DW_PE)    & DW_PE_MASK;
			break;
	}

	// === Configure Preamble code
	chan_ctrl_val &= ~DW_TX_PCODE_MASK;
	chan_ctrl_val &= ~DW_RX_PCODE_MASK;

	uint8_t preamble_code = (uint8_t)dw_conf->preamble_code;
	chan_ctrl_val |= (preamble_code << DW_TX_PCODE) & DW_TX_PCODE_MASK;
	chan_ctrl_val |= (preamble_code << DW_RX_PCODE) & DW_RX_PCODE_MASK;

	// === Configure PAC size
	switch (dw_conf->pac_size)
	{
		case DW_PAC_SIZE_8:
			if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x311A002D;}
			else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x313B006B;}
			break;
		case DW_PAC_SIZE_16:
			if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x331A0052;}
			else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x333B00BE;}
			break;
		case DW_PAC_SIZE_32:
			if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x351A009A;}
			else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x353B015E;}
			break;
		case DW_PAC_SIZE_64:
			if      (dw_conf->prf == DW_PRF_16_MHZ) {drx_tune2_val = 0x371A011D;}
			else if (dw_conf->prf == DW_PRF_64_MHZ) {drx_tune2_val = 0x373B0296;}
			break;
	}

	// === Configure SFD
	// TODO: Implement user specified 
	chan_ctrl_val &= ~DW_DWSFD_MASK;

	if (dw_conf->sfd_type == DW_SFD_USER_SPECIFIED)
		DW_ERROR("dw_conf - SFD: User specified SFD not implemented");
	switch (dw_conf->sfd_type)
	{
		case DW_SFD_STANDARD:
			chan_ctrl_val &= ~((1 << DW_DWSFD) & DW_DWSFD_MASK);
			break;
		case DW_SFD_NON_STANDARD:
			chan_ctrl_val |= (1 << DW_DWSFD) & DW_DWSFD_MASK;
			break;
		case DW_SFD_USER_SPECIFIED:
			// Not implemented yet!
			break;
	}
	switch (dw_conf->data_rate)
	{
		case DW_DATA_RATE_110_KBPS:
			if      (dw_conf->sfd_type == DW_SFD_STANDARD    ) {drx_tune0b_val = 0x000A;}
			else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0016;}
			break;
		case DW_DATA_RATE_850_KBPS:
			if      (dw_conf->sfd_type == DW_SFD_STANDARD    ) {drx_tune0b_val = 0x0001;}
			else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0006;}
			break;
		case DW_DATA_RATE_6800_KBPS:
			if      (dw_conf->sfd_type == DW_SFD_STANDARD    ) {drx_tune0b_val = 0x0001;}
			else if (dw_conf->sfd_type == DW_SFD_NON_STANDARD) {drx_tune0b_val = 0x0002;}
			break;
	}

	// === Configure Data rate
	sys_cfg_val  &= ~DW_RXM110K_MASK;
	tx_fctrl_val &= ~DW_TXBR_MASK;
	switch (dw_conf->data_rate)
	{
		case DW_DATA_RATE_110_KBPS:
			sys_cfg_val  |= (1<<DW_RXM110K) & DW_RXM110K_MASK;
			tx_fctrl_val |= (0x00 << DW_TXBR) & DW_TXBR_MASK;
			break;
		case DW_DATA_RATE_850_KBPS:
			sys_cfg_val  &= ~((1<<DW_RXM110K) & DW_RXM110K_MASK);
			tx_fctrl_val |= (0x01 << DW_TXBR) & DW_TXBR_MASK;
			break;
		case DW_DATA_RATE_6800_KBPS:
			sys_cfg_val  &= ~((1<<DW_RXM110K) & DW_RXM110K_MASK);
			tx_fctrl_val |= (0x02 << DW_TXBR) & DW_TXBR_MASK;
			break;
	}

	// Commit configuration to device
	dw_write_reg(DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&sys_cfg_val);
	dw_write_reg(DW_REG_TX_FCTRL, 4, (uint8_t *)&tx_fctrl_val);
	dw_write_reg(DW_REG_CHAN_CTRL, DW_LEN_CHAN_CTRL, (uint8_t *)&chan_ctrl_val);
	dw_write_subreg(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE1, DW_SUBLEN_AGC_TUNE1, (uint8_t *)&agc_tune1_val);
	dw_write_subreg(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE2, DW_SUBLEN_AGC_TUNE2, (uint8_t *)&agc_tune2_val);
	dw_write_subreg(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE3, DW_SUBLEN_AGC_TUNE3, (uint8_t *)&agc_tune3_val);
	dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE0b, DW_SUBLEN_DRX_TUNE0b, (uint8_t *)&drx_tune0b_val);
	dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1a, DW_SUBLEN_DRX_TUNE1a, (uint8_t *)&drx_tune1a_val);
	dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1b, DW_SUBLEN_DRX_TUNE1b, (uint8_t *)&drx_tune1b_val);
	dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE2 , DW_SUBLEN_DRX_TUNE2 , (uint8_t *)&drx_tune2_val );
	dw_write_subreg(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE4h, DW_SUBLEN_DRX_TUNE4h, (uint8_t *)&drx_tune4h_val);
	dw_write_subreg(DW_REG_RF_CONF , DW_SUBREG_RF_RXCTRLH, DW_SUBLEN_RF_RXCTRLH, (uint8_t *)&rf_rxctrl_val );
	dw_write_subreg(DW_REG_RF_CONF , DW_SUBREG_RF_TXCTRL , DW_SUBLEN_RF_TXCTRL , (uint8_t *)&rf_txctrl_val );
	dw_write_subreg(DW_REG_TX_CAL  , DW_SUBREG_TC_PGDELAY, DW_SUBLEN_TC_PGDELAY, (uint8_t *)&tc_pgdelay_val);
	dw_write_subreg(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLCFG , DW_SUBLEN_FS_PLLCFG , (uint8_t *)&fs_pllcfg_val );
	dw_write_subreg(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLTUNE, DW_SUBLEN_FS_PLLTUNE, (uint8_t *)&fs_plltune_val);

	// DW_LOG("Configuration complete.");
}

/**
 * \brief Configures the DW1000 to be ready to receive a message. See \ref
 * dw1000_rx_conf_t.
 *
 * \details Whenever a data should be received the device must be configured
 * properly. An example of how this is done follows.
 * \code
 * dw_rx_conf_t rx_conf;
 * rx_conf.is_delayed = 0;
 * rx_conf.timeout    = 30000; // Unit, approximate milliseconds
 * dw_conf_rx( &rx_conf );
 * dw_receive(DW_TRANCEIVE_ASYNC);
 * \endcode
 *
 * \param[in]	rx_conf Configuration speceification.
 */
void dw_conf_rx( dw1000_rx_conf_t * rx_conf )
{
	// Timeout
	dw_set_rx_timeout(rx_conf->timeout);
	if (rx_conf->timeout)
	{
		dw_enable_rx_timeout();
	}
	else
	{
		dw_disable_rx_timeout();
	}

	// Delayed reception
	if ( rx_conf->is_delayed )
	{
		dw_set_dx_timestamp( rx_conf->dx_timestamp );
		
		uint32_t sys_ctrl_val;
		sys_ctrl_val  = dw_read_reg_32( DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL );
		sys_ctrl_val |= DW_RXDLYE_MASK;
		dw_write_reg( DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl_val);
	}
}

/**
 * \brief Configures the DW1000 to be ready to transmit message. See \ref
 * dw1000_tx_conf_t.
 *
 * \details Whenever a data should be transmitted the device must be configured
 * properly. An example of how this is done follows.
 * \code
 * dw_tx_conf_t tx_conf;
 * tx_conf.is_delayed = 0;
 * dw_conf_tx( &tx_conf );
 * dw_transmit( p_data, p_data_len, DW_TRANCEIVE_SYNC);
 * \endcode
 * 
 * \param[in] tx_conf 	Configuration specification.
 */
void dw_conf_tx( dw1000_tx_conf_t * tx_conf )
{
	// TODO: Handling of long data frames (length > 128 or whatever.)
	// TODO: Cache data..?
	// TODO: Should check dw1000 configuration for FCS enable and add the 2 conditionally.
	uint32_t data_len = tx_conf->data_len;
	data_len += 2; // The +2 is for fcs
	uint32_t tx_frame_control_val = dw_read_reg_32(DW_REG_TX_FCTRL, 4);
	tx_frame_control_val |= (data_len << DW_TXLEN) & DW_TXLEN_MASK;
	dw_write_reg( DW_REG_TX_FCTRL, 4, (uint8_t *)&tx_frame_control_val );

	// Delayed transmission
	if ( tx_conf->is_delayed )
	{
		dw_set_dx_timestamp( tx_conf->dx_timestamp );
		
		uint32_t ctrl_reg_val;
		ctrl_reg_val  = dw_read_reg_32( DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL );
		ctrl_reg_val |= DW_TXDLYS_MASK;
		dw_write_reg( DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&ctrl_reg_val);
	}
}

/**
 * \brief Reads the current configuration from device and prints it using 
 *  printf. The current configuration is all registers that can be modified by
 *  \ref dw_conf, \ref dw_conf_rx and \ref dw_conf_tx.
 */
void
dw_conf_print()
{
	uint32_t sys_cfg_val    = 0;
	uint32_t tx_fctrl_val   = 0;
	uint32_t chan_ctrl_val  = 0;
	uint32_t agc_tune1_val  = 0;
	uint32_t agc_tune2_val  = 0;
	uint32_t agc_tune3_val  = 0;
	uint32_t drx_tune0b_val = 0;
	uint32_t drx_tune1a_val = 0;
	uint32_t drx_tune1b_val = 0;
	uint32_t drx_tune2_val  = 0;
	uint32_t drx_tune4h_val = 0;
	uint32_t rf_rxctrl_val  = 0;
	uint32_t rf_txctrl_val  = 0;
	uint32_t tc_pgdelay_val = 0;
	uint32_t fs_pllcfg_val  = 0;
	uint32_t fs_plltune_val = 0;

	sys_cfg_val   = dw_read_reg_32(DW_REG_SYS_CFG, DW_LEN_SYS_CFG);
	tx_fctrl_val = dw_read_reg_32(DW_REG_TX_FCTRL, 4);
	chan_ctrl_val = dw_read_reg_32(DW_REG_CHAN_CTRL, DW_LEN_CHAN_CTRL);
	agc_tune1_val  = dw_read_subreg_32(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE1, DW_SUBLEN_AGC_TUNE1);
	agc_tune2_val  = dw_read_subreg_32(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE2, DW_SUBLEN_AGC_TUNE2);
	agc_tune3_val  = dw_read_subreg_32(DW_REG_AGC_CTRL, DW_SUBREG_AGC_TUNE3, DW_SUBLEN_AGC_TUNE3);
	drx_tune0b_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE0b, DW_SUBLEN_DRX_TUNE0b);
	drx_tune1a_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1a, DW_SUBLEN_DRX_TUNE1a);
	drx_tune1b_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE1b, DW_SUBLEN_DRX_TUNE1b);
	drx_tune2_val  = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE2 , DW_SUBLEN_DRX_TUNE2 );
	drx_tune4h_val = dw_read_subreg_32(DW_REG_DRX_CONF, DW_SUBREG_DRX_TUNE4h, DW_SUBLEN_DRX_TUNE4h);
	rf_rxctrl_val  = dw_read_subreg_32(DW_REG_RF_CONF , DW_SUBREG_RF_RXCTRLH, DW_SUBLEN_RF_RXCTRLH);
	rf_txctrl_val  = dw_read_subreg_32(DW_REG_RF_CONF , DW_SUBREG_RF_TXCTRL , DW_SUBLEN_RF_TXCTRL );
	tc_pgdelay_val = dw_read_subreg_32(DW_REG_TX_CAL  , DW_SUBREG_TC_PGDELAY, DW_SUBLEN_TC_PGDELAY);
	fs_pllcfg_val  = dw_read_subreg_32(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLCFG , DW_SUBLEN_FS_PLLCFG );
	fs_plltune_val = dw_read_subreg_32(DW_REG_FS_CTRL , DW_SUBREG_FS_PLLTUNE, DW_SUBLEN_FS_PLLTUNE);

	float temperature = dw_get_temperature(DW_ADC_SRC_LATEST);
	float voltage     = dw_get_voltage(DW_ADC_SRC_LATEST);
	
	printf("============================\n");
	printf("DW1000 Current Configuration\n");
	printf("============================\n");
	printf("Device id   : %llx\n", dw_get_device_id());
	printf("sys_status  : %llx\n", dw_read_reg_64(DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS));
	printf("============================\n");
	printf( "sys_cfg    : %x\n", sys_cfg_val    );
	printf( "tx_fctrl   : %x\n", tx_fctrl_val   );
	printf( "chan_ctrl  : %x\n", chan_ctrl_val  );
	printf( "agc_tune1  : %x\n", agc_tune1_val  );
	printf( "agc_tune2  : %x\n", agc_tune2_val  );
	printf( "agc_tune3  : %x\n", agc_tune3_val  );
	printf( "drx_tune0b : %x\n", drx_tune0b_val );
	printf( "drx_tune1a : %x\n", drx_tune1a_val );
	printf( "drx_tune1b : %x\n", drx_tune1b_val );
	printf( "drx_tune2  : %x\n", drx_tune2_val  );
	printf( "drx_tune4h : %x\n", drx_tune4h_val );
	printf( "rf_rxctrl  : %x\n", rf_rxctrl_val  );
	printf( "rf_txctrl  : %x\n", rf_txctrl_val  );
	printf( "tc_pgdelay : %x\n", tc_pgdelay_val );
	printf( "fs_pllcfg  : %x\n", fs_pllcfg_val  );
	printf( "fs_plltune : %x\n", fs_plltune_val );
	printf("============================\n");
	printf( "temperature : %f\n", temperature );
	printf( "voltage     : %f\n", voltage     );
}
/** @}*/

/**
 * \brief Prints a message if SPI-communication is working properly.
 */
void
dw_test(void)
{
	uint32_t canTalk = 0;

	canTalk = ( 0xDECA0130 == dw_read_reg_32(DW_REG_DEV_ID, DW_LEN_DEV_ID) );

	if ( canTalk )
	{
		printf("You can now talk with the device (spi)!\n");
	}
}

/*===========================================================================*/
/*===========================================================================*/
/** 
 * \addtogroup registers Low-level register access
 *
 * \brief Provides low level access to the register file of the dw1000.
 *
 * \details Enables reads and writes to the registers on the dw1000. The
 * register addresses and lengths can be found in \ref dw1000-base.h.
 * @{
 */

/**
 * \brief Reads the value from a register on the dw1000 as a stream of bytes.
 * \param[in] regAddr 	 	Register address as specified in the manual or by
 * 							 the DW_REG_* defines.
 * \param[in] regLen 		Nunmber of bytes to read. Should not be longer than
 * 							 the length specified in the manual or the DW_LEN_*
 * 							 defines.
 * \param[out] pData 		Data read from the device.
 */
void
dw_read_reg( uint32_t regAddr, uint32_t regLen, uint8_t * pData )
{
	// Initiate read
	uint8_t instruction = 0x00 | (regAddr & 0x3F);
	dw_spi_transfer_byte(instruction, DW_SPI_TRANSFER_CONT);
	
	// Read data
	dw_spi_read_n_bytes( regLen, pData, DW_SPI_TRANSFER_DONE );
}

/**
 * \brief Reads the value from a register on the dw1000 as a 32-bit integer.
 * \param[in] regAddr 	 	Register address as specified in the manual or by
 * 							 the DW_REG_* defines.
 * \param[in] regLen 		Nunmber of bytes to read. Should not be longer than
 * 							 the length specified in the manual or the DW_LEN_*
 * 							 defines. Neither should it be larger than 4 bytes.
 * \return A 32-bit unsigned integer read from a register of the dw1000.
 */
uint32_t
dw_read_reg_32( uint32_t regAddr, uint32_t regLen )
{
	uint32_t result = 0;
	dw_read_reg( regAddr, regLen, (uint8_t *)&result );
	return result;
}

/**
 * \brief Reads the value from a register on the dw1000 as a 64-bit integer.
 * \param[in] regAddr 	 	Register address as specified in the manual or by
 * 							 the DW_REG_* defines.
 * \param[in] regLen 		Nunmber of bytes to read. Should not be longer than
 * 							 the length specified in the manual or the DW_LEN_*
 * 							 defines. Neither should it be larger than 8 bytes.
 * \return A 64-bit unsigned integer read from a register of the dw1000.
 */
uint64_t
dw_read_reg_64( uint32_t regAddr, uint32_t regLen )
{
	uint64_t result = 0;
	dw_read_reg(regAddr, regLen, (uint8_t *)&result);
	return result;
}

/**
 * \brief Reads the value from a subregister on the dw1000 as a byte stream.
 * \param[in] reg_addr 	 	Register address as specified in the manual and by
 * 							 the DW_REG_* defines.
 * \param[in] subreg_addr	Subregister address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len	Nunmber of bytes to read. Should not be longer than
 * 							 the length specified in the manual or the 
 * 							 DW_SUBLEN_* defines.
 * \param[out] p_data 		Data read from the device.
 */
void
dw_read_subreg( uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t * p_data )
{
	uint32_t is_three_octet = (subreg_addr > 0x7F);

	// Compose instruction
	uint32_t instruction = 0x40 | (reg_addr & 0x3F);
	instruction = (instruction << 8) | ((subreg_addr & 0x7F  )     ) | (is_three_octet<<7);
	instruction = (instruction << 8) | ((subreg_addr & 0x7F80) >> 7);

	// Write instruction
	// Data is written as --001122 
	uint8_t * pInstr = (uint8_t *)(&instruction) + 2;
	dw_spi_transfer_byte(*pInstr--, DW_SPI_TRANSFER_CONT);
	dw_spi_transfer_byte(*pInstr--, DW_SPI_TRANSFER_CONT);
	if (is_three_octet)
	{
		dw_spi_transfer_byte(*pInstr, DW_SPI_TRANSFER_CONT);	
	}

	// Read data
	dw_spi_read_n_bytes(subreg_len, p_data, DW_SPI_TRANSFER_DONE);
}

/**
 * \brief Reads the value from a subregister on the dw1000 as 32-bit integer.
 * \param[in] reg_addr 	 	Register address as specified in the manual and by
 * 							 the DW_REG_* defines.
 * \param[in] subreg_addr	Subregister address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len	Nunmber of bytes to read. Should not be longer than
 * 							 the length specified in the manual or the 
 * 							 DW_SUBLEN_* defines. Neither should it be larger 
 * 							 than 4 bytes.
 * \return A 32-bit unsigned integer read from a register of the dw1000.
 */
uint32_t
dw_read_subreg_32( uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len )
{
	uint32_t result = 0U;
	dw_read_subreg( reg_addr, subreg_addr, subreg_len, (uint8_t *)&result );
	return result;
}

/**
 * \brief Reads the value from a subregister on the dw1000 as 64-bit integer.
 * \param[in] reg_addr 	 	Register address as specified in the manual and by
 * 							 the DW_REG_* defines.
 * \param[in] subreg_addr	Subregister address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len	Nunmber of bytes to read. Should not be longer than
 * 							 the length specified in the manual or the 
 * 							 DW_SUBLEN_* defines. Neither should it be larger 
 * 							 than 8 bytes.
 * \return A 64-bit unsigned integer read from a register of the dw1000.
 */
uint64_t
dw_read_subreg_64( uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len )
{
	uint64_t result = 0ULL;
	dw_read_subreg( reg_addr, subreg_addr, subreg_len, (uint8_t *)&result );
	return result;
}

/**
 * \brief Writes a stream of bytes to the specified dw1000 register.
 * \param[in] reg_addr		Register address as specified in the manual and by
 * 							 the DW_REG_* defines.
 * \param[in] reg_len 		Nunmber of bytes to write. Should not be longer than
 * 							 the length specified in the manual or the DW_LEN_*
 * 							 defines.
 * \param[in] p_data 		A stream of bytes to write to device.
 */
void
dw_write_reg( 	uint32_t  reg_addr, 
				uint32_t  reg_len, 
				uint8_t * p_data )
{
	uint8_t instruction = 0x80 | (reg_addr & 0x3F);
	dw_spi_write_byte(instruction, DW_SPI_TRANSFER_CONT);
	dw_spi_read_byte();
	
	dw_spi_write_n_bytes( reg_len, p_data, DW_SPI_TRANSFER_DONE);
}

/**
 * \brief Writes a value to a subregister on the dw1000 as a byte stream.
 * \param[in] reg_addr 	 	Register address as specified in the manual and by
 * 							 the DW_REG_* defines.
 * \param[in] subreg_addr	Subregister address as specified in the manual and
 *                           by the DW_SUBREG_* defines.
 * \param[in] subreg_len	Nunmber of bytes to write. Should not be longer
 *                           than the length specified in the manual or the 
 * 							 DW_SUBLEN_* defines.
 * \param[in] p_data 		A stream of bytes to write to device.
 */
void
dw_write_subreg( 	uint32_t  reg_addr, 
					uint32_t  subreg_addr, 
					uint32_t  subreg_len, 
					uint8_t * p_data )
{
	// Check if 3-octet header is requried or if 2 will do
	uint32_t isThreeOctet = (subreg_addr > 0x7F);
	
	// Prepare instruction
	uint32_t instruction = 0x0;
	instruction = (0xC0 | (reg_addr&0x3F));
	instruction = (instruction << 8) | (subreg_addr&0x7F       ) | (isThreeOctet<<7);
	instruction = (instruction << 8) | (subreg_addr&0x7F80 >> 7);
	
	// Write instruction
	uint8_t * pInstr = (uint8_t *)(&instruction) + 2;
	dw_spi_transfer_byte(*pInstr--, DW_SPI_TRANSFER_CONT);
	dw_spi_transfer_byte(*pInstr--, DW_SPI_TRANSFER_CONT);
	if (isThreeOctet) { dw_spi_transfer_byte(*pInstr--, DW_SPI_TRANSFER_CONT); }
	
	// Write data
	dw_spi_write_n_bytes( subreg_len, p_data, DW_SPI_TRANSFER_DONE);
}

/**
 * \brief Takes several arrays and writes them as a single one to the device.
 * This can be used to write nested data frame structures easily.
 *
 * The following pattern is used in the demo ranging application.
 * \code
 *     uint32_t   n_data_segments = 2;
 *     uint32_t   data_len[2]     = { DW_FRAME_RANGE_LEN, DW_MSG_RANGE_INIT_LEN };
 *     uint8_t  * pp_data[2];
 *     pp_data[0] = (uint8_t *)&frameRange;
 *     pp_data[1] = (uint8_t *)&msgRangeInit;
 *     dw_transmit_multiple_data( pp_data, data_len, n_data_segments, DW_TRANCEIVE_SYNC );
 * \endcode
 * 
 * \param[in] reg_addr 			Destination register on dw1000.
 * \param[in] reg_len 			Length of register on dw1000.
 * \param[in] pp_data 			Array of data segments.
 * \param[in] p_data_len 		Length of each data segment.
 * \param[in] len_pp_data 		Length of pp_data.
 */
void 
dw_write_reg_multiple_data( uint32_t    reg_addr, 
							uint32_t    reg_len, 
							uint8_t  ** pp_data, 
							uint32_t *  p_data_len, 
							uint32_t    len_pp_data )
{
	// Get total length of data.
	uint32_t data_len = 0;
	uint32_t length = len_pp_data;
	while (length-- ) {data_len += *p_data_len++;}
	p_data_len-=len_pp_data;

	// Bounds check
	if (data_len > reg_len) {return;}

	// Transfer data to dw1000
	uint8_t instruction = 0x80 | (reg_addr & 0x3F);
	dw_spi_write_byte(instruction, DW_SPI_TRANSFER_CONT);
	dw_spi_read_byte();

	int i_transaction;
	for (i_transaction = 0; i_transaction < len_pp_data-1; ++i_transaction)
	{
		dw_spi_write_n_bytes( *p_data_len++, *pp_data++, DW_SPI_TRANSFER_CONT);
	}
	dw_spi_write_n_bytes( *p_data_len, *pp_data, DW_SPI_TRANSFER_DONE);
}

/**
 * \brief Reads a value from the one time programmable memory.
 * 
 * \param [in] otp_addr The address to read data from.
 * 
 * \return Contents of the otp memory location.
 */
uint32_t
dw_read_otp_32( uint16_t otp_addr )
{
	static const uint8_t cmd[] = {	DW_OTPRDEN_MASK||DW_OTPREAD_MASK, // Enable manual read
									DW_OTPREAD_MASK,                  // Do the acutal read
									0x00                              // Reset otp_ctrl
								 };

	uint32_t read_data = 0;
	dw_write_subreg(DW_REG_OTP_IF  , DW_SUBREG_OTP_ADDR, DW_SUBLEN_OTP_ADDR, (uint8_t *)&otp_addr);
	dw_write_subreg(DW_REG_OTP_IF  , DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[0]);
	dw_write_subreg(DW_REG_OTP_IF  , DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[1]);
	read_data = dw_read_subreg_32(DW_REG_OTP_IF, DW_SUBREG_OTP_RDAT, DW_SUBLEN_OTP_RDAT);
	dw_write_subreg(DW_REG_OTP_IF  , DW_SUBREG_OTP_CTRL, 1, (uint8_t *)&cmd[2]);

	return read_data;
}

/** @}*/

/*===========================================================================*/
/*===========================================================================*/
/** 
 * \addtogroup rxtx RX/TX
 *
 * \brief Functions to perform message reception and transmission.
 *
 * \details Requires that the device has been configured before hand using \ref
 * dw_conf_rx or \ref dw_conf_tx.
 * 
 * @{
 */

/**
 * \brief Receive a frame either asyncronously or syncronoulsy. The DW1000
 * must be configured with function dw_write_tx_conf( data_len, tx_conf ) before
 * running dw_transmit().
 *
 * \param receive_type 	[in] 	Specifies whether to use asyncronous or 
 * 								syncronous communication.
 * 
 */
void
dw_receive( dw1000_tranceive_t receive_type )
{
	// TODO: Fast receive / transmit

	if (   dw1000.state == DW_STATE_RECEIVING
		|| dw1000.state == DW_STATE_TRANSMITTING )
	{
		printf("dw1000 error: already using antenna.\n");
		return;
	}

	//  Start reception
	dw_init_rx();

	const uint32_t wait_mask_lo = DW_RXDFR_MASK
							| DW_RXPHE_MASK
							| DW_RXRFTO_MASK
							| DW_RXPTO_MASK
							| DW_RXSFDTO_MASK
							| DW_RXRFSL_MASK;
	uint64_t status_reg;
	uint64_t has_received;
	switch ( receive_type )
	{
		case DW_TRANCEIVE_ASYNC:
			dw_hal_enable_interrupt();
			break;

		case DW_TRANCEIVE_SYNC:
			do
			{
				// Wait until data received
				status_reg   = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS );
				has_received  = status_reg & wait_mask_lo;
				has_received |= (status_reg>>31>>1) & DW_RXPREJ_MASK;
			}
			while( !has_received );
			
			dw_get_rx_buffer();
			break;
	}
	return;
}

/**
 * \brief Transmit a frame either syncronously or asyncronously. The DW1000
 * must be configured with function dw_write_tx_conf( data_len, tx_conf ) before
 * running dw_transmit().
 * 
 * \param[in] p_data 			Pointer to data that is to be transmitted.
 * \param[in] data_len 		 	Length of data pointed to by p_data.
 * \param[in] transmit_type 	Specifies whether to use asyncronous or 
 * 								syncronous communication.
 */
void dw_transmit( uint8_t * p_data, uint32_t data_len, dw1000_tranceive_t transmit_type )
{
	// TODO: Functionality can be separated so that this function only triggers 
	// a transmission, akin to how reception works. You would then have a 
	// singluar dw_transmit and two upload functions, dw_upload_data and 
	// dw_upload_multiple_data.
	if (   dw1000.state == DW_STATE_RECEIVING
		|| dw1000.state == DW_STATE_TRANSMITTING )
	{
		printf("dw1000 error: already using antenna.\n");
		return;
	}

	// Place data on DW1000
	if (data_len > 0 && data_len < 1024)
	{
		// Copy data to dw1000
		dw_write_reg( DW_REG_TX_BUFFER, data_len, p_data );
	}

	// Initiate transmission
	dw_init_tx();

	// Handle transmission complete
	uint64_t status_reg;
	uint64_t is_sending;
	switch( transmit_type )
	{
		case DW_TRANCEIVE_ASYNC:
			dw_hal_enable_interrupt();
			break;

		case DW_TRANCEIVE_SYNC:
			do
			{
				// Wait until data sent
				status_reg = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS );
				is_sending = !( status_reg & (DW_TXFRS_MASK) );
			}
			while( is_sending );

			dw1000.state = DW_STATE_IDLE;
			break;
	}
	return;
}

/**
 * \brief Transmit a single data frame spread out over several arrays.
 * Loads all data arrays into dw1000 memory before starting the transmission.
 * 
 * \param[in] pp_data 		 	An array of arrays to be transmitted as a
 * 								single frame.
 * \param[in] p_data_len 		Length of each element in pp_array.
 * \param[in] length 		 	Length of the pp_data array.
 * \param[in] transmit_type  	Signifies asyncronous or syncronous
 * 								communication.
 */
void
dw_transmit_multiple_data( 	uint8_t  ** pp_data,
							uint32_t *  p_data_len,
							uint32_t    length,
							dw1000_tranceive_t transmit_type )
{
	if (   dw1000.state == DW_STATE_RECEIVING
		|| dw1000.state == DW_STATE_TRANSMITTING )
	{
		printf("dw1000 error: already using antenna.\n");
		return;
	}

	// Copy data to dw1000
	dw_write_reg_multiple_data( DW_REG_TX_BUFFER, DW_LEN_TX_BUFFER, pp_data, p_data_len, length );

	dw_transmit( NULL, 0, transmit_type );
}
/** @}*/

/*===========================================================================*/
/*===========================================================================*/
/** 
 * \addtogroup utility Utility
 *
 * \brief Utility functions here to simplify your life.
 * 
 *  @{
 */

/**
 * \brief Generates a sequence number for use with a new transmission.
 * \return A new sequence number (unique mod 256).
 */
uint8_t
dw_get_seq_no()
{
	static uint8_t seq_no = 0;
	return seq_no++;
}

/**
 * \brief Converts from floating point milliseconds to device time ticks.
 * \param[in]  t 	 	Time in milliseconds (ms).
 * \return Time in device clock ticks (~15.65 ps per tick).
 */
uint64_t
dw_ms_to_device_time( float t )
{
	return (uint64_t)(t * DW_MS_TO_DEVICE_TIME_SCALE);
}

/**
 * \brief Get the component unique id.
 * \return Component unique id.
 */
uint32_t
dw_get_device_id(void)
{
	static uint64_t device_id = 0x0ULL;
	if (device_id == 0x0ULL)
	{
		device_id = dw_read_reg_64(DW_REG_DEV_ID, DW_LEN_DEV_ID);
	}
	return device_id;
}

/**
 * \brief Returns the current system clock of the dw1000.
 * \return Current system clock time.
 */
uint64_t
dw_get_device_time()
{
	return dw_read_reg_64( DW_REG_SYS_TIME, DW_LEN_SYS_TIME );
}

/** @}*/

/*===========================================================================*/
/*===========================================================================*/
/** 
 * \addtogroup adc ADC
 *
 * \brief Control the onboard ADC and sensors.
 * 
 *  @{
 */

/**
 * \brief Enables power to the ADC circuitry. The power must be enabled before
 * any additional adc operations can be performed.
 */
void dw_enable_adc()
{
	uint32_t pmsc_val = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0);
	pmsc_val |= DW_ADCCE_MASK;
	dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, (uint8_t *)&pmsc_val);
}

/**
 * \brief Disables power to the ADC circuitry.
 */
void dw_disable_adc()
{
	uint32_t pmsc_val = dw_read_subreg_32(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0);
	pmsc_val &= ~DW_ADCCE_MASK;
	dw_write_subreg(DW_REG_PMSC, DW_SUBREG_PMSC_CTRL0, DW_SUBLEN_PMSC_CTRL0, (uint8_t *)&pmsc_val);
}

/**
 * \brief Private function. Forces the ADC to update sensor samples.
 *
 * \bug Seems like the values of tc_sarl are either not updated or updated 
 * incorrectly. See DW1000-User_Manual-V2.00.pdf page 56 - "Measuring IC 
 * temperature and voltage" for details on how sampling is performed.
 *
 * \todo Make private in documentation.
 */
void dw_adc_sample()
{
	// Make sure adc clock is enabled
	dw_enable_adc();

	// Undocumented procedure to take a sample
	uint8_t val;
	val = 0x80;
	dw_write_subreg(0x28, 0x11, 1, &val);
	val = 0x0A;
	dw_write_subreg(0x28, 0x12, 1, &val);
	val = 0x0F;
	dw_write_subreg(0x28, 0x12, 1, &val);

	// Take sample.
	// Wait for reading to complete.
	// Disable sampling
	uint8_t tc_sarc_val;
	tc_sarc_val = DW_SAR_CTRL_MASK;
	dw_write_subreg(DW_REG_TX_CAL, DW_SUBREG_TC_SARC, 1, &tc_sarc_val);
	udelay(200);
	tc_sarc_val = 0;
	dw_write_subreg(DW_REG_TX_CAL, DW_SUBREG_TC_SARC, 1, &tc_sarc_val);

	return;
}

/**
 * \brief Gets a temperature reading from the dw1000.
 *
 * \param[in] temp_source 	 	If given as DW_ADC_SRC_LATEST a new senors 
 *                     			sample will be taken and reported.
 *                    			If given as DW_ADC_SRC_WAKEUP the reading from
 *                    			the last wakeup will be used.
 * 
 * \return Temperature measurement from adc
 *
 * \bug The values generated by these functions are not to be trusted! There 
 * seems to be an error in the \ref dw_adc_sample function.
 */
float
dw_get_temperature( dw_adc_src_t temp_source )
{
	// Get calibration data from otp. Tmeas @ 23 degrees resides in addr 0x9.
	uint32_t otp_temp = dw_read_otp_32(0x009) & 0xFF;
	uint32_t read_temp;

	// Load to CPU sample
	switch (temp_source)
	{
		case DW_ADC_SRC_LATEST:
			dw_adc_sample();
			read_temp   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARL, DW_SUBLEN_TC_SARL);
			read_temp  &= DW_SAR_LTEMP_MASK;
			read_temp >>= DW_SAR_LTEMP;
			break;

		case DW_ADC_SRC_WAKEUP:
			read_temp   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARW, DW_SUBLEN_TC_SARW);
			read_temp  &= DW_SAR_WTEMP_MASK;
			read_temp >>= DW_SAR_WTEMP;
			break;
	}

	return ((float)read_temp - (float)otp_temp)*1.14f + 23.f;
}

/**
 * \brief Gets a voltage reading from the dw1000.
 *
 * \param[in] voltage_source 	If given as DW_ADC_SRC_LATEST a new senors 
 *                     			sample will be taken and reported.
 *                    			If given as DW_ADC_SRC_WAKEUP the reading from
 *                    			the last wakeup will be used.
 * 
 * NOTE: The effective range of measurement is 2.25 V to 3.76 V.
 * 
 * \return Voltage measurement from adc
 * 
 * \bug The values generated by these functions are not to be trusted! There 
 * seems to be an error in the \ref dw_adc_sample function.
 */
float
dw_get_voltage( dw_adc_src_t voltage_source )
{
	// Get calibration data from otp. Vmeas @ 3.3V residies in addr 0x8.
	uint32_t otp_voltage = dw_read_otp_32(0x008) & 0xFF;
	uint32_t read_voltage;
	
	switch (voltage_source)
	{
		case DW_ADC_SRC_LATEST:
			dw_adc_sample();
			read_voltage   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARL, DW_SUBLEN_TC_SARL);
			read_voltage  &= DW_SAR_LVBAT_MASK;
			read_voltage >>= DW_SAR_LVBAT;
			break;

		case DW_ADC_SRC_WAKEUP:
			read_voltage   = dw_read_subreg_32(DW_REG_TX_CAL, DW_SUBREG_TC_SARW, DW_SUBLEN_TC_SARW);
			read_voltage  &= DW_SAR_WVBAT_MASK;
			read_voltage >>= DW_SAR_WVBAT;
			break;
	}

	return ((float)read_voltage - (float)otp_voltage)/173.f + 3.3f;
}

/** @}*/

/*===========================================================================*/
/*===========================================================================*/
/** 
 * \addtogroup quality Signal Quality Estimation
 *
 * \brief Estimate the received signal qualtity.
 * 
 *  @{
 */

/**
 * \brief Gives a measure of the standard deviation of the noise level in the 
 * data in the Rx Frame Quality Information register (\ref DW_REG_RX_FQUAL). 
 * Can  be used as an absolute value or compared to the value reported in the 
 * \ref DW_FP_AMPL2 field of the Rx Frame Quality Information register. A large 
 * noise value is generally bad. If the noise value is larger than the value 
 * in FP_AMPL2 the quality is quite possibly bad.
 * 
 * \return Noise level of measurement.
 */
float
dw_get_noise_level()
{
	return (float)((dw_read_reg_64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL) & (DW_STD_NOISE_MASK)) >> DW_STD_NOISE);
}

/**
 * \brief Returns the estimated receive signal amplitude in the first path. 
 * Used to calculate the estimated power in the first path.
 * \return Amplitude in first path.
 */
float
dw_get_fp_ampl()
{
	return (float)((dw_read_reg_64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL) & (DW_FP_AMPL2_MASK)) >> DW_FP_AMPL2);
}

/**
 * \brief Estimate total power received in all paths.
 * 
 * \note The function used to calculate this requires a logarithm. Thus this 
 * value needs to be post processed. Use 10 * log_10( dw_get_rx_power() ) - a 
 * where a is a constant 115.72 for 16 MHZ PRF and 121.74 for 64 MHZ PRF.
 */
float
dw_get_rx_power()
{
	uint64_t rx_fqual_val = dw_read_reg_64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL);
	uint32_t rx_finfo_val = dw_read_reg_32(DW_REG_RX_FINFO, DW_LEN_RX_FINFO);
	float c = (rx_fqual_val & (DW_CIR_PWR_MASK)) >> DW_CIR_PWR;
	float n = (rx_finfo_val & (DW_RXPACC_MASK)) >> DW_RXPACC;
	float a;
	float rx_power;

	switch (dw1000.conf.prf)
	{
		case DW_PRF_16_MHZ: a = 115.72; break;
		case DW_PRF_64_MHZ: a = 121.74; break;
	}

	// If you have access to logarithm...
	//rx_power = 10.f * log10( (float)(c * powf(2,17)) / (float)(n*n) ) - a;
	// This value needs external processing 
	rx_power = (float)(c * powf(2,17)) / (float)(n*n);
	return rx_power;
}
/**
 * \brief Calculates the estimated signal power in the first path.
 * \return Estimated reception signal power in the first path. [dBmW]
 * 
 * \note The function used to calculate this requires a logarithm. Thus this 
 * value needs to be post processed. Use 10 * log_10( dw_get_rx_power() ) - a 
 * where a is a constant 115.72 for 16 MHZ PRF and 121.74 for 64 MHZ PRF.
 */
float dw_get_fp_power()
{
	uint64_t rx_fqual_val = dw_read_reg_64(DW_REG_RX_FQUAL, DW_LEN_RX_FQUAL);
	uint32_t rx_finfo_val = dw_read_reg_32(DW_REG_RX_FINFO, DW_LEN_RX_FINFO);
	// Special way to read fp_ampl1, not following ordinary definitions.
	uint32_t fp_ampl1_val = dw_read_subreg_32(DW_REG_RX_TIME, 0x7, 0x2);

	float fp_ampl1 = (float)fp_ampl1_val;
	float fp_ampl2 = (float)((rx_fqual_val & (DW_FP_AMPL2_MASK)) >> DW_FP_AMPL2);
	float fp_ampl3 = (float)((rx_fqual_val & (DW_FP_AMPL3_MASK)) >> DW_FP_AMPL3);
	float n = (float)((rx_finfo_val & (DW_RXPACC_MASK)) >> DW_RXPACC);
	float a;
	float fp_power;

	switch (dw1000.conf.prf)
	{
		case DW_PRF_16_MHZ: a = 115.72; break;
		case DW_PRF_64_MHZ: a = 121.74; break;
	}

	float fp_ampl1_2 = (float) (fp_ampl1 * fp_ampl1);
	float fp_ampl2_2 = (float) (fp_ampl2 * fp_ampl2);
	float fp_ampl3_2 = (float) (fp_ampl3 * fp_ampl3);
	float n_2 = (float) (n * n);

	// Use this if you have math lib.
	//fp_power = 10 * log10( (fp_ampl1_2+fp_ampl2_2+fp_ampl3_2)/(n_2) ) - a;
	//Else, we compute logarithm externally.
	fp_power = (fp_ampl1_2+fp_ampl2_2+fp_ampl3_2)/(n_2);
	return fp_power;
}
/** @}*/

/*===========================================================================*/
/*===========================================================================*/
/** 
 * \addtogroup rxtx
 *  @{
 */

/**
 * \brief Fetches the rx buffer from the DW1000 and parses any reception
 * errors, setting the updating the state accordingly. Afterwards the buffers
 * can be accessed through the global dw1000_base_driver variable dw1000.
 *
 * \details Typically this function is called in the interrupt handler if the
 * detected event was a reception. After the call dw1000.p_receive_buffer will
 * mirror the data contained in the dw1000 register RX_BUFFER.
 */
void
dw_get_rx_buffer(void)
{
	uint32_t * status_reg;
	uint64_t status_reg_64;
	uint32_t isError;
	uint32_t rx_frame_info_reg;
	uint32_t rx_len;

	// Get error status
	// TODO: Break out into separate function.
	const uint32_t error_mask_lo = DW_RXPHE_MASK | DW_RXRFTO_MASK | DW_RXPTO_MASK | DW_RXSFDTO_MASK | DW_RXRFSL_MASK;
	const uint32_t error_mask_hi = DW_RXPREJ_MASK;
	status_reg_64 = dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS );
	status_reg = (uint32_t *)&status_reg_64;
	isError  = *(status_reg+0) & error_mask_lo;
	isError |= *(status_reg+1) & error_mask_hi;

	if (isError) dw1000.state = DW_STATE_ERROR;
	else         dw1000.state = DW_STATE_IDLE;

	// Get length of received frame
	rx_frame_info_reg = dw_read_reg_32( DW_REG_RX_FINFO, DW_LEN_RX_FINFO );
	rx_len = rx_frame_info_reg & (DW_RXFLEN_MASK|DW_RXFLE_MASK);
	rx_len = (rx_len < DW_RX_BUFFER_MAX_LEN) ? (rx_len) : (DW_RX_BUFFER_MAX_LEN);

	// Store rx data in global variable rxBuffer
	dw1000.receive_buffer_len = rx_len;
	
	if (rx_len > 0)
	{
		// Store rx data in global variable rxBuffer
		dw_read_reg( DW_REG_RX_BUFFER, rx_len, dw1000.p_receive_buffer );
	}

	// Cleanup
	dw_clear_pending_interrupt(DW_RXDFR_MASK | DW_RXRFTO_MASK);
}

/**
 * \brief Sets the DW1000 rx timeout interval. If no preamble has been
 * 			discovered in this time the event flag RXRFTO is set.
 * \param[in] us 	 	Timeout period in microseconds (~1.026 us per tick).
 */	
void dw_set_rx_timeout( uint16_t us )
{
	dw_write_reg( DW_REG_RX_FWTO, DW_LEN_RX_FWTO, (uint8_t *)&us );
}

/**
 * \brief Read the current timeout period from the DW1000.
 * \return The current timeout period in microseconds (~1.026 us per tick).
 */
uint16_t
dw_get_rx_timeout()
{
	return dw_read_reg_32( DW_REG_RX_FWTO, DW_LEN_RX_FWTO );
}

/**
 * \brief Enables rx timeout. After the period set in register RX_FWTO the bit
 * 			RXRFTO will be set in register SYS_STATUS and the reception will be
 * 			aborted.
 */
void
dw_enable_rx_timeout()
{
	uint32_t cfgReg;
	cfgReg  = dw_read_reg_32( DW_REG_SYS_CFG, DW_LEN_SYS_CFG );
	cfgReg |= DW_RXWTOE_MASK;
	dw_write_reg( DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&cfgReg);
}

/**
 * \brief Disables rx timeout.
 */
void
dw_disable_rx_timeout()
{
	uint32_t cfgReg;
	cfgReg  = dw_read_reg_32( DW_REG_SYS_CFG, DW_LEN_SYS_CFG );
	cfgReg &= ~DW_RXWTOE_MASK;
	dw_write_reg( DW_REG_SYS_CFG, DW_LEN_SYS_CFG, (uint8_t *)&cfgReg);
	printf("CFG: %x\n", cfgReg);
}

/**
 * \brief Gets the timestamp for the latest received frame.
 */
uint64_t
dw_get_rx_timestamp()
{
	return dw_read_reg_64(DW_REG_RX_TIME, 8) & 0x000000FFFFFFFFFFULL;
}

/**
 * \brief Gets the timestamp for the latest transmitted frame.
 */
uint64_t
dw_get_tx_timestamp()
{
	return dw_read_reg_64(DW_REG_TX_TSTAMP, 8) & 0x000000FFFFFFFFFFULL;
}

/**
 * \brief Specifies the antenna delay used to calculate the tx and rx
 * timestamps (~15.65 ps per tick).
 *
 * \details This function assumes there is an equal transmit and receive delay.
 */
void
dw_set_antenna_delay( uint16_t delay )
{
	dw_write_subreg(DW_REG_LDE_IF, DW_SUBREG_LDE_RXANTD, DW_SUBLEN_LDE_RXANTD, (uint8_t *)&delay);
	dw_write_reg(DW_REG_TX_ANTD, DW_LEN_TX_ANTD, (uint8_t *)&delay);
}

/**
 * Get the current antenna delay. (~15.65 ps per tick)
 */
uint16_t
dw_get_antenna_delay()
{
	return dw_read_reg_32(DW_REG_TX_ANTD, DW_LEN_TX_ANTD);
}

/**
 * \brief Setter for the delayed transmit/receive register. If delayed operation 
 * is enabled the transmission/receeption will not take place until the system
 * time has exceeded this value.
 *
 * \note The low order nine bits are ignored. Thus, when working with 
 * dx_timestamps the macro \ref DX_TIMESTAMP_CLEAR_LOW_9 can be quite helpful.
 */
void
dw_set_dx_timestamp( uint64_t timestamp )
{
	dw_write_reg( DW_REG_DX_TIME, DW_LEN_DX_TIME, (uint8_t *)&timestamp );
}

/**
 * \brief Getter for the delayed transmit/receive register.
 */
uint64_t
dw_get_dx_timestamp()
{
	return dw_read_reg_64(DW_REG_DX_TIME, DW_LEN_DX_TIME) & 0x000000FFFFFFFFFFULL;
}

/** @}*/

/**
 * \brief Enables interrupts as specified in parameter mask. Previous values in
 * SYS_MASK are overwritten. Example usage:
 * \code
 * dw_enable_interrupt( DW_MTXFRS_MASK | DW_MRXPHE_MASK | DW_MRXDFR_MASK );
 * \endcode
 * The interrupt constants can be found in dw1000-base.h under BITFIELDS for 
 * DW_REG_SYS_MASK.
 * 
 * \param[in] 	mask 	Value to overwrite SYS_MASK register with.
 */
void
dw_enable_interrupt( uint32_t mask )
{
	dw_write_reg( DW_REG_SYS_MASK, DW_LEN_SYS_MASK, (uint8_t *)&mask );
}

/**
 * \brief Clear a pending masked interrupt. Usage same as \ref 
 * dw_enable_interrupt.
 */
void
dw_clear_pending_interrupt( uint64_t mask )
{
	dw_write_reg( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS, (uint8_t *)&mask);
}

/*-----------------------------------------------------------------------------
   Private functions
-----------------------------------------------------------------------------*/

/**
 * \brief Aborts current transimission or reception and returns device to idle.
 */
void
dw_trxoff()
{
	uint32_t sys_ctrl_val = dw_read_reg_32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
	sys_ctrl_val |= (1<<DW_TRXOFF) & DW_TRXOFF_MASK;
	dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl_val);
}

/**
 * \brief Initiates a new reception on the dw1000. Assumes that it has been 
 * configured already.
 */
void
dw_init_rx(void)
{
	dw1000.state = DW_STATE_RECEIVING;
	// Enable antenna
	uint32_t sys_ctrl_val = dw_read_reg_32(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL);
	sys_ctrl_val |= (1<<DW_RXENAB) & DW_RXENAB_MASK;
	dw_write_reg(DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&sys_ctrl_val);
}

/**
 * \brief Starts a new transmission. Data must either already be uploaded to 
 * DW1000 or be uploaded VERY shortly.
 */
void
dw_init_tx(void)
{
	dw1000.state = DW_STATE_TRANSMITTING;

	// Start transmission
	uint32_t ctrl_reg_val;
	ctrl_reg_val  = dw_read_reg_32( DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL );
	ctrl_reg_val |= DW_TXSTRT_MASK;
	dw_write_reg( DW_REG_SYS_CTRL, DW_LEN_SYS_CTRL, (uint8_t *)&ctrl_reg_val);
}
