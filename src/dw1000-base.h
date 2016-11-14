#ifndef DW1000_BASE_H
#define DW1000_BASE_H

/**
 *  \file dw1000-base.h
 *  \author Kim Albertsson
 *  \date 2014-Oct-16
 *  
 *  \brief Interface for communicating with the Decawave dw1000 chipset.
 *  Defines a number of macros specifying all the registers of the dw1000, 
 *  their length in bytes, sub register contained in that register if any and
 *  all bit named bits in said registers. All names and sizes etc. are taken 
 *  from the Decawave dw1000 User Manual v2.00.
 *
 * \note The register map and named bit list is as of yet incomplete.
 *
 * \note Assumes only one instance of the dw1000 to communicate to.
 */

/*===========================================================================*/
/*================================ Includes =================================*/

#include <stdint.h> // For fixed size integer uint32_t etc.

/*===========================================================================*/
/*================================ Defines ==================================*/

#define DW_REG_DEV_ID   0x00 /**< \brief Register Device Identifier, address */
#define DW_LEN_DEV_ID   4    /**< \brief Register Device Identifier, length  */

#define DW_REG_EID      0x01 /**< \brief Register Extended Unique Identifier, address */
#define DW_LEN_EID      8    /**< \brief Register Extended Unique Identifier, length  */

#define DW_REG_PANADR   0x03 /**< \brief Register PAN Identifier and Short Address, address */
#define DW_LEN_PANADR   4    /**< \brief Register PAN Identifier and Short Address, length  */

#define DW_REG_SYS_CFG  0x04 /**< \brief Register System Configuration bitmap, address */
#define DW_LEN_SYS_CFG  4    /**< \brief Register System Configuration bitmap, length  */

#define DW_REG_SYS_TIME 0x06 /**< \brief dummy */
#define DW_LEN_SYS_TIME 5    /**< \brief dummy */

#define DW_REG_TX_FCTRL 0x08 /**< \brief dummy */
#define DW_LEN_TX_FCTRL 5    /**< \brief dummy */
#define DW_SUBREG_TX_FCTRL_LOW  0x00 /**< \brief dummy */
#define DW_SUBLEN_TX_FCTRL_LOW  4    /**< \brief dummy */
#define DW_SUBREG_TX_FCTRL_HIGH 0x04 /**< \brief dummy */
#define DW_SUBLEN_TX_FCTRL_HIGH 4    /**< \brief dummy */

#define DW_REG_TX_BUFFER 0x09 /**< \brief dummy */
#define DW_LEN_TX_BUFFER 1024 /**< \brief dummy */

#define DW_REG_DX_TIME 0x0A /**< \brief dummy */
#define DW_LEN_DX_TIME 5    /**< \brief dummy */

#define DW_REG_RX_FWTO 0x0C /**< \brief dummy */
#define DW_LEN_RX_FWTO 2    /**< \brief dummy */

#define DW_REG_SYS_CTRL 0x0D /**< \brief dummy */
#define DW_LEN_SYS_CTRL 4    /**< \brief dummy */

#define DW_REG_SYS_MASK 0x0E /**< \brief dummy */
#define DW_LEN_SYS_MASK 4    /**< \brief dummy */

#define DW_REG_SYS_STATUS 0x0F /**< \brief dummy */
#define DW_LEN_SYS_STATUS 5    /**< \brief dummy */

#define DW_REG_RX_FINFO 0x10 /**< \brief dummy */
#define DW_LEN_RX_FINFO 4    /**< \brief dummy */

#define DW_REG_RX_BUFFER 0x11 /**< \brief dummy */
#define DW_LEN_RX_BUFFER 1024 /**< \brief dummy */

#define DW_REG_RX_FQUAL 0x12 /**< \brief dummy */
#define DW_LEN_RX_FQUAL 8    /**< \brief dummy */

#define DW_REG_TX_POWER 0x1E /**< \brief dummy */
#define DW_LEN_TX_POWER 4    /**< \brief dummy */

#define DW_REG_CHAN_CTRL 0x1F /**< \brief dummy */
#define DW_LEN_CHAN_CTRL 4    /**< \brief dummy */

#define DW_REG_AGC_CTRL 0x23 /**< \brief dummy */
#define DW_LEN_AGC_CTRL 32   /**< \brief dummy */
#define DW_SUBREG_AGC_CTRL1 0x02 /**< \brief dummy */
#define DW_SUBLEN_AGC_CTRL1 2    /**< \brief dummy */
#define DW_SUBREG_AGC_TUNE1 0x04 /**< \brief dummy */
#define DW_SUBLEN_AGC_TUNE1 2    /**< \brief dummy */
#define DW_SUBREG_AGC_TUNE2 0x0C /**< \brief dummy */
#define DW_SUBLEN_AGC_TUNE2 4    /**< \brief dummy */
#define DW_SUBREG_AGC_TUNE3 0x12 /**< \brief dummy */
#define DW_SUBLEN_AGC_TUNE3 2    /**< \brief dummy */

#define DW_REG_RX_TIME 0x15 /**< \brief dummy */
#define DW_LEN_RX_TIME 14   /**< \brief dummy */

#define DW_REG_TX_TSTAMP 0x17 /**< \brief dummy */
#define DW_LEN_TX_TSTAMP 10   /**< \brief dummy */

#define DW_REG_TX_ANTD 0x18 /**< \brief dummy */
#define DW_LEN_TX_ANTD 2    /**< \brief dummy */

#define DW_REG_DRX_CONF      0x27 /**< \brief dummy */
#define DW_SUBREG_DRX_TUNE0b 0x02 /**< \brief dummy */
#define DW_SUBLEN_DRX_TUNE0b 2    /**< \brief dummy */
#define DW_SUBREG_DRX_TUNE1a 0x04 /**< \brief dummy */
#define DW_SUBLEN_DRX_TUNE1a 2    /**< \brief dummy */
#define DW_SUBREG_DRX_TUNE1b 0x06 /**< \brief dummy */
#define DW_SUBLEN_DRX_TUNE1b 2    /**< \brief dummy */
#define DW_SUBREG_DRX_TUNE2  0x08 /**< \brief dummy */
#define DW_SUBLEN_DRX_TUNE2  4    /**< \brief dummy */
#define DW_SUBREG_DRX_TUNE4h 0x26 /**< \brief dummy */
#define DW_SUBLEN_DRX_TUNE4h 2    /**< \brief dummy */

#define DW_REG_RF_CONF 0x28 /**< \brief dummy */
#define DW_SUBREG_RF_RXCTRLH 0x0B /**< \brief dummy */
#define DW_SUBLEN_RF_RXCTRLH 1    /**< \brief dummy */
#define DW_SUBREG_RF_TXCTRL  0x0C /**< \brief dummy */
#define DW_SUBLEN_RF_TXCTRL  4    /**< \brief dummy */

#define DW_REG_TX_CAL 0x2A /**< \brief dummy */
#define DW_SUBREG_TC_SARC    0x00 /**< \brief dummy */
#define DW_SUBLEN_TC_SARC    3    /**< \brief dummy */
#define DW_SUBREG_TC_SARL    0x03 /**< \brief dummy */
#define DW_SUBLEN_TC_SARL    3    /**< \brief dummy */
#define DW_SUBREG_TC_SARW    0x06 /**< \brief dummy */
#define DW_SUBLEN_TC_SARW    2    /**< \brief dummy */
#define DW_SUBREG_TC_PGDELAY 0x0B /**< \brief dummy */
#define DW_SUBLEN_TC_PGDELAY 1    /**< \brief dummy */

#define DW_REG_FS_CTRL 0x2B /**< \brief dummy */
#define DW_SUBREG_FS_PLLCFG  0x07 /**< \brief dummy */
#define DW_SUBLEN_FS_PLLCFG  4    /**< \brief dummy */
#define DW_SUBREG_FS_PLLTUNE 0x0B /**< \brief dummy */
#define DW_SUBLEN_FS_PLLTUNE 1    /**< \brief dummy */

#define DW_REG_LDE_IF        0x2E /**< \brief dummy */
#define DW_SUBREG_LDE_RXANTD 0x1804 /**< \brief dummy */
#define DW_SUBLEN_LDE_RXANTD 2      /**< \brief dummy */
#define DW_SUBREG_LDE_CFG2   0x1806 /**< \brief dummy */
#define DW_SUBLEN_LDE_CFG2   2      /**< \brief dummy */

#define DW_REG_OTP_IF 0x2D /**< \brief dummy */
#define DW_SUBREG_OTP_WDAT  0x00 /**< \brief dummy */
#define DW_SUBLEN_OTP_WDAT  4    /**< \brief dummy */
#define DW_SUBREG_OTP_ADDR  0x04 /**< \brief dummy */
#define DW_SUBLEN_OTP_ADDR  2    /**< \brief dummy */
#define DW_SUBREG_OTP_CTRL  0x06 /**< \brief dummy */
#define DW_SUBLEN_OTP_CTRL  2    /**< \brief dummy */
#define DW_SUBREG_OTP_STAT  0x08 /**< \brief dummy */
#define DW_SUBLEN_OTP_STAT  2    /**< \brief dummy */
#define DW_SUBREG_OTP_RDAT  0x0A /**< \brief dummy */
#define DW_SUBLEN_OTP_RDAT  4    /**< \brief dummy */
#define DW_SUBREG_OTP_SRDAT 0x0E /**< \brief dummy */
#define DW_SUBLEN_OTP_SRDAT 4    /**< \brief dummy */
#define DW_SUBREG_OTP_SF    0x12 /**< \brief dummy */
#define DW_SUBLEN_OTP_SF    1    /**< \brief dummy */

#define DW_REG_PMSC 0x36 /**< \brief dummy */
#define DW_SUBREG_PMSC_CTRL0 0x00 /**< \brief dummy */
#define DW_SUBLEN_PMSC_CTRL0 0x04 /**< \brief dummy */

/*=================================================
=========== Bitfields
=================================================*/

/* DW_REG_SYS_CFG 0x04 */
#define DW_RXM110K      22        /**< \brief dummy */
#define DW_RXM110K_MASK (0x1<<22) /**< \brief dummy */
#define DW_RXWTOE       28        /**< \brief dummy */
#define DW_RXWTOE_MASK  (0x1<<28) /**< \brief dummy */

/* DW_REG_TX_FCTRL 0x08 */
/* DW_SUBREG_TX_FCTRL_LOW 0x00 */
#define DW_TXLEN         0         /**< \brief dummy */
#define DW_TXLEN_MASK    (0x7F<<0) /**< \brief dummy */
#define DW_TXLE          7         /**< \brief dummy */
#define DW_TXLE_MASK     (0x07<<7) /**< \brief dummy */
#define DW_TXBR          13        /**< \brief dummy */
#define DW_TXBR_MASK     (0x3<<13) /**< \brief dummy */
#define DW_TR            15        /**< \brief dummy */
#define DW_TR_MASK       (0x1<<15) /**< \brief dummy */
#define DW_TXPRF         16        /**< \brief dummy */
#define DW_TXPRF_MASK    (0x3<<16) /**< \brief dummy */
#define DW_TXPSR         18        /**< \brief dummy */
#define DW_TXPSR_MASK    (0x3<<18) /**< \brief dummy */
#define DW_PE            20        /**< \brief dummy */
#define DW_PE_MASK       (0x3<<20) /**< \brief dummy */
#define DW_TXBOFFS       22        /**< \brief dummy */
#define DW_TXBOFFS_MASK  (0x3FF<<22) /**< \brief dummy */
/* DW_SUBREG_TX_FCTRL_HIGH 0x04 */
#define DW_IFSDELAY      0         /**< \brief dummy */
#define DW_IFSDELAY_MASK (0xFF<<0) /**< \brief dummy */

/* DW_REG_SYS_CTRL 0x0D */
#define DW_TXSTRT       1      /**< \brief dummy */
#define DW_TXSTRT_MASK  (1<<1) /**< \brief dummy */
#define DW_TXDLYS       2      /**< \brief dummy */
#define DW_TXDLYS_MASK  (1<<2) /**< \brief dummy */
#define DW_TRXOFF       6      /**< \brief dummy */
#define DW_TRXOFF_MASK  (1<<6) /**< \brief dummy */
#define DW_RXENAB       8      /**< \brief dummy */
#define DW_RXENAB_MASK  (1<<8) /**< \brief dummy */
#define DW_RXDLYE       9      /**< \brief dummy */
#define DW_RXDLYE_MASK  (1<<9) /**< \brief dummy */
#define DW_RXPHE        12      /**< \brief dummy */
#define DW_RXPHE_MASK   (1<<12) /**< \brief dummy */
#define DW_RXRFSL       16      /**< \brief dummy */
#define DW_RXRFSL_MASK  (1<<16) /**< \brief dummy */
#define DW_RXPTO        21      /**< \brief dummy */
#define DW_RXPTO_MASK   (1<<21) /**< \brief dummy */
#define DW_RXSFDTO      26      /**< \brief dummy */
#define DW_RXSFDTO_MASK (1<<26) /**< \brief dummy */

/* DW_REG_SYS_MASK 0x0E */
#define DW_MTXFRS        7       /**< \brief dummy */
#define DW_MTXFRS_MASK   (1<<7)  /**< \brief dummy */
#define DW_MRXPHE        12      /**< \brief dummy */
#define DW_MRXPHE_MASK   (1<<12) /**< \brief dummy */
#define DW_MRXDFR        13      /**< \brief dummy */
#define DW_MRXDFR_MASK   (1<<13) /**< \brief dummy */
#define DW_MRXRFSL       16      /**< \brief dummy */
#define DW_MRXRFSL_MASK  (1<<16) /**< \brief dummy */
#define DW_MRXRFTO       17      /**< \brief dummy */
#define DW_MRXRFTO_MASK  (1<<17) /**< \brief dummy */
#define DW_MRXPTO        21      /**< \brief dummy */
#define DW_MRXPTO_MASK   (1<<21) /**< \brief dummy */
#define DW_MRXSFDTO      26      /**< \brief dummy */
#define DW_MRXSFDTO_MASK (1<<26) /**< \brief dummy */

/* DW_REG_SYS_STATUS LOW */
#define DW_IRQS        0       /**< \brief dummy */
#define DW_IRQS_MASK   (1<<0)  /**< \brief dummy */
#define DW_TXFRS       7       /**< \brief dummy */
#define DW_TXFRS_MASK  (1<<7)  /**< \brief dummy */
#define DW_RXDFR       13      /**< \brief dummy */
#define DW_RXDFR_MASK  (1<<13) /**< \brief dummy */
#define DW_RXRFTO      17      /**< \brief dummy */
#define DW_RXRFTO_MASK (1<<17) /**< \brief dummy */
/* DW_REG_SYS_STATUS HIGH */
#define DW_RXPREJ      1      /**< \brief dummy */
#define DW_RXPREJ_MASK (1<<1) /**< \brief dummy */
#define DW_TXPUTE      2      /**< \brief dummy */
#define DW_TXPUTE_MASK (1<<2) /**< \brief dummy */

/* DW_REG_RX_FINFO 0x10 */
#define DW_RXFLEN      0           /**< \brief dummy */
#define DW_RXFLEN_MASK (0x7F<<0)   /**< \brief dummy */
#define DW_RXFLE       7           /**< \brief dummy */
#define DW_RXFLE_MASK  (0x07<<7)   /**< \brief dummy */
#define DW_RXNSPL      11          /**< \brief dummy */
#define DW_RXNSPL_MASK (0x3<<11)   /**< \brief dummy */
#define DW_RXBR        13          /**< \brief dummy */
#define DW_RXBR_MASK   (0x3<<13)   /**< \brief dummy */
#define DW_RNG         15          /**< \brief dummy */
#define DW_RNG_MASK    (0x1<<15)   /**< \brief dummy */
#define DW_RXPRF       16          /**< \brief dummy */
#define DW_RXPRF_MASK  (0x3<<16)   /**< \brief dummy */
#define DW_RXPSR       18          /**< \brief dummy */
#define DW_RXPSR_MASK  (0x3<<18)   /**< \brief dummy */
#define DW_RXPACC      20          /**< \brief dummy */
#define DW_RXPACC_MASK (0xFFF<<20) /**< \brief dummy */

/* DW_REG_RX_FQUAL 0x12 */
#define DW_STD_NOISE      0               /**< \brief dummy */
#define DW_STD_NOISE_MASK (0xFFFFULL<<0)  /**< \brief dummy */
#define DW_FP_AMPL2       16              /**< \brief dummy */
#define DW_FP_AMPL2_MASK  (0xFFFFULL<<16) /**< \brief dummy */
#define DW_FP_AMPL3       32              /**< \brief dummy */
#define DW_FP_AMPL3_MASK  (0xFFFFULL<<32) /**< \brief dummy */
#define DW_CIR_PWR        48              /**< \brief dummy */
#define DW_CIR_PWR_MASK   (0xFFFFULL<<48) /**< \brief dummy */

/* DW_REG_CHAN_CTRL 0x1F */
#define DW_TXCHAN        0          /**< \brief dummy */
#define DW_TXCHAN_MASK   (0x1F<<0)  /**< \brief dummy */
#define DW_RXCHAN        5          /**< \brief dummy */
#define DW_RXCHAN_MASK   (0x1F<<5)  /**< \brief dummy */
#define DW_DWSFD         17         /**< \brief dummy */
#define DW_DWSFD_MASK    (1<<17)    /**< \brief dummy */
#define DW_RXPRFR        18         /**< \brief dummy */
#define DW_RXPRFR_MASK   (0x03<<18) /**< \brief dummy */
#define DW_TNSSFD        20         /**< \brief dummy */
#define DW_TNSSFD_MASK   (1<<20)    /**< \brief dummy */
#define DW_RNSSFD        21         /**< \brief dummy */
#define DW_RNSSFD_MASK   (1<<21)    /**< \brief dummy */
#define DW_TX_PCODE      22         /**< \brief dummy */
#define DW_TX_PCODE_MASK (0x3F<<22) /**< \brief dummy */
#define DW_RX_PCODE      27         /**< \brief dummy */
#define DW_RX_PCODE_MASK (0x3F<<27) /**< \brief dummy */

/* DW_REG_TX_CAL 0x2A */
/* DW_SUBREG_SARC 0x2A:00 */
#define DW_SAR_CTRL      0      /**< \brief dummy */
#define DW_SAR_CTRL_MASK (1<<0) /**< \brief dummy */
/* DW_SUBREG_SARL 0x2A:03 */
#define DW_SAR_LTEMP      0         /**< \brief dummy */
#define DW_SAR_LTEMP_MASK (0xFF<<0) /**< \brief dummy */
#define DW_SAR_LVBAT      8         /**< \brief dummy */
#define DW_SAR_LVBAT_MASK (0xFF<<8) /**< \brief dummy */
/* DW_SUBREG_SARW 0x2A:06 */
#define DW_SAR_WTEMP      0         /**< \brief dummy */
#define DW_SAR_WTEMP_MASK (0xFF<<0) /**< \brief dummy */
#define DW_SAR_WVBAT      8         /**< \brief dummy */
#define DW_SAR_WVBAT_MASK (0xFF<<8) /**< \brief dummy */

/* DW_REG_OTP_IF 0x2D */
/* DW_SUBREG_OTP_WDAT  0x2D:00 */
/* DW_SUBREG_OTP_ADDR  0x2D:04 */
#define DW_OTPADDR      0          /**< \brief dummy */
#define DW_OTPADDR_MASK (0x7FF<<0) /**< \brief dummy */
/* DW_SUBREG_OTP_CTRL  0x2D:06 */
#define DW_OTPRDEN      0        /**< \brief dummy */
#define DW_OTPRDEN_MASK (1<<0)   /**< \brief dummy */
#define DW_OTPREAD      1        /**< \brief dummy */
#define DW_OTPREAD_MASK (1<<1)   /**< \brief dummy */
#define DW_OTPMRWR      3        /**< \brief dummy */
#define DW_OTPMRWR_MASK (1<<3)   /**< \brief dummy */
#define DW_OTPPROG      6        /**< \brief dummy */
#define DW_OTPPROG_MASK (1<<6)   /**< \brief dummy */
#define DW_OTPMR        7        /**< \brief dummy */
#define DW_OTPMR_MASK   (0xF<<7) /**< \brief dummy */
#define DW_LDELOAD      15       /**< \brief dummy */
#define DW_LDELOAD_MASK (1<<15)  /**< \brief dummy */
/* DW_SUBREG_OTP_STAT  0x2D:08 */
#define DW_OTPPRGD      0      /**< \brief dummy */
#define DW_OTPPRGD_MASK (1<<0) /**< \brief dummy */
#define DW_OTPVPOK      1      /**< \brief dummy */
#define DW_OTPVPOK_MASK (1<<1) /**< \brief dummy */
/* DW_SUBREG_OTP_RDAT  0x2D:0A */
/* DW_SUBREG_OTP_SRDAT 0x2D:0E */
/* DW_SUBREG_OTP_SF    0x2D:12 */
#define DW_OPS_KICK      0        /**< \brief dummy */
#define DW_OPS_KICK_MASK (1<<0)   /**< \brief dummy */
#define DW_OPS_SEL       5        /**< \brief dummy */
#define DW_OPS_SEL_MASK  (0x3<<5) /**< \brief dummy */

/* DW_REG_PMSC 0x36 */
/* DW_REG_PMSC_CTRL0 0x36:00 */
#define DW_SYS_CLKS       0         /**< \brief dummy */
#define DW_SYS_CLKS_MASK  (0x3<<0)  /**< \brief dummy */
#define DW_ADCCE          10        /**< \brief dummy */
#define DW_ADCCE_MASK     (0x1<<10) /**< \brief dummy */
#define DW_SOFTRESET      28        /**< \brief dummy */
#define DW_SOFTRESET_MASK (0xF<<28) /**< \brief dummy */

/**
 * \brief Defines the max length of the host mirror of the recevie buffer in 
 * variable dw1000.
 * \todo Put in dw_driver_config.h
 */
#define DW_RX_BUFFER_MAX_LEN 128

/**
 * \brief Used to align delayed timestamps.
 * \details Whenever a variable is called dx_timestamp the low order nine bits 
 * are ignored by the device, thus to get the correct manual timestamping 
 * behaviour this needs to be accounted for.
 */
#define DX_TIMESTAMP_CLEAR_LOW_9 (0xFFFFFFFFFFFFFFE00ULL)

/**
  * \def DW_ERROR(...)
  * \brief Conditional print of encountered errors. Prints erros if the 
  * compiler is invoked with \code -D ERROR_REPORT \endcode
  */
#ifdef ERROR_REPORT
	#define DW_ERROR(...) do { printf(__VA_ARGS__); } while (0);
#else
 	#define DW_ERROR(...)
#endif

/**
 * \brief Used to choose from which source ADC should sample.
 */
typedef enum
{
	DW_ADC_SRC_LATEST = 0,
	DW_ADC_SRC_WAKEUP
} dw_adc_src_t;

/**
 * \brief DW1000 state representation.
 */
typedef enum
{
	DW_STATE_UNINITIALIZED = 0, /**< \brief DW1000 uninitialized state. */
	DW_STATE_INITIALIZING,      /**< \brief DW1000 currently running initialization.*/
	DW_STATE_IDLE,              /**< \brief DW1000 idle state.*/
	DW_STATE_SLEEP,             /**< \brief DW1000 has enetered sleep mode.*/
	DW_STATE_DEEP_SLEEP,        /**< \brief DW1000 has enetered deep sleep mode.*/
	DW_STATE_RECEIVING,         /**< \brief Reception turned on.*/
	DW_STATE_TRANSMITTING,      /**< \brief Transmission started.*/
	DW_STATE_ERROR              /**< \brief DW1000 has encouterend an error.*/
} dw1000_state_t;

/**
 * \brief DW1000 error codes.
 * \todo Add the other error codes. Find in register SYS_EVENT.
 */
typedef enum
{
	/**
	 * \brief No error encountered.
	 */
	DW_ERROR_NO_ERROR = 0,
	/**
	 * \brief Reception timed out.
	 */
	DW_ERROR_TIMEOUT

} dw1000_error_t;

/*===========================================================================*/
/*============================= Configuration ===============================*/

/**
 * \brief Specifies if a wireless transaction should be sync or async.
 * Default for statically allocated variables is async.
 */
typedef enum
{
	DW_TRANCEIVE_ASYNC = 0, /**< \brief Asynchronous communication. Default. */
	DW_TRANCEIVE_SYNC       /**< \brief Synchronous communication. */
} dw1000_tranceive_t;

/**
 * \brief Specifies which pulse repetition frequency the preamble and data 
 * sections should use.
 */
typedef enum
{
	DW_PRF_16_MHZ = 1, /**< 16 MHz Pulse repetition */
	DW_PRF_64_MHZ = 2  /**< 64 MHz Pulse repetition */
} dw1000_prf_t;

/**
 * \brief Selects the centre frequency and bandwidth for communication. The 
 * dw1000 supports a subset of the 16 channels defined in the IEEE 802.15.4
 * UWB PHY.
 */
typedef enum
{
	DW_CHANNEL_1 = 1, /**< C-Freq: 3494.4 MHz, BWD: 499.2  MHz */
	DW_CHANNEL_2 = 2, /**< C-Freq: 3993.6 MHz, BWD: 499.2  MHz */
	DW_CHANNEL_3 = 3, /**< C-Freq: 4492.8 MHz, BWD: 499.2  MHz */
	DW_CHANNEL_4 = 4, /**< C-Freq: 3993.6 MHz, BWD: 1331.2 MHz */
	DW_CHANNEL_5 = 5, /**< C-Freq: 6489.6 MHz, BWD: 499.2  MHz */
	// DW_CHANNEL_6 = 6, // Not supported
	DW_CHANNEL_7 = 7  /**< C-Freq: 6489.6 MHz, BWD: 1081.6 MHz */
	// DW_CHANNEL_8  = 8,  // Not supported
	// DW_CHANNEL_9  = 9,  // Not supported
	// DW_CHANNEL_10 = 10, // Not supported
	// DW_CHANNEL_11 = 11, // Not supported
	// DW_CHANNEL_12 = 12, // Not supported
	// DW_CHANNEL_13 = 13, // Not supported
	// DW_CHANNEL_14 = 14, // Not supported
	// DW_CHANNEL_15 = 15, // Not supported
	// DW_CHANNEL_16 = 16  // Not supported
} dw1000_channel_t;

/**
 * \brief The preamble is part of the synchronisation frame. This option
 * configures the number of symbols in the preamble.
 *
 * \details A short header provides increased speed traded for communication 
 * range.
 * 
 */
typedef enum
{
	DW_PREAMBLE_LENGTH_64   = 64,   /**< Uses 64 symbols */
	DW_PREAMBLE_LENGTH_128  = 128,  /**< Uses 128 symbols */
	DW_PREAMBLE_LENGTH_256  = 256,  /**< Uses 256 symbols */
	DW_PREAMBLE_LENGTH_512  = 512,  /**< Uses 512 symbols */
	DW_PREAMBLE_LENGTH_1024 = 1024, /**< Uses 1024 symbols */
	DW_PREAMBLE_LENGTH_1536 = 1536, /**< Uses 1536 symbols */
	DW_PREAMBLE_LENGTH_2048 = 2048, /**< Uses 2048 symbols */
	DW_PREAMBLE_LENGTH_4096 = 4096  /**< Uses 4096 symbols */

} dw1000_preamble_length_t;

/**
 * \brief The preamble code determines the specific pulse sequence of the preamble.
 */
typedef enum
{
	DW_PREAMBLE_CODE_1  = 1,
	DW_PREAMBLE_CODE_2  = 2,
	DW_PREAMBLE_CODE_3  = 3,
	DW_PREAMBLE_CODE_4  = 4,
	DW_PREAMBLE_CODE_5  = 5,
	DW_PREAMBLE_CODE_6  = 5,
	DW_PREAMBLE_CODE_7  = 7,
	DW_PREAMBLE_CODE_8  = 8,
	DW_PREAMBLE_CODE_9  = 9,
	DW_PREAMBLE_CODE_10 = 10,
	DW_PREAMBLE_CODE_11 = 11,
	DW_PREAMBLE_CODE_12 = 12,
	// DW_PREAMBLE_CODE_13 = 13,
	// DW_PREAMBLE_CODE_14 = 14,
	// DW_PREAMBLE_CODE_15 = 15,
	// DW_PREAMBLE_CODE_16 = 16,
	DW_PREAMBLE_CODE_17 = 17,
	DW_PREAMBLE_CODE_18 = 18,
	DW_PREAMBLE_CODE_19 = 19,
	DW_PREAMBLE_CODE_20 = 20
} dw1000_preamble_code_t;

/**
 * \brief Preamble Acquisition Chunk, the chunk size in which the preamble is 
 * processed. A larger PAC size is generally better for acquisition but should
 * be small compared to the preamble length. 
 * \warning A PAC size of 64 cannot detect a preamble of length 64.
 */
typedef enum
{
	DW_PAC_SIZE_8  = 8,
	DW_PAC_SIZE_16 = 16,
	DW_PAC_SIZE_32 = 32,
	DW_PAC_SIZE_64 = 64
} dw1000_pac_size_t;

/**
 * \brief Select SFD (Start of frame delimiter). The dw1000 can use the SFD 
 * defined in IEEE 802.15.4 and a Decawave specific SFD for improved 
 * performance on 100kbps. There is also the option to specify your own SFD.
 *
 * \details See the decawave dw1000 user manual for more information on how 
 * use custom SFD.
 * 
 * \warning As of 2014-10-17 the user specified SFD has not been implemented.
 */
typedef enum
{
	DW_SFD_STANDARD = 0,  /**< SFD as specified in IEEE 802.15.4. */
	DW_SFD_NON_STANDARD,  /**< Decawave SFD. */
	DW_SFD_USER_SPECIFIED /**< Indicates that a custom SFD has been defined. */
} dw1000_sfd_type_t;

/**
 * \brief DW1000 data transfer rate selection.
 *  
 *  The device can use three different data rate for communication. 110 kbps,
 *  850 kbps and 6.8 Mbps where increasing data rate decreases effective range.
 *  
 *  Default value, 6.8 Mbps.
 *
 *  \bug 110 kbps is not working properly. Packet drops for long data frames.
 *  In the ranging protocol, the blink, ranging-init, poll and response 
 *  messages are transmitted and received properly. However the final message
 *  always fails.
 */
typedef enum
{
	DW_DATA_RATE_110_KBPS  = 110,
	DW_DATA_RATE_850_KBPS  = 850,
	DW_DATA_RATE_6800_KBPS = 6800
	
} dw1000_data_rate_t;


/**
 * \brief DW1000 base configuration. Configuration of tranception properties
 *  that seldom changes. Handles configuration for:
 *  	- PRF
 *  	- Channel
 *  	- Preamble length
 *  	- Preabmle code
 *  	- PAC size
 *  	- SFD type
 *  	- Data rate
 *
 * Often changing configuration data for rx and tx are handled by
 *  \ref dw1000_rx_conf_t and \ref dw1000_tx_conf_t respectively.
 */
typedef struct
{
	/**
	 * \brief Select Pulse Repetition Frequency to use for tx/rx.
	 */
	dw1000_prf_t prf;
	/**
	 * \brief Select channel to use for rx and tx. Should be configured the
	 *  same as the remote device.
	 */
	dw1000_channel_t channel;
	/**
	 * \brief Select length of preamble.
	 */
	dw1000_preamble_length_t preamble_length;
	/**
	 * \brief Select which preamble code to use. Should be set according to
	 *  table below. Should be configured identically on both transmitting and
	 *  receiving end.
	 *
	 *        Channel no. | 16 MHz PRF |  64 MHz PRF
	 *        :---------: | :--------: | :-------------:
	 *        	   1      |    1, 2    |  9, 10, 11, 12
	 *        	   2      |    3, 4    |  9, 10, 11, 12
	 *        	   3      |    5, 6    |  9, 10, 11, 12
	 *        	   4      |    7, 8    | 17, 18, 19, 20
	 *        	   5      |    3, 4    |  9, 10, 11, 12
	 *        	   7      |    7, 8    | 17, 18, 19, 20
	 */
	dw1000_preamble_code_t preamble_code;
	/**
	 * \brief PAC size determines in how large chunks the preamble is processed
	 *  and should be set according to the table below.
	 * 
	 *        Tx preamble len | Rx PAC size
	 *        :-------------: | :---------:
	 *          64            |    8
	 *          128           |    8
	 *          256           |    16
	 *          512           |    16
	 *          1024          |    32
	 *          1536          |    64
	 *          2048          |    64
	 *          4096          |    64
	 */
	dw1000_pac_size_t pac_size;
	/**
	 * \brief Select the sfd scheme to use. Non-standard configuration can
	 *  improve performance.
	 *
	 * Can take three valid values: DW_SFD_STANDARD, DW_SFD_NON_STANDARD and
	 *  DW_SFD_USER_SPECIFIED. When configured in standard operation, SFD
	 *  sequences defined in IEEE 802.15.4-2011 UWB are used. When configured 
	 *  for non-standard operation, SFD sequences defined by Decawave are used.
	 *  The SFD sequence can also be manually configured. For more information 
	 *  on this, see DW1000 User Manual page 105 and onward.
	 */
	dw1000_sfd_type_t sfd_type;
	/**
	 * \brief Select bit rate for data transfer. The dw1000 supports three
	 *  speeds: 110 kbps, 850 kbps and 6.8 Mbps.
	 */
	dw1000_data_rate_t data_rate;

} dw1000_base_conf_t;

/**
 * \brief Specifies how to receive data with the dw1000.
 */
typedef struct
{
	/**
	 * \brief Listening will start at time specified by dx_timestamp.
	 */
	uint32_t is_delayed;
	/**
	 * \brief See \ref is_delayed.
	 * \details Value to be programmed into DW1000 dx_timestamp register.
	 */
	uint64_t dx_timestamp;

	/**
	 * \brief Timeout interval in approximate milliseconds for receiver.
	 * \details 1.026 us per tick to be exact.
	 */
	uint16_t timeout;

} dw1000_rx_conf_t;

/**
 * \brief Specifies how to transmit data with the dw1000.
 */
typedef struct
{
	uint16_t data_len; /**< \brief Length of data segment to send. */

	/**
	 * \brief Listening will start at time specified by dx_timestamp.
	 */
	uint32_t is_delayed;
	/**
	 * \brief See \ref is_delayed.
	 * \details Value to be programmed into DW1000 dx_timestamp register.
	 */
	uint64_t dx_timestamp;
} dw1000_tx_conf_t;

/*===========================================================================*/
/*============================== Base driver ================================*/

/**
 * \brief DW1000 base driver. 
 */
typedef struct
{

	/** 
	 * \brief The current state of the dw1000.
	 * \note Not fully implemented
	 */
	dw1000_state_t state;
	/** 
	 * \brief Shows last encountered error.
	 * \note Not fully implemented
	 */
	dw1000_error_t error_code;

	/**
	 * \brief Holds the configuration of the device. Modifications are
	 * committed to the dw1000 by using dw_conf().
	 */
	dw1000_base_conf_t conf;

	/**
	 * \brief Holds the number bytes received in last reception.
	 */
	uint32_t receive_buffer_len;
	/**
	 * \brief Holds the most recently received data.
	 */
	uint8_t  p_receive_buffer[DW_RX_BUFFER_MAX_LEN];

} dw1000_base_driver;

/*===========================================================================*/
/*========================== Public Declarations ============================*/

/**
 * \brief Singleton instance of the dw1000 driver. This instance mirrors the 
 * configuration on the actual device. Also provides a global access point to
 * device receive buffer data.
 */
dw1000_base_driver dw1000;

/*===========================================================================*/
/*============================ Public Functions =============================*/

// Configuration
void dw_init();
void dw_conf( dw1000_base_conf_t * dw_conf  );
void dw_conf_rx( dw1000_rx_conf_t * rx_conf );
void dw_conf_tx( dw1000_tx_conf_t * tx_conf );
void dw_conf_print();
void dw_test(void); // TODO: Test suite!

// Device communication
// Registers
void     dw_write_reg(   uint32_t reg_addr, uint32_t reg_len, uint8_t * p_data );
uint32_t dw_read_reg_32( uint32_t reg_addr, uint32_t reg_len );
uint64_t dw_read_reg_64( uint32_t reg_addr, uint32_t reg_len );
// Sub registers
void     dw_write_subreg(   uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len, uint8_t * data );
uint32_t dw_read_subreg_32( uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len );
uint64_t dw_read_subreg_64( uint32_t reg_addr, uint32_t subreg_addr, uint32_t subreg_len );
// Composite data
void dw_write_reg_multiple_data( uint32_t    reg_addr, 
								 uint32_t    reg_len, 
								 uint8_t  ** pp_data, 
								 uint32_t *  len_p_data, 
								 uint32_t    len_pp_data );
// OTP
uint32_t dw_read_otp_32( uint16_t otp_addr );

// RX / TX
void dw_receive( dw1000_tranceive_t receive_type );
void dw_transmit( uint8_t * p_data, uint32_t data_len, dw1000_tranceive_t transmit_type );

void dw_transmit_multiple_data( 	uint8_t  ** pp_data,
									uint32_t *  p_data_len,
									uint32_t    length,
									dw1000_tranceive_t transmit_type );

// Utility

// Device independent
uint8_t  dw_get_seq_no();
uint64_t dw_ms_to_device_time( float t );

// Device dependent
uint32_t dw_get_device_id();
uint64_t dw_get_device_time();

// ADC
float   dw_get_temperature(dw_adc_src_t temp_source);
float   dw_get_voltage(dw_adc_src_t voltage_source);

// Diagnostics
float dw_get_noise_level();
float dw_get_fp_ampl();
float dw_get_rx_power();
float dw_get_fp_power();

// RX/TX
void dw_get_rx_buffer(void);

void dw_set_rx_timeout( uint16_t timeout );
void dw_enable_rx_timeout();
void dw_disable_rx_timeout();

uint64_t dw_get_rx_timestamp();
uint64_t dw_get_tx_timestamp();
void     dw_set_antenna_delay( uint16_t delay );
uint16_t dw_get_antenna_delay();

uint64_t dw_get_dx_timestamp();
void 	 dw_set_dx_timestamp( uint64_t timestamp );

void dw_clear_pending_interrupt( uint64_t mask );

#endif
