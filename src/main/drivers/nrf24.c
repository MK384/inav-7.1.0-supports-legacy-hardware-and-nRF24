/**
 * Copyright (c) 2024 - present MEDA-FC All rights reserved
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      exhal_driver_nrf24.c
 * @brief     driver nrf42 source file
 * @version   1.0.0
 * @author    Mohammed Khaled
 * @date      2024-02-21
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author   		    <th>Description
 * <tr><td>2024/02/21  <td>1.0      <td>Mohammed Khaled
 * <td>
 * This driver, exhal_driver_nrf24, utilizes the nRF24 module for communication between the drone and the remote controller.
 * It provides functions and definitions necessary for initializing, configuring, sending, and receiving data using
 * the NRF24 module. This communication is used for receiving control commands from the remote controller and
 * transmitting state data such as location and battery state from the drone.
 * </table>
 */
/* Includes */
#include "nrf24.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


/**
 * @defgroup Private defines
 * @brief    contaian the driver private defines
 * @{
 */





/**
 * @brief NRF24L0 instruction definitions
 * @{
 */

#define NRF24_CMD_R_REGISTER       (uint8_t)0x00 // Register read
#define NRF24_CMD_W_REGISTER       (uint8_t)0x20 // Register write
#define NRF24_CMD_ACTIVATE         (uint8_t)0x50 // (De)Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features
#define NRF24_CMD_R_RX_PL_WID	   (uint8_t)0x60 // Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define NRF24_CMD_R_RX_PAYLOAD     (uint8_t)0x61 // Read RX payload
#define NRF24_CMD_W_TX_PAYLOAD     (uint8_t)0xA0 // Write TX payload
#define NRF24_CMD_W_ACK_PAYLOAD    (uint8_t)0xA8 // Write ACK payload
#define NRF24_CMD_W_TX_PAYLOAD_NOACK (uint8_t) 0xB0//Write TX payload and disable AUTOACK
#define NRF24_CMD_FLUSH_TX         (uint8_t)0xE1 // Flush TX FIFO
#define NRF24_CMD_FLUSH_RX         (uint8_t)0xE2 // Flush RX FIFO
#define NRF24_CMD_REUSE_TX_PL      (uint8_t)0xE3 // Reuse TX payload
#define NRF24_CMD_LOCK_UNLOCK      (uint8_t)0x50 // Lock/unlock exclusive features
#define NRF24_CMD_NOP              (uint8_t)0xFF // No operation (used for reading status register)

/**
 * @}
 */


/**
 * @brief  NRF24L0 register definitions
 *
 * @{
 */
#define NRF24_REG_CONFIG           (uint8_t)0x00 // Configuration register
#define NRF24_REG_EN_AA            (uint8_t)0x01 // Enable "Auto acknowledgment" (ShockBurst)
#define NRF24_REG_EN_RXADDR        (uint8_t)0x02 // Enable RX addresses
#define NRF24_REG_SETUP_AW         (uint8_t)0x03 // Setup of address widths
#define NRF24_REG_SETUP_RETR       (uint8_t)0x04 // Setup of automatic retransmit
#define NRF24_REG_RF_CH            (uint8_t)0x05 // RF channel
#define NRF24_REG_RF_SETUP         (uint8_t)0x06 // RF setup register
#define NRF24_REG_STATUS	       (uint8_t)0x07 // Status register
#define NRF24_REG_OBSERVE_TX       (uint8_t)0x08 // Transmit observe register
#define NRF24_REG_RPD              (uint8_t)0x09 // Received power detector
#define NRF24_REG_RX_ADDR_P0       (uint8_t)0x0A // Receive address data pipe 0
#define NRF24_REG_RX_ADDR_P1       (uint8_t)0x0B // Receive address data pipe 1
#define NRF24_REG_RX_ADDR_P2       (uint8_t)0x0C // Receive address data pipe 2
#define NRF24_REG_RX_ADDR_P3       (uint8_t)0x0D // Receive address data pipe 3
#define NRF24_REG_RX_ADDR_P4       (uint8_t)0x0E // Receive address data pipe 4
#define NRF24_REG_RX_ADDR_P5       (uint8_t)0x0F // Receive address data pipe 5
#define NRF24_REG_TX_ADDR          (uint8_t)0x10 // Transmit address
#define NRF24_REG_RX_PW_P0         (uint8_t)0x11 // Number of bytes in RX payload in data pipe 0
#define NRF24_REG_RX_PW_P1         (uint8_t)0x12 // Number of bytes in RX payload in data pipe 1
#define NRF24_REG_RX_PW_P2         (uint8_t)0x13 // Number of bytes in RX payload in data pipe 2
#define NRF24_REG_RX_PW_P3         (uint8_t)0x14 // Number of bytes in RX payload in data pipe 3
#define NRF24_REG_RX_PW_P4         (uint8_t)0x15 // Number of bytes in RX payload in data pipe 4
#define NRF24_REG_RX_PW_P5         (uint8_t)0x16 // Number of bytes in RX payload in data pipe 5
#define NRF24_REG_FIFO_STATUS      (uint8_t)0x17 // FIFO status register
#define NRF24_REG_DYNPD            (uint8_t)0x1C // Enable dynamic payload length
#define NRF24_REG_FEATURE          (uint8_t)0x1D // Feature register


#define	NRF24_REG_DOMAIN		   (uint8_t)0x1D // Last Register in the register map

/**
 * @}
 */

/**
 * @brief Register bits definitions
 *
 * @{
 */
#define NRF24_CONFIG_PRIM_RX       (uint8_t)0x01 // PRIM_RX bit in CONFIG register
#define NRF24_CONFIG_PWR_UP        (uint8_t)0x02 // PWR_UP bit in CONFIG register

#define NRF24_FEATURE_EN_DYN_ACK   (uint8_t)0x01 // EN_DYN_ACK bit in FEATURE register
#define NRF24_FEATURE_EN_ACK_PAY   (uint8_t)0x02 // EN_ACK_PAY bit in FEATURE register
#define NRF24_FEATURE_EN_DPL       (uint8_t)0x04 // EN_DPL bit in FEATURE register

/**
 * @}
 */


/**
 * @brief Register masks definitions
 * @{
 */

#define NRF24_MASK_REG_MAP         (uint8_t)0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
#define NRF24_MASK_CRC             (uint8_t)0x0C // Mask for CRC bits [3:2] in CONFIG register
#define NRF24_MASK_STATUS_IRQ      (uint8_t)0x70 // Mask for all IRQ bits in STATUS register
#define NRF24_MASK_RF_PWR          (uint8_t)0x06 // Mask RF_PWR[2:1] bits in RF_SETUP register
#define NRF24_MASK_RX_P_NO         (uint8_t)0x0E // Mask RX_P_NO[3:1] bits in STATUS register
#define NRF24_MASK_DATARATE        (uint8_t)0x08 // Mask RD_DR_[5,3] bits in RF_SETUP register
#define NRF24_MASK_EN_RX           (uint8_t)0x3F // Mask ERX_P[5:0] bits in EN_RXADDR register
#define NRF24_MASK_RX_PW           (uint8_t)0x3F // Mask [5:0] bits in RX_PW_Px register
#define NRF24_MASK_RETR_ARD        (uint8_t)0xF0 // Mask for ARD[7:4] bits in SETUP_RETR register
#define NRF24_MASK_RETR_ARC        (uint8_t)0x0F // Mask for ARC[3:0] bits in SETUP_RETR register
#define NRF24_MASK_RXFIFO          (uint8_t)0x03 // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
#define NRF24_MASK_TXFIFO          (uint8_t)0x30 // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
#define NRF24_MASK_PLOS_CNT        (uint8_t)0xF0 // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
#define NRF24_MASK_ARC_CNT         (uint8_t)0x0F // Mask for ARC_CNT[3:0] bits in OBSERVE_TX register

/**
 * @}
 */






/**
 * @brief  Pin state definitions
 * @{
 */

#define		PIN_LOW				 ((uint8_t) 0)
#define		PIN_HIGH			 ((uint8_t) 1)


/**
* }@
*/



// Fake address to test transceiver presence (5 bytes long)
#define NRF24_TEST_ADDR            "INAV7"


// interrupt transmission state type
typedef			uint8_t		it_state_t;


/**
 * @defgroup Addresses of the RX_PW_P# registers (NRF24_REG_RX_PW_xx)
 * @{
 */
static const uint8_t NRF24_RX_PW_PIPE[6] = {

		NRF24_REG_RX_PW_P0,
		NRF24_REG_RX_PW_P1,
		NRF24_REG_RX_PW_P2,
		NRF24_REG_RX_PW_P3,
		NRF24_REG_RX_PW_P4,
		NRF24_REG_RX_PW_P5
};

/**
 * @}
 */

/**
 * @defgroup Addresses of the address registers (NRF24_REG_RX_ADDR_xx)
 * @{
 */

static const uint8_t NRF24_ADDR_REGS[7] = {

		NRF24_REG_RX_ADDR_P0,
		NRF24_REG_RX_ADDR_P1,
		NRF24_REG_RX_ADDR_P2,
		NRF24_REG_RX_ADDR_P3,
		NRF24_REG_RX_ADDR_P4,
		NRF24_REG_RX_ADDR_P5,
		NRF24_REG_TX_ADDR
};


/**
 * @brief private inner variables
 */
static nrf24_handle_t* p_handler;
static it_state_t	transmit_it_active = 0;
static it_state_t	receive_it_active = 0;
static uint8_t*     it_data_buf;
static uint8_t*     it_data_length;

/**
 * @}
 */


/* -------------------------------------------------------------------------------------- */
/* ----------------------------- Inner function definition ------------------------------ */
/* -------------------------------------------------------------------------------------- */


/**
 * @brief set the CE Pin low
 */
static  void nrf24_ce_low(void){

	p_handler->ce_pin_write(PIN_LOW);
}

/**
 * @brief set the CE Pin high
 */
static  void nrf24_ce_high(void){

	p_handler->ce_pin_write(PIN_HIGH);
}

/**
 * @brief set the CSN Pin low
 */
static  void nrf24_csn_low(void) {

	p_handler->csn_pin_write(PIN_LOW);
}

/**
 * @brief set the CSN Pin high
 */
static  void nrf24_csn_high(void) {

	p_handler->csn_pin_write(PIN_HIGH);
}

/**
 * @brief low level SPI read interface
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 */
static  uint8_t nrf24_spi_read(uint8_t* pData, uint16_t size) {

	if (p_handler->spi_read == NULL){

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		p_handler->debug_print("nrf24: spi_read function not linked.\n");
#endif

		return 1;
	}

	if (p_handler->spi_read(pData, size) != 0){

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		p_handler->debug_print("nrf24: SPI failed to receive data.\n");
#endif
		return 1;
	}
	else {

		return 0;
	}
}

/**
 * @brief low level SPI write interface
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 */
static  uint8_t nrf24_spi_write(uint8_t* pData, uint16_t size) {

	if (p_handler->spi_write == NULL){

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		p_handler->debug_print("nrf24: spi_write function not linked.\n");
#endif
		return 1;
	}

	if (p_handler->spi_write(pData, size) != 0){

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		p_handler->debug_print("nrf24: SPI failed to transceive.\n");
#endif
		return 1;
	}
	else {

		return 0;
	}
}

/**
 * @brief delay milliseconds
 */
static  void nrf24_delay_ms(uint32_t ms){

	p_handler->delay_ms(ms);
}


/* -------------------------------------------------------------------------------------- */
/* ------------------------  Utility functions definitions--------------------------------*/
/* -------------------------------------------------------------------------------------- */

/**
 * @brief       Read a register
 * @param[in]   register address
 * @param[out]  register value
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note
 */
static uint8_t nrf24_read_register(uint8_t address , uint8_t* value) {

 	uint8_t reg = (address & NRF24_MASK_REG_MAP);

 	if (reg > NRF24_REG_DOMAIN) {

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		p_handler->debug_print("nrf24: access to invalid register address.\n");

#endif
 		return 	1;		/* return error */
 	}

 	nrf24_csn_low();

 	if (nrf24_spi_write(&reg, 1) != 0 ){

 	 	nrf24_csn_high();
 		return 	1;		/* return error */
 	}

 	if (nrf24_spi_read(value, 1) != 0 ){

 	 	nrf24_csn_high();
 		return 	1;		/* return error */
 	}

 	nrf24_csn_high();
 	return 0;
}

 /**
  * @brief       write a register
  * @param[in]   register address
  * @param[in]  register value
  * @return      status code
  *             - 0 success
  *             - 1 write failed
  * @note
  */
 static uint8_t nrf24_write_register(uint8_t address, uint8_t value) {

	uint8_t reg = (address & NRF24_MASK_REG_MAP);
	uint8_t buf[2];

	if (reg > NRF24_REG_DOMAIN) {

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		p_handler->debug_print("nrf24: access to invalid register address.\n");
#endif

		return 	1;		/* return error */
	}

 	nrf24_csn_low();

 	buf[0] = (reg | NRF24_CMD_W_REGISTER);
 	buf[1] = value;

 	if (nrf24_spi_write(buf, 2) != 0 ){

 	 	nrf24_csn_high();
 		return 	1;		/* return error */
 	}

 	nrf24_csn_high();
 	return 0;
 }


 /**
  * @brief       send nrf24 command
  * @param       the cmd to send
  * @return      status code
  *             - 0 success
  *             - 1 write failed
  * @note
  */
 static uint8_t nrf24_send_cmd(uint8_t cmd){

	nrf24_csn_low();

	if (nrf24_spi_write(&cmd, 1) != 0 ){

		nrf24_csn_high();
		return 	1;		/* return error */
	}

	nrf24_csn_high();
	return 0;
 }

 /**
  * @brief       Read multi-bytes
  * @param       pBuf - pointer to the buffer for register data
  * @param		 length - number of bytes to read
  * @return      status code
  *             - 0 success
  *             - 1 read failed
  * @note
  */
 static uint8_t nrf24_read_bytes(uint8_t cmd, uint8_t *pBuf, uint16_t length) {

 	nrf24_csn_low();

	if (nrf24_spi_write(&cmd, 1) != 0 ){

		nrf24_csn_high();
		return 	1;		/* return error */
	}

	if (nrf24_spi_read(pBuf, length) != 0 ){

		nrf24_csn_high();
		return 	1;		/* return error */
	}

 	nrf24_csn_high();
 	return 0;
 }


 /**
  * @brief       Write multi-bytes
  * @param       pBuf - pointer to the buffer for register data
  * @param		 length - number of bytes to write
  * @return      status code
  *             - 0 success
  *             - 1 write failed
  * @note
  */
 static uint8_t nrf24_write_bytes(uint8_t cmd, uint8_t *pBuf, uint16_t length) {

	 	nrf24_csn_low();

		if (nrf24_spi_write(&cmd, 1) != 0 ){

			nrf24_csn_high();
			return 	1;		/* return error */
		}

		if (nrf24_spi_write(pBuf, length) != 0 ){

			nrf24_csn_high();
			return 	1;		/* return error */
		}

	 	nrf24_csn_high();
	 	return 0;

}

 /* -------------------------------------------------------------------------------------- */
 /* -------------------- Private Configuration Functions Definitions --------------------- */
 /* -------------------------------------------------------------------------------------- */


 /**
  * @brief       Set transceiver DynamicPayloadLength feature for a specific pipe
  * @param       mode - status, one of NRF24_DPL_xx values
  * @param		 pipe - number of the pipe to configure
  * @return      status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
 static uint8_t nrf24_set_dpl_mode(nrf24_dpl_mode_t mode , nrf24_pipe_t pipe) {
 	uint8_t reg;


 	switch (mode) {
		case NRF24_DPL_ON:

			if (nrf24_read_register(NRF24_REG_FEATURE, &reg) != 0)
			{
				return 	1;		/* return error */
			}
			// enable dynamic payload in FEATURE Register
			if (nrf24_write_register(NRF24_REG_FEATURE, reg | NRF24_FEATURE_EN_DPL) != 0)
			{
				return 	1;		/* return error */
			}

	 		if (nrf24_read_register(NRF24_REG_DYNPD, &reg) != 0)
			{
				return 	1;		/* return error */
			}
	 		// enable DPL for the pipe
	 		if (nrf24_write_register(NRF24_REG_DYNPD, reg | (1 << pipe)) != 0)
			{
				return 	1;		/* return error */
			}
			break;
		case NRF24_DPL_OFF:

	 		// disable DPL for the pipe
	 	    if (nrf24_read_register(NRF24_REG_DYNPD, &reg) != 0)
			{
				return 	1;		/* return error */
			}
	 		if (nrf24_write_register(NRF24_REG_DYNPD, reg & ~(1 << pipe)) != 0)
			{
				return 	1;		/* return error */
			}


	 		// Disable the feature if dpl for all the feature are off
			if (nrf24_read_register(NRF24_REG_DYNPD, &reg) != 0)
			{
				return 	1;		/* return error */
			}

	 		if ( (reg & 0x3F) == 0x00)
	 		{
				if (nrf24_read_register(NRF24_REG_FEATURE, &reg) != 0)
				{
					return 	1;		/* return error */
				}
		 		if (nrf24_write_register(NRF24_REG_FEATURE, reg &  ~NRF24_FEATURE_EN_DPL) != 0)
		 		{
		 							return 	1;		/* return error */
		 		}
	 		}
	 		break;
		default:
			return 1;
			break;
	}
 	return 0;
 }

 /**
  * @brief       Configure transceiver CRC scheme
  * @param       mode - one of NRF24_CRC_xx values
  * @return		status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
 static uint8_t nrf24_set_crc_mode(nrf24_crc_mode_t mode) {

 	uint8_t reg;

 	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
 	if (nrf24_read_register(NRF24_REG_CONFIG, &reg) != 0)
	{
		return 	1;		/* return error */
	}

 	reg &= ~NRF24_MASK_CRC;
 	reg |= (mode & NRF24_MASK_CRC);

 	if (nrf24_write_register(NRF24_REG_CONFIG, reg) != 0)
	{
		return 	1;		/* return error */
	}
 	return 0;
 }

 /**
  * @brief       Set frequency channel
  * @param       channel - radio frequency channel, value from 0 to 127
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note		frequency will be (2400 + channel)MHz
  */
 static uint8_t nrf24_set_RFchannel(uint8_t channel) {

	 uint8_t temp;
	if (channel > 0x7F){
		return 	1;		/* return error */
	}

 	if (nrf24_write_register(NRF24_REG_RF_CH, channel) != 0)
	{
		return 	1;		/* return error */
	}
 	if (nrf24_read_register(NRF24_REG_RF_CH, &temp) != 0){

 		return 1;
 	}
 	if (temp != channel){

 		return 1;
 	}

 	return 0;
}

 /**
  * @brief       Set automatic retransmission parameters
  * @param       ard - auto retransmit delay, one of NRF24_ARD_xx values
  * @param       arc - count of auto retransmits, value form 0 to 15
  *
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note		zero arc value means that the automatic retransmission disabled
  */
 static uint8_t nrf24_set_auto_retransmit(nrf24_ard_t ard, uint8_t arc) {
 	// Set auto retransmit settings (SETUP_RETR register)
 	if (nrf24_write_register(NRF24_REG_SETUP_RETR, (uint8_t)((ard << 4) | (arc & NRF24_MASK_RETR_ARC))) != 0)
	{
		return 	1;		/* return error */
	}
 	return 0;
}

 /**
  * @brief       Set of address width
  * @param       addr_width - RX/TX address field width, value from 3 to 5
  *
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note		this setting is common for all pipes
  */
 static uint8_t nrf24_set_addr_width(uint8_t addr_width) {

	if(addr_width > 5 || addr_width < 3) {
		return 	1;		/* return error */
	}
 		if (nrf24_write_register(NRF24_REG_SETUP_AW, addr_width - 2) != 0)
	{
		return 	1;		/* return error */
	}
 	return 0;
}

 /**
  * @brief       Configure RF output power in TX mode
  * @param       tx_pwr - RF output power, one of NRF24_TXPWR_xx values
  *
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
 static uint8_t nrf24_set_tx_pwr(nrf24_tx_power_t tx_pwr) {

 	uint8_t reg;

 	// Configure RF_PWR[2:1] bits of the RF_SETUP register
 	if (nrf24_read_register(NRF24_REG_RF_SETUP, &reg) != 0)
	{
		return 	1;		/* return error */
	}
 	reg &= ~NRF24_MASK_RF_PWR;
 	reg |= tx_pwr;

 	if (nrf24_write_register(NRF24_REG_RF_SETUP, reg) != 0)
	{
		return 	1;		/* return error */
	}
 	return 0;
 }


 /**
  * @brief       Configure transceiver data rate
  * @param       data_rate - data rate, one of NRF24_DR_xx values
  *
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
 static uint8_t nrf24_set_data_rate(nrf24_data_rate_t data_rate) {

 	uint8_t reg;

 	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
 	if (nrf24_read_register(NRF24_REG_RF_SETUP, &reg) != 0)
	{
		return 	1;		/* return error */
	}
 	reg &= ~NRF24_MASK_DATARATE;
 	reg |= data_rate;

	if (nrf24_write_register(NRF24_REG_RF_SETUP, reg) != 0)
	{
		return 	1;		/* return error */
	}
	return 0;
}


 /**
  * @brief       Configure auto retransmit (a.k.a. enhanced ShockBurst) for RX pipe
  * @param       mode - one of NRF24_AA_xx value
  * @param       pipe - Rx Pipe number (value of nrf24_pipe_t)
  *
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
 static uint8_t nrf24_set_auto_ack(nrf24_auto_ack_mode_t mode, nrf24_pipe_t pipe) {

		uint8_t reg;

		if (nrf24_read_register(NRF24_REG_EN_AA, &reg) != 0)
		{
			return 	1;		/* return error */
		}

		switch (mode) {
			case NRF24_AUTO_ACK_ON:

				reg |= (1 << (uint8_t) pipe);

				break;
			case NRF24_AUTO_ACK_OFF:

				reg &= ~(1 << (uint8_t) pipe);

				break;
			default:
				break;
		}

		if (nrf24_write_register(NRF24_REG_EN_AA, reg) != 0)
		{
			return 	1;		/* return error */
		}

		return 0;
 }


 /**
  * @brief       Write TX payload
  * @param       pBuf - pointer to the buffer with payload data
  * @param       length - payload length in bytes
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
 static uint8_t nrf24_write_payload(uint8_t *pBuf, uint8_t length) {

 	if (nrf24_write_bytes(NRF24_CMD_W_TX_PAYLOAD, pBuf, length) != 0)
	{
		return 	1;		/* return error */
	}
 	return 0;
}

 /**
  * @brief       auxiliary function to get the received payload width from a pipe
  * @param[IN]       pipe - rx pipe to get the received payload width from
  * @param[OUT]      width - payload width in bytes
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
static uint8_t nrf24_get_rx_payload_width(nrf24_pipe_t pipe, uint8_t* width) {

 	uint8_t reg, dpl_mode;

 	if (nrf24_read_register(NRF24_REG_DYNPD, &reg) != 0)
	{
		return 	1;		/* return error */
	};

 	dpl_mode = reg & (1 << pipe);

 	if(dpl_mode){

 		// DPL is on for this pipe
 		if (nrf24_read_bytes(NRF24_CMD_R_RX_PL_WID, width, 1) != 0)
		{
			return 	1;		/* return error */
		}
 	}
 	else	// DPL is off for this pipe
 	{

 		if (nrf24_read_register(NRF24_RX_PW_PIPE[pipe], width) != 0)
		{
			return 	1;		/* return error */
		}
 	}

 	return 0;
}


static uint8_t nrf24_receive_packet_in_irq(void){


 	uint8_t pipe ,reg;


	if (nrf24_get_rx_fifo_status() ==  NRF24_FIFO_STATUS_DATA){

	 	// Extract a payload pipe number from the STATUS register
	 	if (nrf24_read_register(NRF24_REG_STATUS, &reg) != 0)
		{
			return 	1;		/* return error */
		}

	 	pipe = (reg & NRF24_MASK_RX_P_NO) >> 1;

	 	// RX FIFO empty?
	 	if (pipe < 6) {

	 		// Get payload length
	 		if(nrf24_get_rx_payload_width(pipe, it_data_length) != 0){

				return 	1;		/* return error */
	 		}

	 		// read the payload
			if (nrf24_read_bytes(NRF24_CMD_R_RX_PAYLOAD, it_data_buf, *it_data_length) != 0)
			{
				return 	1;		/* return error */
			}

	 	}
	 	else if(pipe == 7){

		 	// The RX FIFO is empty
		 	*it_data_length = 0;
			return 	1;		/* return error */

	 	}

	}

	if (nrf24_get_rx_fifo_status() ==  NRF24_FIFO_STATUS_FULL){

	 	nrf24_flush_rx_fifo();
	}

 	return 0;

}


 /* -------------------------------------------------------------------------------------- */
 /* ----------------------------- APIs Functions Definitions -------------------------- */
 /* -------------------------------------------------------------------------------------- */


 /**
  * @brief      Reset the nrf24 transceiver chip to it's initial state
  * @param	   none
  * @return     status code
  *             - 0 success
  *             - 1 failed to communicate with the nrf24 module
  * @note       RX/TX pipe addresses remains untouched
  */
 uint8_t nrf24_chip_reset(void) {

	 uint8_t default_tx_addr     [5]    = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
	 uint8_t default_rx_addr_p1  [5]    = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2 };

	 // assert CSN pin high (chip release)
	 	nrf24_csn_high();

 	// Write to registers their initial values
 	if (nrf24_write_register(NRF24_REG_CONFIG, 0x08))				 return 1;
 	if (nrf24_write_register(NRF24_REG_EN_AA, 0x3F))  				 return 1;
 	if (nrf24_write_register(NRF24_REG_EN_RXADDR, 0x03)) 			 return 1;
 	if (nrf24_write_register(NRF24_REG_SETUP_AW, 0x03)) 			 return 1;
 	if (nrf24_write_register(NRF24_REG_SETUP_RETR, 0x03))			 return 1;
 	if (nrf24_write_register(NRF24_REG_RF_CH, 0x02))				 return 1;
 	if (nrf24_write_register(NRF24_REG_RF_SETUP, 0x0F)) 			 return 1;
 	if (nrf24_write_register(NRF24_REG_STATUS, 0x0E)) 				 return 1;

 	if(nrf24_write_bytes(NRF24_REG_RX_ADDR_P0 | NRF24_CMD_W_REGISTER, default_tx_addr, 5))
 																	 return 1;
 	if(nrf24_write_bytes(NRF24_REG_RX_ADDR_P1 | NRF24_CMD_W_REGISTER, default_rx_addr_p1, 5))
 																	 return 1;

 	if (nrf24_write_register(NRF24_REG_RX_ADDR_P2, 0xC3)) 			 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_ADDR_P3, 0xC4)) 			 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_ADDR_P4, 0xC5)) 			 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_ADDR_P5, 0xC6)) 			 return 1;


 	if(nrf24_write_bytes(NRF24_REG_TX_ADDR | NRF24_CMD_W_REGISTER, default_tx_addr, 5))
 																	 return 1;

 	if (nrf24_write_register(NRF24_REG_RX_PW_P0, 0x00))				 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_PW_P1, 0x00)) 			 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_PW_P2, 0x00))				 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_PW_P3, 0x00))				 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_PW_P4, 0x00))				 return 1;
 	if (nrf24_write_register(NRF24_REG_RX_PW_P5, 0x00)) 			 return 1;

 	if (nrf24_write_register(NRF24_REG_DYNPD, 0x00))				 return 1;
 	if (nrf24_write_register(NRF24_REG_FEATURE, 0x00))			     return 1;
 	// Clear the FIFO's
 	if (nrf24_flush_rx_fifo())										 return 1;
 	if (nrf24_flush_tx_fifo())										 return 1;

 	// assert CSN pin high (chip release)
 	nrf24_csn_high();
 	// assert CW pin low (standby mode 1)
 	nrf24_ce_low();

 	return 0;
 }

 /**
  * @brief      Check if the nRF2424L01 present and available
  * @param	   none
  * @return     status code
  *             - 0 NRF24L01 is online and responding
  *             - 1 failed to communicate with the nrf24 module
  * @note
  */
 uint8_t nrf24_check_available(void) {

 	uint8_t rxbuf[5] = {0,0,0,0,0};
 	uint8_t i;
 	uint8_t *ptr = (uint8_t *) NRF24_TEST_ADDR;

 	// Write test TX address and read TX_ADDR register
 	if (nrf24_write_bytes((NRF24_REG_TX_ADDR | NRF24_CMD_W_REGISTER), ptr, 5) != 0){
		return 	1;		/* return error */
 	}

    nrf24_delay_ms(2);

	if (nrf24_read_bytes(NRF24_REG_TX_ADDR, rxbuf, 5) != 0)
	{
		return 	1;		/* return error */
	}

 	// Compare buffers, return error on first mismatch
 	for (i = 0; i < 5; i++) {

        if (rxbuf[i] != ptr[i]) return 1;        /* return error */
 	}

 	return 0;
}

 /**
  * @brief      Init the NRF24 Module with the configuration pattern in the handler struct
  * @param[in] *handle pointer to the handler struct type
  * @return     status code
  *             - 0 success
  *             - 1 failed to communicate with the nrf24 module
  *             - 2 handle is NULL
  * @note       none
  */
 uint8_t nrf24_init(nrf24_handle_t* handle){


	if(handle == NULL){

		return 2;		/* handle is null error */
	}

	p_handler = handle;

	if (nrf24_chip_reset())															return 1;
	if (nrf24_check_available()) 													return 1;
 	if (nrf24_set_RFchannel(handle->RF_channel)) 									return 1;
 	if (nrf24_set_addr_width(handle->address_width))	   						    return 1;
	if (nrf24_set_auto_retransmit(handle->ard, handle->arc)) 						return 1;
 	if (nrf24_set_tx_pwr(handle->tx_pwr))											return 1;
 	if (nrf24_set_data_rate(handle->data_rate))										return 1;
 	if (nrf24_set_crc_mode(handle->crc_mode))									 	return 1;

 	if (nrf24_flush_rx_fifo())														return 1;
 	if (nrf24_flush_tx_fifo())														return 1;

 	if (nrf24_clear_irq_flags(NRF24_FLAG_MAX_RT | NRF24_FLAG_RX_DR | NRF24_FLAG_TX_DS))
 																					return 1;

 	return 0;
 }


 /**
  * @brief     irq handler

  * @return    status code
  *            - 0 success
  *            - 1 run failed
  *            - 2 handle is NULL
  * @note      none
  */
 uint8_t nrf24_irq_handler(void)
 {
	    uint8_t flags;

	     if (p_handler == NULL)                                                                   /* check handle */
	     {
	         return 2;                                                                         /* return error */
	     }

	     if (nrf24_get_irq_flags(&flags))
		 {
	    	 return	1;																		 /* return error */
		 }

	     //---------------------------------------------------------------------------------
	     //--------------------- Max Retransmission Flag Handling --------------------------
	     //---------------------------------------------------------------------------------

	     if ((flags & NRF24_FLAG_MAX_RT) != 0)                                 			   /* if max retransmission */
	     {
	    	 if(transmit_it_active){

	    		 // assert ce pin low
	    		 nrf24_ce_low();

//	    		 transmit_it_active  = 0;

	    	 }

	         if (p_handler->receive_callback != NULL)                                       /* if receive callback */
	         {
	             p_handler->receive_callback(NRF24_FLAG_MAX_RT);                           /* run callback */
	         }

	         nrf24_clear_irq_flags(NRF24_FLAG_MAX_RT);
	     }

	     //---------------------------------------------------------------------------------
	     //--------------------- Data Received Ready Flag Handling -------------------------
	     //---------------------------------------------------------------------------------
	     if ((flags & NRF24_FLAG_RX_DR) != 0)                                   		 /* if received data ready */
	     {

	    	 if (receive_it_active){

	    		 receive_it_active = 0;

	    		 if (nrf24_receive_packet_in_irq()  !=  0){

	    			 return 1;

	    		 }
	    	 }

	         if (p_handler->receive_callback != NULL)                                    /* if receive callback */
	         {
	             p_handler->receive_callback(NRF24_FLAG_RX_DR);                           /* run callback */
	         }

	         nrf24_clear_irq_flags(NRF24_FLAG_RX_DR);

	     }

	     //---------------------------------------------------------------------------------
	     //--------------------- Data Sent Flag Handling --------------------------
	     //---------------------------------------------------------------------------------

	     if ((flags & NRF24_FLAG_TX_DS) != 0)                                  			  /* if transmission done */
	     {
	    	 if(transmit_it_active){

	    		 // assert ce pin low
	    		 nrf24_ce_low();

	    		 transmit_it_active = 0;

	    	 }

	         if (p_handler->receive_callback != NULL)                                     /* if receive callback */
	         {
	             p_handler->receive_callback(NRF24_FLAG_TX_DS);                           /* run callback */
	         }

	         nrf24_clear_irq_flags(NRF24_FLAG_TX_DS);

			 if (nrf24_get_tx_fifo_status() == NRF24_FIFO_STATUS_FULL){

				 nrf24_flush_tx_fifo();
			 }
	     }

	     return 0;                                                                        /* success return 0 */
 }





 /**
  * @brief      Control transceiver power ON/OFF mode
  * @param	   mode - new state of power mode, one of NRF24_PWR_xx values
  * @return     status code
  *             - 0 success
  *             - 1 failed to communicate with the nrf24 module
  * @note       the power must be set on before any operation!
  */
 uint8_t nrf24_set_power_mode(nrf24_power_mode_t mode) {

 	uint8_t reg;

 	if (nrf24_read_register(NRF24_REG_CONFIG, &reg))		return 1;


 	switch (mode) {
		case NRF24_PWR_UP:
	 		// Set the PWR_UP bit of CONFIG register to wake the transceiver
	 		// It goes into Stanby-I mode with consumption about 26uA
	 		reg |= NRF24_CONFIG_PWR_UP;
	 		break;
		case NRF24_PWR_DOWN:
	 		// Clear the PWR_UP bit of CONFIG register to put the transceiver
	 		// into power down mode with consumption about 900nA
	 		reg &= ~NRF24_CONFIG_PWR_UP;
			break;
		default:
			break;
	}

 	if (nrf24_write_register(NRF24_REG_CONFIG, reg))		return 1;

 	return 0;

 }


 /**
  * @brief  NRF24_TransmitPacket : Function to transmit data packet
  * @param[IN]  uint8_t*  pBuf:		 pointer to the buffer with data to transmit
    @param[IN]  uint8_t	 length:	 length of the data buffer in bytes
    @param[IN]  uint32_t  timeOut:    time to wait if the TX FIFO is Full
  * @return	   nrf24_tx_result_t :	 one of nrf24_tx_result_t values
  * @note
  */
 nrf24_tx_result_t nrf24_transmit_packet(void *p_data, uint8_t length, uint32_t time_out ) {


 	uint8_t flags = 0;

 	// Deassert the CE pin (in case if it still high)
 	nrf24_ce_low();

 	// Transfer a data from the specified buffer to the TX FIFO
 	if (nrf24_write_payload((uint8_t*)p_data, length)){
			return NRF24_TX_ERROR;

 	}

 	// Start a transmission by asserting CE pin (must be held at least 10us)
 	nrf24_ce_high();

 	// Poll the transceiver status register until one of the following flags will be set:
 	//   TX_DS  - means the packet has been transmitted
 	//   MAX_RT - means the maximum number of TX retransmits happened
 	do {

 		if (nrf24_get_irq_flags(&flags))
 		{
 			return NRF24_TX_ERROR;
 		}

 		if (flags & (NRF24_FLAG_TX_DS | NRF24_FLAG_MAX_RT)) {
 			break;
 		}

 		if (time_out)
		{
 	 		nrf24_delay_ms(1);
 			time_out--;
		}
 		else if (time_out == 0)
 		{
 			nrf24_ce_low();
 			nrf24_clear_irq_flags(flags);

 	 	 	if (nrf24_flush_tx_fifo()) return NRF24_TX_ERROR;

 			return NRF24_TX_TIMEOUT;
 		}

 	} while (1);

 	// Deassert the CE pin (Standby-II --> Standby-I)
 	nrf24_ce_low();

	nrf24_clear_irq_flags(flags);

		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
 	if (flags & NRF24_FLAG_MAX_RT)
	{
 	 	nrf24_flush_tx_fifo();
 		return NRF24_TX_MAXRT;
	}

		// Successful transmission
 	if (flags & NRF24_FLAG_TX_DS)
	{
 		return NRF24_TX_SUCCESS;
	}

 	// Some undefined error happened, a payload remains in the TX FIFO, flush it
 	nrf24_flush_tx_fifo();
 	return NRF24_TX_ERROR;
 }

 /**
  * @brief  NRF24_TransmitPacket : Function to transmit data packet in interrupt mode
  * @param[IN]  uint8_t*  pBuf:		 pointer to the buffer with data to transmit
    @param[IN]  uint8_t	 length:	 length of the data buffer in bytes
  * @return	   status code
  *
  * 				0	Success
  * 				1   nrf24 module hasn't initialized
  * 				2	invalid pointer to data
  * 				3	previous interrupt transmission still active
  * 				4	communication error
  * @note
  */
 uint8_t nrf24_transmit_packet_it(void *p_data, uint8_t length){

	 uint8_t result;

	 if(p_handler == NULL){

			return 	1;		/* return error */

	 }
	 if(p_data == NULL){

#ifdef	NRF24_DEBUG_PRINT_ENABLED

		 	if (p_handler->debug_print != NULL)
		 	{
		 		p_handler->debug_print("nrf24: transmitting invalid pointer to data!\n");
		 	}
#endif
			return 	1;		/* return error */
	 }

	 if(transmit_it_active == 1){

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		 	if (p_handler->debug_print != NULL)
		 	{
		 		p_handler->debug_print("nrf24: prev transmission still active!\n");
		 	}
#endif
		 	// clear MAX RT Flag if asserted
		 	nrf24_clear_irq_flags(NRF24_FLAG_MAX_RT);
		 	result = 	NRF24_TX_MAXRT;		/* return error */

	 }
	 else{

		 transmit_it_active = 1;
		 result = NRF24_TX_SUCCESS;
	 }

	 	// Deassert the CE pin (in case if it still high)
	 	nrf24_ce_low();

	 	// Transfer a data from the specified buffer to the TX FIFO
	 	if (nrf24_write_payload((uint8_t*)p_data, length))
	 	{
	 		return	NRF24_TX_ERROR;		/* return error */
	 	}

	 	// Start a transmission by asserting CE pin (must be held at least 10us)
	 	nrf24_ce_high();

	 	return	result;
 }

 /**
  * @brief       Configure the ack with payload mode
  * @param       mode - one of NRF24_ACK_PAYLOAD_xx values
  * @return		 status code
  *             - 0 success
  *             - 1 operation failed
  * @note
  */
 uint8_t nrf24_set_ack_payload_mode(nrf24_ack_payload_mode_t mode){

	 uint8_t reg;

     /* ACTIVATE the feature through the activate command  */
     reg = 0x73;
     if (nrf24_write_bytes(NRF24_CMD_ACTIVATE, &reg, 1)){

        		return 	1;		/* return error */
     }

    nrf24_delay_ms(2);      // time for settling

	if (nrf24_read_register(NRF24_REG_FEATURE, &reg))
	{
		return 	1;		/* return error */
	}

	 switch (mode) {
		case NRF24_ACK_PAYLOAD_ON:

			if (nrf24_write_register(NRF24_REG_FEATURE, reg | NRF24_FEATURE_EN_ACK_PAY) != 0)
			{
				return 	1;		/* return error */
			}

		      // Enable dynamic payload on pipes 0 & 1
			if (nrf24_set_dpl_mode(NRF24_DPL_ON, NRF24_PIPE0) != 0)
			{
				return 	1;		/* return error */
			}
			if (nrf24_set_dpl_mode(NRF24_DPL_ON, NRF24_PIPE1) != 0)
			{
				return 	1;		/* return error */
			}

			break;
		case NRF24_ACK_PAYLOAD_OFF:

			if (nrf24_write_register(NRF24_REG_FEATURE, reg & ~NRF24_FEATURE_EN_ACK_PAY) != 0)
			{
				return 	1;		/* return error */
			}
			break;
		default:
			break;
	}
	 return 0;
 }

 /**
  * @brief 	 					    Transmit ack with payload (DPL must be on to work with this feature)
  * @param nrf24_pipe_t pipe: 	    pipe that associated with ack packet
  * @param  uint8_t*payload :        pointer to the buffer with data to transmit in ack packet
    @param uint8_t length : 			length of the data buffer in bytes
  * @return     status code
  *             - 0 success
  *             - 1 failed to communicate with the nrf24 module
  * @note
  */
 uint8_t nrf24_send_ack_payload(nrf24_pipe_t pipe, uint8_t *payload, uint8_t length) {


	if (nrf24_write_bytes((NRF24_CMD_W_ACK_PAYLOAD | pipe), payload, length) != 0)
	{
		return 	1;		/* return error */
	}

	return 0;
}

 /**
  * @brief   						 Receive a data packet
  * @param[OUT]  uint8_t* pBuf : 	 buffer for the data,
  * @param[OUT]  uint8_t* length :  	 length of the data
  * @param[IN]  uint32_t time_out	 	 time to wait if the RX FIFO is empty
  * @return   nrf24_rx_result_t :	 one of the NRF24_RX_xx values
  * @note
  */
 nrf24_rx_result_t nrf24_receive_packet(uint8_t *pBuf, uint8_t* length, uint32_t time_0ut){

	uint8_t pipe ,reg;

	do{

		if (!(nrf24_get_rx_fifo_status() & NRF24_FIFO_STATUS_EMPTY)){
			break;
		}

		if (time_0ut){

			nrf24_delay_ms(1);
			time_0ut--;
		}
		else if (time_0ut == 0)
		{
			return NRF24_RX_TIMEOUT;
		}

	}while(1);

	// Extract a payload pipe number from the STATUS register
	if (nrf24_read_register(NRF24_REG_STATUS, &reg) != 0)
	{
		return 	NRF24_RX_ERROR;		/* return error */
	}

	pipe = (reg & NRF24_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {

		// Get payload length
		if(nrf24_get_rx_payload_width(pipe, length) != 0){
			return 	NRF24_RX_ERROR;		/* return error */
		}

		// Read a payload from the RX FIFO
		if (*length) {

			if (nrf24_read_bytes(NRF24_CMD_R_RX_PAYLOAD, pBuf, *length) != 0)
			{
				return 	NRF24_RX_ERROR;		/* return error */
			}

			return NRF24_RX_SUCCESS;

		}
		else{

			return NRF24_RX_ERROR;
		}
	}
	else if(pipe == 7){

		// The RX FIFO is empty
		*length = 0;
		return NRF24_RX_EMPTY;

	}

	nrf24_flush_rx_fifo();
	return NRF24_RX_ERROR;
}

 /**
  * @brief  			 Function to receive  data packet in interrupt mode
  * @param[IN]  uint8_t*  pBuf:		 pointer to the buffer with data to receive
    @param[IN]  uint8_t	 length:	 length of the data buffer in bytes
  * @return	   		status code
  *
   * 				0	Success
   * 				1   nrf24 module hasn't initialized
   * 				2	invalid pointer to data
   * 				3	previous interrupt receive still active
   * 				4	communication error
  * @note
  */
uint8_t nrf24_receive_packet_it(uint8_t *p_data, uint8_t* length){



	 if(p_handler == NULL){

			return 	1;		/* return error */

	 }
	 if(p_data == NULL){

#ifdef	NRF24_DEBUG_PRINT_ENABLED
		 	if (p_handler->debug_print != NULL)
		 	{
		 		p_handler->debug_print("nrf24: transmitting invalid pointer to data!\n");
		 	}
#endif

			return 	1;		/* return error */
	 }

	 if(receive_it_active == 1){

			return 	3;		/* return error */

	 }

	if (nrf24_get_rx_fifo_status() ==  NRF24_FIFO_STATUS_FULL){

		nrf24_flush_rx_fifo();
	}

	it_data_buf	   = p_data;
	it_data_length = length;

	*it_data_length = 0;

	receive_it_active = 1;

	return 0;

}

 /**
  * @brief      Set the nRF24 in RX Mode or in TX Mode
  * @param	   mode : one of two modes NRF24_MODE_RX or NRF24_MODE_TX
  * @return     status code
  *             - 0 success
  *             - 1 failed to communicate with the nrf24 module
  * @note: 	   none
  */
 uint8_t nrf24_set_radio_mode(nrf24_radio_mode_t mode){

	 	uint8_t reg;

	 	// Configure PRIM_RX bit of the CONFIG register
	 	if (nrf24_read_register(NRF24_REG_CONFIG, &reg) != 0){
			return 	1;		/* return error */
	 	}
	 	reg &= ~NRF24_CONFIG_PRIM_RX;
	 	reg |= (mode & NRF24_CONFIG_PRIM_RX);

	 	if (nrf24_write_register(NRF24_REG_CONFIG, reg) != 0){
			return 	1;		/* return error */
	 	}

        // Assert we are in correct mode
        if (nrf24_read_register(NRF24_REG_CONFIG, &reg) != 0){
			return 	1;		/* return error */
	 	}
        switch (mode)
        {
        case NRF24_MODE_RX:
        if ((reg & NRF24_CONFIG_PRIM_RX) != NRF24_CONFIG_PRIM_RX){
			return 	1;		/* return error */
	 	}
            break;
        case NRF24_MODE_TX:
        if ((reg & NRF24_CONFIG_PRIM_RX) == NRF24_CONFIG_PRIM_RX){
			return 	1;		/* return error */
	 	}
            break;            
        
        default:
            break;
        }


	 switch (mode) {
		case NRF24_MODE_RX: nrf24_ce_high();
			break;

		case NRF24_MODE_TX: nrf24_ce_low();
			break;
		default:
			break;
	}

	 return 0;

 }

 /**
  * @brief      Open and configure transmitting Pipe
  * @param	   uint8_t* address: address of Transmitter
  * @return     status code
  *             - 0 success
  *             - 1 failed to communicate with the nrf24 module
  * @note: 	   none
  */
 uint8_t nrf24_open_tx_pipe(uint8_t* address, nrf24_auto_ack_mode_t auto_ack_mode){

		uint8_t addr_width , reg;
		// get address width
		if (nrf24_read_register(NRF24_REG_SETUP_AW, &reg) != 0)
		{
			return 	1;		/* return error */
		}
		addr_width = reg + 2;
		if (nrf24_write_bytes(NRF24_REG_TX_ADDR, address, addr_width) != 0)
		{
			return 	1;		/* return error */
		}

		// pipe 0 must has the same address if AA is enabled (as stated in datasheet)
		if (auto_ack_mode == NRF24_AUTO_ACK_ON){

			if (nrf24_open_rx_pipe(NRF24_PIPE0, address, NRF24_AUTO_ACK_ON, NRF24_DPL_ON, 0) != 0)
			{
				return 	1;		/* return error */
			}
		}

		return 0;
 }

 /**
  * @brief  NRF24_OpenRxPipe : Open Receiving Pipe
  * @param  pipe : the pipe to be opened for receiving
  * @param  address:	 bytes of the address of the pipe (number of bytes is set in config struct)
  * @param  auto_ack_mode: one of the NRF24_AA_xx values
  * @param  Mode of the DPL for this Pipe
  * @param  payloadLength: the length of the payload received on this pipe , if DPL is enabled the value is ignored.
  * @return     status code
  *             - 0 success
  *             - 1 failed to communicate with the nrf24 module
  * @note   payload_length value is ignored if dpl mode is on for the pipe
  */
 uint8_t nrf24_open_rx_pipe( nrf24_pipe_t pipe ,uint8_t* address, nrf24_auto_ack_mode_t auto_ack_mode,
 							nrf24_dpl_mode_t dpl_mode , uint8_t payload_length){

	 	uint8_t addr_width , reg;

	 	// Enable the specified pipe (EN_RXADDR register)
	 	if (nrf24_read_register(NRF24_REG_EN_RXADDR, &reg))							return 1;
	 	if (nrf24_write_register(NRF24_REG_EN_RXADDR, (reg | (1 << pipe)) & NRF24_MASK_EN_RX))
	 		return 1;

	 	// Set the address for the pipe
	 	switch (pipe) {
	 		case NRF24_PIPE0:
	 		case NRF24_PIPE1:
	 			// Get address width
	 			if (nrf24_read_register(NRF24_REG_SETUP_AW, &addr_width))			return 1;
	 			if (nrf24_write_bytes(NRF24_ADDR_REGS[pipe] | NRF24_CMD_W_REGISTER, address, addr_width +2))
	 				return 1;

	 			break;
	 		case NRF24_PIPE2:
	 		case NRF24_PIPE3:
	 		case NRF24_PIPE4:
	 		case NRF24_PIPE5:
	 			// Write address LSBbyte (only first byte from the addr buffer)
	 			if (nrf24_write_register(NRF24_ADDR_REGS[pipe], *address)) 			return 1;
	 			break;
	 		default:
	 			// Incorrect pipe number -> return error
	 			return 1;
	 			break;
	 	}

	 	if (nrf24_set_auto_ack(auto_ack_mode, pipe))		return 1;

	 	if (nrf24_set_dpl_mode(dpl_mode, pipe))		    	return 1;
	 	// set the payload length only if the DPL is Off
	 	if (dpl_mode == NRF24_DPL_OFF)
	 	{
	 	// Set RX payload length (RX_PW_Px register)
	 		if (nrf24_write_register(NRF24_RX_PW_PIPE[pipe], payload_length & NRF24_MASK_RX_PW))
	 			return 1;
	 	}

	 	return 0;
 }

/**
* @brief    Close Receiving Pipe
* @param    pipe : pipe to be closed
* @return     status code
*             - 0 success
*             - 1 failed to communicate with the nrf24 module
* @note
*/
uint8_t nrf24_close_rx_pipe(nrf24_pipe_t pipe){

 	uint8_t reg;

 	if (nrf24_read_register(NRF24_REG_EN_RXADDR, &reg))		return 1;
 	reg &= ~(1 << pipe);
 	reg &= NRF24_MASK_EN_RX;
 	if (nrf24_write_register(NRF24_REG_EN_RXADDR, reg))		return 1;

 	return 0;
 }


/**
 * @brief      Sets the interrupt masks for the IRQ pin
 * @param[in]  masks - interrupt masks to be set, one or more of NRF24_MASKS_xx values
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note: By default Interrupts reflected on the IRQ pin , Masking for disabling the interrupt.
 * @note: The interrupt on IRQ pin is active low.
 */
uint8_t nrf24_set_interrupt_masks(uint8_t masks){

	 uint8_t reg = 0;

	 if (nrf24_read_register(NRF24_REG_CONFIG, &reg))			return 1;

	 reg &= ~NRF24_MASK_STATUS_IRQ; 		            // clear the three masks bits first
	 // write the new masks
	 if (nrf24_write_register(NRF24_REG_CONFIG, reg | masks)) 	return 1;

	 return 0;
}



/**
 * @brief      Get pending IRQ flags
 * @param      flags:Pending irq active flags:
 * 			   current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
* @return     status code
*             - 0 success
*             - 1 failed to communicate with the nrf24 module
 * @note: 	   none
 */
uint8_t nrf24_get_irq_flags(uint8_t* pending_flags) {


 	 if (nrf24_read_register(NRF24_REG_STATUS, pending_flags))
 	 {
 		 return	1;
 	 }
 	 return  0;
 }

/**
 * @brief      Get status of the RX FIFO
 * @param	   one of the NRF24_FIFO_STATUS_xx values
 * @return     none
 * @note: 	   none
 */
nrf24_fifo_status_t nrf24_get_rx_fifo_status(void) {
	uint8_t reg;
 	nrf24_read_register(NRF24_REG_FIFO_STATUS, &reg);
	return (reg & NRF24_MASK_RXFIFO);
 }

/**
 * @brief      Get status of the TX FIFO
 * @param	   one of the NRF24_FIFO_STATUS_xx values
 * @return     none
 * @note: 	   none
 */
nrf24_fifo_status_t nrf24_get_tx_fifo_status(void) {
	uint8_t reg;
 	nrf24_read_register(NRF24_REG_FIFO_STATUS, &reg);
	return ((reg & NRF24_MASK_TXFIFO) >> 4);
 }

/**
 * @brief      Get pipe number for the payload available for reading from RX FIFO
 * @param	   none
 * @return     pipe number, one value of NRF24_PIPEx
 * @note: 	   if value of 0x07 returned that means the RX FIFO is empty
 */
nrf24_pipe_t nrf24_get_rx_pipe_source(void) {
	uint8_t reg;
	nrf24_read_register(NRF24_REG_STATUS, &reg);
 	return (nrf24_pipe_t)((reg & NRF24_MASK_RX_P_NO) >> 1);
 }

 // Flush the TX FIFO
 uint8_t nrf24_flush_tx_fifo(void) {

 	return nrf24_send_cmd(NRF24_CMD_FLUSH_TX);
 }

 // Flush the RX FIFO
 uint8_t nrf24_flush_rx_fifo(void) {

	return nrf24_send_cmd(NRF24_CMD_FLUSH_RX);
 }

 /**
  * @brief      Clear any pending IRQ flags, RX_DR, TX_DS and MAX_RT bits of the STATUS register
  * @param	   on or more of NRF24_FLAGS_xx values
  * @return     none
  * @note: 	   none
  */
 uint8_t nrf24_clear_irq_flags(uint8_t flags) {
 	uint8_t reg;

 	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
 	if (nrf24_read_register(NRF24_REG_STATUS, &reg)) 			return 1;
 	if (nrf24_write_register(NRF24_REG_STATUS,reg & flags))		return 1;

 	return 0;
}

