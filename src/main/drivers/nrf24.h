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
 * @file      exhal_driver_nrf24.h
 * @brief     driver nrf42 header file
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

#ifndef EXHAL_DRIVER_NRF2424_H_
#define EXHAL_DRIVER_NRF2424_H_



/* --- Includes -------------------------------------------- */
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C"{
#endif

/* --- Global Defines -------------------------------------------- */

//#define		NRF24_DEBUG_PRINT_ENABLED

/**
 * @defgroup nrf24_driver data structure
 * @brief    nrf24-driver modules required data types
 * @{
 */

/**
 * @brief Auto Retransmit Delay (NRF24_ARD_xx)
 */
typedef enum {
	NRF24_ARD_NONE   = (uint8_t)0x00, // Dummy value for case when retransmission is not used
	NRF24_ARD_250us  = (uint8_t)0x00,
	NRF24_ARD_500us  = (uint8_t)0x01,
	NRF24_ARD_750us  = (uint8_t)0x02,
	NRF24_ARD_1000us = (uint8_t)0x03,
	NRF24_ARD_1250us = (uint8_t)0x04,
	NRF24_ARD_1500us = (uint8_t)0x05,
	NRF24_ARD_1750us = (uint8_t)0x06,
	NRF24_ARD_2000us = (uint8_t)0x07,
	NRF24_ARD_2250us = (uint8_t)0x08,
	NRF24_ARD_2500us = (uint8_t)0x09,
	NRF24_ARD_2750us = (uint8_t)0x0A,
	NRF24_ARD_3000us = (uint8_t)0x0B,
	NRF24_ARD_3250us = (uint8_t)0x0C,
	NRF24_ARD_3500us = (uint8_t)0x0D,
	NRF24_ARD_3750us = (uint8_t)0x0E,
	NRF24_ARD_4000us = (uint8_t)0x0F
}nrf24_ard_t;

/**
* @brief Data rate (NRF24_DR_xx)
*/
typedef enum {
	// NRF24_DR_250kbps = (uint8_t)0x20, // 250kbps data rate
	NRF24_DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
	NRF24_DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
}nrf24_data_rate_t;


/**
* @brief RF output power in TX mode (NRF24_TXPWR_xx)
*/
typedef enum {
	NRF24_TXPWR_18dBm = (uint8_t)0x00, // -18dBm
	NRF24_TXPWR_12dBm = (uint8_t)0x02, // -12dBm
	NRF24_TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
	NRF24_TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
}nrf24_tx_power_t;

/**
* @brief CRC encoding scheme (NRF24_CRC_xx)
*/
typedef enum {
	NRF24_CRC_off   = (uint8_t)0x00, // CRC disabled
	NRF24_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
	NRF24_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
}nrf24_crc_mode_t;

/**
* @brief NRF24L01 power control (NRF24_PWR_xx)
*/
typedef enum {
	NRF24_PWR_UP   = (uint8_t)0x02, // Power up
	NRF24_PWR_DOWN = (uint8_t)0x00  // Power down
}nrf24_power_mode_t;

/**
* @brief Transceiver mode (NRF24_MODE_xx)
*/
typedef enum {
	NRF24_MODE_RX = (uint8_t)0x01, // PRX
	NRF24_MODE_TX = (uint8_t)0x00  // PTX
}nrf24_radio_mode_t;

/**
* @brief Dynamic Payload lenght mode (NRF24_DPL_xx)
*/
typedef enum {
	NRF24_DPL_ON = (uint8_t)0x01, // Dynamic payload is On
	NRF24_DPL_OFF = (uint8_t)0x00  // Dynamic payload is Off
}nrf24_dpl_mode_t ;

/**
* @brief Enumeration of RX pipe addresses and TX address
*/
typedef enum {
	NRF24_PIPE0  = (uint8_t)0x00, // pipe0
	NRF24_PIPE1  = (uint8_t)0x01, // pipe1
	NRF24_PIPE2  = (uint8_t)0x02, // pipe2
	NRF24_PIPE3  = (uint8_t)0x03, // pipe3
	NRF24_PIPE4  = (uint8_t)0x04, // pipe4
	NRF24_PIPE5  = (uint8_t)0x05, // pipe5
}nrf24_pipe_t;

/**
* @brief State of auto acknowledgment for specified pipe (NRF24_AA_xx)
*/
typedef enum {
	NRF24_AUTO_ACK_OFF = (uint8_t)0x00,
	NRF24_AUTO_ACK_ON  = (uint8_t)0x01
}nrf24_auto_ack_mode_t;

/**
* @brief Enables the Ack with payload feature (this features forces enabling the DPL for pipe 0)
*/
typedef enum {
	NRF24_ACK_PAYLOAD_OFF = (uint8_t)0x00,
	NRF24_ACK_PAYLOAD_ON  = (uint8_t)0x01
}nrf24_ack_payload_mode_t;

/**
* @brief Status of the RX/TX FIFO (nrf24_fifo_status_t_xx)
*/
typedef enum {
	NRF24_FIFO_STATUS_DATA  = (uint8_t)0x00, // The RX/TX FIFO contains data and available locations
	NRF24_FIFO_STATUS_EMPTY = (uint8_t)0x01, // The RX/TX FIFO is empty
	NRF24_FIFO_STATUS_FULL  = (uint8_t)0x02, // The RX/TX FIFO is full
	NRF24_FIFO_STATUS_ERROR = (uint8_t)0x03  // Impossible state: RX/TX FIFO cannot be empty and full at the same time
}nrf24_fifo_status_t;

/**
* @brief nrf24_tx_result_t Result of the Transmitted Packet
*/
typedef enum {

	NRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	NRF24_TX_ERROR  , 				// Unknown error
	NRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	NRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nrf24_tx_result_t;

/**
* @brief nrf24_tx_result_t Result of the Transmitted Packet
*/
typedef enum {
	NRF24_RX_SUCCESS,                // Packet has been received successfully
	NRF24_RX_ERROR, 				// Unknown error
	NRF24_RX_EMPTY,
	NRF24_RX_TIMEOUT,                // It was timeout during packet receiving
} nrf24_rx_result_t;

/**
* @brief nrf24_handle_t
*/
typedef struct nrf24_handle_s
{

	uint8_t (*spi_init)(void);							     /**< point to an spi_init function address */
	uint8_t (*spi_deinit)(void);						  	 /**< point to an spi_deinit function address */
	uint8_t (*spi_read)(uint8_t *pData, uint16_t Size);		 /**< point to an spi_read_ function address */
	uint8_t (*spi_write)(uint8_t *pData, uint16_t Size);     /**< point to an spi_write function address */

	void (*ce_pin_write)(uint8_t pin_state);				 /**< point to an ce_pin_write function address */
	void (*csn_pin_write)(uint8_t pin_state);				 /**< point to an csn_pin_write function address */

    void (*delay_ms)(uint32_t ms);                           /**< point to a delay_ms function address */
    void (*debug_print)(const char *const fmt, ...);         /**< point to a debug_print function address */
    void (*receive_callback)(uint8_t irq_flag);              /**< point to a receive_callback function address */

	uint8_t RF_channel; 					/* channel - radio frequency channel, value from 0 to 125 */
	uint8_t address_width; 					/* RX/TX address field width, value from 3 to 5 */
	nrf24_tx_power_t tx_pwr; 				/* RF output power, one of NRF24_TXPWR_xx values */
	nrf24_data_rate_t data_rate; 			/* data rate, one of NRF24_DR_xx values */
	nrf24_crc_mode_t crc_mode; 		     	/* CRC scheme, one of NRF24_CRC_xx values */
	nrf24_ard_t ard;					    /* auto retransmit delay, value of NRF24_ARD_xx */
	uint8_t	 	arc;						/* auto retransmit count, between 0 and 15 */

}nrf24_handle_t;


/**
 * @}
 */



/**
 * @defgroup nrf24_link_driver nrf24 link driver function
 * @brief    nrf24 link driver modules
 * @ingroup  nrf24_driver
 * @{
 */

/**
 * @brief     initialize nrf24_handle_t structure
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] STRUCTURE is nrf24_handle_t
 * @note      none
 */
#define DRIVER_NRF24_LINK_INIT(HANDLE, STRUCTURE)         memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     link spi_init function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to an spi_init function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_SPI_INIT(HANDLE, FUC)           (HANDLE)->spi_init = FUC

/**
 * @brief     link spi_deinit function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to an spi_deinit function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_SPI_DEINIT(HANDLE, FUC)         (HANDLE)->spi_deinit = FUC

/**
 * @brief     link spi_read function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to an spi_read_write function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_SPI_READ(HANDLE, FUC)           (HANDLE)->spi_read = FUC

/**
 * @brief     link spi_write function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to an spi_read_write function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_SPI_WRITE(HANDLE, FUC)           (HANDLE)->spi_write = FUC

/**
 * @brief     link CE pin write function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to an CE_pin_write function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_CE_PIN_WRITE(HANDLE, FUC)           (HANDLE)->ce_pin_write = FUC

/**
 * @brief     link CSN pin write function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to an CSN_pin_write function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_CSN_PIN_WRITE(HANDLE, FUC)           (HANDLE)->csn_pin_write = FUC

/**
 * @brief     link delay_ms function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to a delay_ms function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_DELAY_MS(HANDLE, FUC)           (HANDLE)->delay_ms = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to a debug_print function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_DEBUG_PRINT(HANDLE, FUC)        (HANDLE)->debug_print = FUC

/**
 * @brief     link receive_callback function
 * @param[in] HANDLE points to an nrf24 handle structure
 * @param[in] FUC points to a receive_callback function address
 * @note      none
 */
#define DRIVER_NRF24_LINK_RECEIVE_CALLBACK(HANDLE, FUC)   (HANDLE)->receive_callback = FUC


/**
 * @}
 */


/**
 * @defgroup nrf24_driver nrf24 initialization and configuration driver function
 * @brief    nrf24 basic driver modules
 * @ingroup  nrf24_driver
 * @{
 */



/**
 * @brief      Init the NRF24 Module with the configuration pattern in the handler struct
 * @param[in] *handle pointer to the handler struct type
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t nrf24_init(nrf24_handle_t* handle);

 /**
  * @brief     irq handler
  * @return    status code
  *            - 0 success
  *            - 1 run failed
  *            - 2 handle is not initialized
  * @note      none
  */
 uint8_t nrf24_irq_handler(void);

/**
 * @brief      Reset the nrf24 transceiver chip to it's initial state
 * @param	   none
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note       RX/TX pipe addresses remains untouched
 */
uint8_t nrf24_chip_reset(void);

/**
 * @brief      Check if the nRF2424L01 present and available
 * @param	   none
 * @return     status code
 *             - 0 NRF24L01 is online and responding
 *             - 1 failed to communicate with the nrf24 module
 * @note
 */
uint8_t nrf24_check_available(void);

/**
 * @brief      Control transceiver power ON/OFF mode
 * @param	   mode - new state of power mode, one of NRF24_PWR_xx values
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note       the power must be set on before any operation!
 */
uint8_t nrf24_set_power_mode(nrf24_power_mode_t mode);


/**
 * @addtogroup Interrupt Masks (NRF24_MASKS_xx)
 * @brief    Interrupt Masks of the IRQ pin
 * @{
 */
#define 	NRF24_MASK_RX_DR			(0x40)
#define 	NRF24_MASK_TX_DS			(0x20)
#define 	NRF24_MASK_MAX_RT			(0x10)
/**
* }@
*/


/**
 * @brief      Sets the interrupt masks for the IRQ pin
 * @param[in]  masks - interrupt masks to be set, one or more of NRF24_MASKS_xx values
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note: By default Interrupts reflected on the IRQ pin , Masking for disabling the interrupt.
 * @note: The interrupt on IRQ pin is active low.
 */
uint8_t nrf24_set_interrupt_masks(uint8_t masks);

/**
 * @}
 *
 */



/**
 * @defgroup nrf24_driver nrf24 Status driver function
 * @brief    nrf24 basic driver modules
 * @ingroup  nrf24_driver
 * @{
 */


/**
 * @addtogroup Interrupt Flags (NRF24_FLAGS_xx)
 * @brief    Interrupt flags of the IRQ pin
 * @{
 */
#define NRF24_FLAG_RX_DR           (uint8_t)0x40 // DATA READY FLAG (data ready RX FIFO interrupt)
#define NRF24_FLAG_TX_DS           (uint8_t)0x20 // DATA SENT FLAG (data sent TX FIFO interrupt)
#define NRF24_FLAG_MAX_RT          (uint8_t)0x10 // MAX RETRANSMIT FLAG (maximum number of TX retransmits interrupt)
/**
* }@
*/

/**
 * @brief      Get pending IRQ flags
 * @param	   none
 * @return     Pending irq active flags:
 * 			   current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
 * @note: 	   none
 */
uint8_t nrf24_get_irq_flags(uint8_t* pending_flags);

/**
 * @brief      Clear any pending IRQ flags, RX_DR, TX_DS and MAX_RT bits of the STATUS register
 * @param	   on or more of NRF24_FLAGS_xx values
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note: 	   none
 */
uint8_t nrf24_clear_irq_flags(uint8_t flags);

/**
 * @brief      Get status of the RX FIFO
 * @param	   one of the NRF24_FIFO_STATUS_xx values
 * @return     none
 * @note: 	   none
 */
nrf24_fifo_status_t nrf24_get_rx_fifo_status(void);

/**
 * @brief      Get status of the TX FIFO
 * @param	   one of the NRF24_FIFO_STATUS_xx values
 * @return     none
 * @note: 	   none
 */
nrf24_fifo_status_t nrf24_get_tx_fifo_status(void);

/**
 * @brief      Get pipe number for the payload available for reading from RX FIFO
 * @param	   none
 * @return     pipe number, one value of NRF24_PIPEx
 * @note: 	   if value of 0x07 returned that means the RX FIFO is empty
 */
nrf24_pipe_t nrf24_get_rx_pipe_source(void);

/**
* @}
*/


/**
 * @defgroup nrf24_driver nrf24 Communication driver function
 * @brief    nrf24 basic driver modules
 * @ingroup  nrf24_driver
 * @{
 */


/**
 * @brief      Set the nRF24 in RX Mode or in TX Mode
 * @param	   mode : one of two modes NRF24_MODE_RX or NRF24_MODE_TX
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note: 	   none
 */
uint8_t nrf24_set_radio_mode(nrf24_radio_mode_t mode);

/**
 * @brief       Configure the ack with payload mode
 * @param       mode - one of NRF24_ACK_PAYLOAD_xx values
 * @return		 status code
 *             - 0 success
 *             - 1 operation failed
 * @note		enable or disable in two sides (PTX/PRX)
 */
uint8_t nrf24_set_ack_payload_mode(nrf24_ack_payload_mode_t mode);

/**
 * @brief      Open and configure transmitting Pipe
 * @param	   uint8_t* address: address of Transmitter
 * @param	   auto_ack_mode: one of the NRF24_AA_xx values
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note: 	   if auto ack is enabled pipe 0 is forced to have the same address
 */
uint8_t nrf24_open_tx_pipe(uint8_t* address, nrf24_auto_ack_mode_t auto_ack_mode);


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
							nrf24_dpl_mode_t dpl_mode , uint8_t payload_length);

/**
 * @brief    Close Receiving Pipe
 * @param    pipe : pipe to be closed
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note
 */
uint8_t nrf24_close_rx_pipe(nrf24_pipe_t pipe);

/**
 * @brief  	   : Function to transmit data packet in blocking mode
 * @param[IN]  void*  pBuf:		 pointer to the buffer with data to transmit
   @param[IN]  uint8_t	 length:	 length of the data buffer in bytes
   @param[IN]  uint32_t  timeOut:    time to wait if the TX FIFO is Full
 * @return	   nrf24_tx_result_t :	 one of nrf24_tx_result_t values
 * @note
 */
nrf24_tx_result_t nrf24_transmit_packet(void *p_data, uint8_t length, uint32_t time_out );

/**
 * @brief   		: Function to transmit data packet in interrupt mode
 * @param[IN]  void*  pBuf:		 pointer to the buffer with data to transmit
   @param[IN]  uint8_t	 length:	 length of the data buffer in bytes
 * @return	   		status code
 *
  * 				0	Success
  * 				1   nrf24 module hasn't initialized
  * 				2	invalid pointer to data
  * 				3	previous interrupt transmission still active
  * 				4	communication error
 * @note
 */
uint8_t nrf24_transmit_packet_it(void *p_data, uint8_t length);

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
uint8_t nrf24_send_ack_payload(nrf24_pipe_t pipe, uint8_t *payload, uint8_t length);


/**
 * @brief   						 Receive a data packet in blocking mode
 * @param[OUT]  uint8_t* pBuf : 	 buffer for the data,
 * @param[OUT]  uint8_t length :  	 length of the data
 * @param[IN]  uint32_t timeOut	 	 time to wait if the RX FIFO is empty
 * @return   nrf24_rx_result_t :	 one of the NRF24_RX_xx values
 * @note
 */
nrf24_rx_result_t nrf24_receive_packet(uint8_t *p_data, uint8_t* length, uint32_t time_out);

/**
 * @brief  			 Function to receive  data packet in interrupt mode
 * @param[IN]  uint8_t*  pBuf:		 pointer to the buffer with data to receive
   @param[IN]  uint8_t	 length:	 length of the data buffer in bytes
 * @return	   		status code
 *
  * 				0	Success
  * 				1   nrf24 module hasn't initialized
  * 				2	invalid pointer to data
  * 				3	previous interrupt transmission still active
  * 				4	communication error
 * @note
 */
uint8_t nrf24_receive_packet_it(uint8_t *p_data, uint8_t* length);
/**
 * @}
 */


/**
 * @defgroup nrf24_driver nrf24 Auxiliary driver function
 * @brief    nrf24 basic driver modules
 * @ingroup  nrf24_driver
 * @{
 */


/**
 * @brief    Flush the TX FIFO
 * @param    none
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note
 */
uint8_t nrf24_flush_tx_fifo(void);

/**
 * @brief    Flush the RX FIFO
 * @param    none
 * @return     status code
 *             - 0 success
 *             - 1 failed to communicate with the nrf24 module
 * @note
 */
uint8_t nrf24_flush_rx_fifo(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* MEDA_EXHAL_DRIVERS_COMMUNCATION_EXHAL_DRIVERS_INTERFACE_EXHAL_DRIVER_NRF2424_H_ */
