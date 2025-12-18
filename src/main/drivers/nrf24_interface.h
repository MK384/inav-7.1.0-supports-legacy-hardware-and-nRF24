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
 * @file      exhal_driver_interface_nrf24.h
 * @brief     driver nrf24 interface file
 * @version   1.0.0
 * @author    Mohammed Khaled
 * @date      2024-April-15
 */

#ifndef EXHAL_DRIVER_INTERFACE_NRF24_H_
#define EXHAL_DRIVER_INTERFACE_NRF24_H_


/*------ Includes ---------------------------------------------------------------*/
#include "nrf24.h"
#include "bus_spi.h"
#include "io.h"
#include "time.h"
#include "string.h"
#include "stm32f4xx.h"
#ifdef __cplusplus
extern "C"{
#endif

/*------ external References ---------------------------------------------------------------*/


extern void nrf24_callback_receiver(uint8_t);


/*------ Defines ---------------------------------------------------------------*/

/**
 * @defgroup nrf24_interface_driver nrf24 interface driver function
 * @brief    nrf24 interface driver modules
 * @ingroup  nrf24_driver
 * @{
 */

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t nrf24_interface_spi_init(void){

	return 0;
}


/**
 * @brief  interface dma bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t nrf24_interface_dma_init(void){

	return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t nrf24_interface_spi_deinit(void){

	return 0;
}

/**
 * @brief  interface dma deinit
 * @return status code
 *         - 0 success
 *         - 1 dma deinit failed
 * @note   none
 */
uint8_t nrf24_interface_dma_deinit(void){

	return 0;
}


/**
 * @brief     interface spi bus write

 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t nrf24_interface_spi_write(uint8_t *buf, uint16_t len){

#ifdef  RX_SPI_INSTANCE
	for (size_t idx = 0; idx < len; idx++)
        spiTransferByte(RX_SPI_INSTANCE, buf[idx]);
#endif
    return 0;
}

/**
 * @brief     interface spi bus read

 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t nrf24_interface_spi_read(uint8_t *buf, uint16_t len){

#ifdef  RX_SPI_INSTANCE
	for (size_t idx = 0; idx < len; idx++)
       buf[idx] = spiTransferByte(RX_SPI_INSTANCE, 0xff);
#endif
    return 0;
}

/**
 * @brief     interface spi bus dma write

 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t nrf24_interface_spi_dma_write(uint8_t *buf, uint16_t len){

    UNUSED(buf);
    UNUSED(len);
	return 0;
}


/**
 * @brief     interface wait on dma

 * @note      none
 */
void nrf24_interface_wait_on_dma(void){

	while(0)
    ;
}

/**
 * @brief     interface delay ms
 * @param[in] us
 * @note      none
 */
void nrf24_interface_delay_ms(uint32_t ms){

	uint32_t us = ms * 1000;
	uint32_t time_snap = micros();

	while (micros() - time_snap < us)
		;
}

/**
 * @brief     interface receive callback
 * @param[in] irq flags
 * @note      none
 */
void nrf24_interface_receive_callback(uint8_t irq_type){

	nrf24_callback_receiver(irq_type);

}


/**
 * @brief     interface CE pin write
 * @param[in] level 1 for high 0 for low
 * @note      none
 */
void nrf24_interface_ce_pin_write(uint8_t level){

	IOWrite(IOGetByTag(IO_TAG(RX_CE_PIN)), (bool)level);

}

/**
 * @brief     interface CSN pin write
 * @param[in] level 1 for high 0 for low
 * @note      none
 */
void nrf24_interface_csn_pin_write(uint8_t level){

		IOWrite(IOGetByTag(IO_TAG(RX_CSN_PIN)), (bool)level);
}



/**
 * @}
 */

#ifdef __cplusplus
}
#endif





#endif /* MEDA_EXHAL_INTERFACES_INC_EXHAL_DRIVER_INTERFACE_NRF24_H_ */
