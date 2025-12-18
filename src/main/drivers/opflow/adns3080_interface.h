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
 * @file      adn3080_interface.h
 * @brief     driver interface file for ADNS3080
 * @version   1.0.0
 * @author    Mohammed Khaled
 * @date      2024-April-15
 */


#pragma once


/*------ Includes ---------------------------------------------------------------*/
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "stm32f4xx.h"

/*------ Defines ---------------------------------------------------------------*/

/**
 * @defgroup adns3080_interface driver functions
 * @brief    adns3080 interface driver modules
 * @ingroup  adns3080_driver
 * @{
 */

/**
 * @brief  interface io and bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */

static bool isInterfaceInited = false;

bool adns3080_interface_init(void){

if (isInterfaceInited)  return true;

#ifdef OPFLOW_NCS_PIN
    IOInit(IOGetByTag(IO_TAG(OPFLOW_NCS_PIN)), OWNER_OPFLOW, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(IOGetByTag(IO_TAG(OPFLOW_NCS_PIN)), IOCFG_OUT_PP);
    IOHi(IOGetByTag(IO_TAG(OPFLOW_NCS_PIN)));
#endif

#ifdef OPFLOW_RST_PIN
    IOInit(IOGetByTag(IO_TAG(OPFLOW_RST_PIN)), OWNER_OPFLOW, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(IOGetByTag(IO_TAG(OPFLOW_RST_PIN)), IOCFG_OUT_PP);
    IOLo(IOGetByTag(IO_TAG(OPFLOW_RST_PIN)));
#endif

    isInterfaceInited = true;
    return true;
}


/**
 * @brief     interface spi bus write

 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - true: success
 *            - false: write failed
 * @note      none
 */

bool adns3080_interface_spi_write(const uint8_t *buf, uint16_t len){

#ifdef  OPFLOW_SPI_INSTANCE
    for (size_t idx = 0; idx < len; idx++)
        spiTransferByte(OPFLOW_SPI_INSTANCE, buf[idx]);
#endif
    return true;
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
bool adns3080_interface_spi_read(uint8_t *buf, uint16_t len){

#ifdef  OPFLOW_SPI_INSTANCE
	for (size_t idx = 0; idx < len; idx++)
       buf[idx] = spiTransferByte(OPFLOW_SPI_INSTANCE, 0xff);
#endif
    return true;
}

/**
 * @brief     interface delay us
 * @param[in] us
 * @note      none
 */
void adns3080_interface_delay_us(uint32_t us){

	uint32_t time_snap = micros();

	while (micros() - time_snap < us)
		;
}

/**
 * @brief     interface NCS pin write
 * @param[in] level 1 for high 0 for low
 * @note      none
 */
void adns3080_interface_ncs_pin_write(uint8_t level){

    static SPIClockSpeed_e spiSpeed  = SPI_CLOCK_STANDARD;
#ifdef  OPFLOW_SPI_INSTANCE    
const SPIDevice opSPIDevice = spiDeviceByInstance(OPFLOW_SPI_INSTANCE);
if (level == 0){ 
    spiSpeed = spiGetSpeed(OPFLOW_SPI_INSTANCE);
    if (spiInitDevice(opSPIDevice, false) == false) return;
    spiSetSpeed(OPFLOW_SPI_INSTANCE, SPI_CLOCK_SLOW);
} else {
    if (spiInitDevice(opSPIDevice, true) == false)  return;
    spiSetSpeed(OPFLOW_SPI_INSTANCE, spiSpeed != SPI_CLOCK_UNKOWN? spiSpeed: SPI_CLOCK_STANDARD);
}
#endif    
#ifdef  OPFLOW_NCS_PIN
		IOWrite(IOGetByTag(IO_TAG(OPFLOW_NCS_PIN)), (bool)level);
#endif        
}

/**
 * @brief     interface RST pin write
 * @param[in] level 1 for high 0 for low
 * @note      none
 */
void adns3080_interface_rst_pin_write(uint8_t level){

#ifdef  OPFLOW_RST_PIN
		IOWrite(IOGetByTag(IO_TAG(OPFLOW_RST_PIN)), (bool)level);
#endif        
}

/**
 * @}
 */