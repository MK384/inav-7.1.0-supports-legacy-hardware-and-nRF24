/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// This file is copied with modifications from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <platform.h>

#include "nrf24.h"
#include "build/build_config.h"

#include "drivers/time.h"
#include "drivers/io.h"
#include "io_impl.h"
#include "rcc.h"
#include "rx_spi.h"
#include "exti.h"
#include "nvic.h"
#include "drivers/bus_spi.h"


static extiCallbackRec_t nrfExtiCllbackRec = {0};
static void nrf24HandlerCallback(extiCallbackRec_t* rec){(void)(rec); nrf24_irq_handler();}


bool rxSpiDeviceInit(void)
{
    static bool hardwareInitialised = false;

    if (hardwareInitialised) {
        return true;
    }

#ifdef RX_CSN_PIN
    IOInit(IOGetByTag(IO_TAG(RX_CSN_PIN)), OWNER_RX, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(IOGetByTag(IO_TAG(RX_CSN_PIN)), IOCFG_OUT_PP);
    IOHi(IOGetByTag(IO_TAG(RX_CSN_PIN)));
#endif

#ifdef RX_IRQ_PIN
    IO_t rxIrqPin = IOGetByTag(IO_TAG(RX_IRQ_PIN));
    IOInit(rxIrqPin, OWNER_RX, RESOURCE_EXTI, 0);
    IOConfigGPIO(rxIrqPin, IOCFG_IPU);

    EXTIHandlerInit(&nrfExtiCllbackRec, nrf24HandlerCallback);
    EXTIConfig(rxIrqPin ,&nrfExtiCllbackRec, NVIC_PRIO_MAX, EXTI_Trigger_Falling);
    EXTIEnable(rxIrqPin ,true);
#endif

#ifdef RX_CE_PIN
    // CE as OUTPUT
    IOInit(IOGetByTag(IO_TAG(RX_CE_PIN)), OWNER_RX, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(IOGetByTag(IO_TAG(RX_CE_PIN)), IOCFG_OUT_PP);
    IOLo(IOGetByTag(IO_TAG(RX_CE_PIN)));
#endif // RX_CE_PIN


#ifdef  RX_SPI_INSTANCE
    const SPIDevice rxSPIDevice = spiDeviceByInstance(RX_SPI_INSTANCE);
    if (spiInitDevice(rxSPIDevice, true) == false){
        return false;
    } 
    spiSetSpeed(OPFLOW_SPI_INSTANCE, SPI_CLOCK_ABOVE_MEDIUM);

#endif

    hardwareInitialised = true;
    return true;
}

