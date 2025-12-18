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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/maths.h"

#include "platform.h"


#include "build/build_config.h"
#include "build/debug.h"
#include "common/utils.h"


#include "drivers/nrf24.h"
#include "drivers/nrf24_interface.h"

#include "drivers/time.h"
#include "drivers/rx_spi.h"

#include "rx/rx.h"
#include "rx/nrf24_inav.h"

#include "telemetry/ltm.h"

/*
 * iNav Protocol
 * Data rate is 250Kbps - lower data rate for better reliability and range
 *
 * Uses auto acknowledgment and dynamic payload size
 *     ACK payload is used for handshaking in bind phase and telemetry in data phase
 *
 * Bind payload size is 16 bytes
 * Data payload size is 8, 16 or 18 bytes dependent on variant of protocol, (small payload is read more quickly (marginal benefit))
 *
 * Bind Phase
 * uses address {0x4b,0x5c,0x6d,0x7e,0x8f}
 * uses channel 0x4c (76)
 *
 * Data Phase
 * 1) Uses the address received in bind packet
 *
 * 2) Hops between RF channels generated from the address received in bind packet.
 *    The number of RF hopping channels is set during bind handshaking:
 *        the transmitter requests a number of bind channels in payload[7]
 *        the receiver sets ackPayload[7] with the number of hopping channels actually allocated - the transmitter must
 *        use this value.
 *    All receiver variants must support the 16 byte payload. Support for the 8 and 18 byte payload is optional.
 *
 * 3) Uses the payload size negotiated in the bind phase, payload size may be 8, 16 or 18 bytes
 * a) For 8 byte payload there are 6 channels: AETR with resolution of 1 (10-bits are used for the channel data), and AUX1
 *    and AUX2 with resolution of 4 (8-bits are used for the channel data)
 * b) For 16 byte payload there are 16 channels: eight 10-bit analog channels, two 8-bit analog channels, and six digital channels as follows:
 *    Channels 0 to 3, are the AETR channels, values 1000 to 2000 with resolution of 1 (10-bit channels)
 *    Channel AUX1 by deviation convention is used for rate, values 1000, 1500, 2000
 *    Channels AUX2 to AUX6 are binary channels, values 1000 or 2000,
 *        by deviation convention these channels are used for: flip, picture, video, headless, and return to home
 *    Channels AUX7 to AUX10 are analog channels, values 1000 to 2000 with resolution of 1 (10-bit channels)
 *    Channels AUX11 and AUX12 are analog channels, values 1000 to 2000 with resolution of 4 (8-bit channels)
 * c) For 18 byte payload there are 18 channels, the first 16 channelsar are as for 16 byte payload, and then there are two
 *    additional channels: AUX13 and AUX14 both with resolution of 4 (8-bit channels)
 *
 * Intercepting packets
 *
 * Packets are designed to be intercepted by a second receiver. So a second receiver could, for example intercept the
 * ACK packets and use the GPS telemetry to display the position of the aircraft on a map, or to control a camera gimbal
 * to point at the aircraft.
 */

#define NRF24_MAX_CHANNEL 16 // up to 16 RC channels are supported
#define NRF24_AUX_CHANNEL_COUNT   12
#define NRF24L01_MAX_PAYLOAD_SIZE 32

#define PKTS_IN_RSSI_SLIDING_WINDOW         25      // sliding window of 20 packets -> Rssi updated every 25 * 20ms = 500 ms

static rxRuntimeConfig_t *rxRuntimeCnfg;
static nrf24_handle_t nrf24_handler = {0};

static uint16_t nrf24ChannelData[NRF24_MAX_CHANNEL];
static uint8_t ackPayload[NRF24L01_MAX_PAYLOAD_SIZE];

static volatile bool frame_recieved = false;

typedef struct {

	uint16_t CH_Aileron;
	uint16_t CH_Elevator;
	uint16_t CH_Throttle;
	uint16_t CH_Rudder;
	uint16_t CH_Aux;
    // Protocol purposes
    uint8_t PKT_Counter;
    uint8_t LTM_Period;

}__attribute__((__packed__)) nrf24_frame_t;

static nrf24_frame_t nrf24_frame = {
		.CH_Aileron  = 1500,
		.CH_Elevator = 1500,
		.CH_Throttle = 1000,
		.CH_Rudder   = 1500,
		.CH_Aux      = 0U,
        .PKT_Counter = 0,
        .LTM_Period  = 100U,
};
#define   NRF24_FRAME_SIZE     sizeof(nrf24_frame_t)

    static uint8_t recPackets = 0;
    static uint8_t lastPktCnt = 0;

static void writeTelemetryAckPayload(void)
{
#ifdef USE_TELEMETRY_NRF24_LTM
    // set up telemetry data, send back telemetry data in the ACK packet
    static ltm_frame_e ltmFrameType = LTM_FRAME_START;
    const int ackPayloadSize = getLtmFrame(ackPayload, ltmFrameType);
    ++ltmFrameType;
    if (ltmFrameType == LTM_FRAME_COUNT) {
        ltmFrameType = LTM_FRAME_START;
    }
    nrf24_send_ack_payload(NRF24_PIPE1, ackPayload,  ackPayloadSize);
#endif
}

void nrf24_callback_receiver(uint8_t irq_flag)
{
    if(irq_flag == NRF24_FLAG_RX_DR){
        frame_recieved = true;
        recPackets++;
    }
}

static uint16_t nrf24ChannelsReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }
    // Use full range of values (11 bit, channel values)
    return constrain(rxRuntimeConfig->channelData[chan], 0, 2047);
}
void nrf24ChannelsInit(rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfig->rcReadRawFn = nrf24ChannelsReadRawRC;
    for (int b = 0; b < NRF24_MAX_CHANNEL; b++) {
        rxRuntimeConfig->channelData[b] = (16 * PWM_RANGE_MIDDLE) / 10 - 1408;
    }
}
static uint8_t decodeNRF24Frame(rxRuntimeConfig_t *rxRuntimeConfig){
    
    static timeMs_t tick = 0;
    uint8_t length = 0;
    nrf24_rx_result_t result = nrf24_receive_packet((uint8_t*)&nrf24_frame, &length, 0);
    if (result != NRF24_RX_SUCCESS || length != NRF24_FRAME_SIZE)
    {
        // Frame is not valid
        return RX_FRAME_DROPPED;
    }

    // We use AETR mapping
    nrf24ChannelData[0]  = nrf24_frame.CH_Aileron;
    nrf24ChannelData[1]  = nrf24_frame.CH_Elevator;
    nrf24ChannelData[2]  = nrf24_frame.CH_Throttle;
    nrf24ChannelData[3]  = nrf24_frame.CH_Rudder;
    for (size_t ix = 4; ix < NRF24_MAX_CHANNEL; ix++)
    {
        nrf24ChannelData[ix] = (nrf24_frame.CH_Aux & (1 << ix))? 2000 : 1000;
    }

    if (recPackets >= PKTS_IN_RSSI_SLIDING_WINDOW){
        uint8_t pcktlost = (nrf24_frame.PKT_Counter - lastPktCnt) - recPackets;
        uint8_t pcktArrived = recPackets - constrain(pcktlost, 0, recPackets);
        lqTrackerSet(rxRuntimeConfig->lqTracker, scaleRange(pcktArrived, 0, recPackets, 0, RSSI_MAX_VALUE));
        recPackets = 0;
        lastPktCnt = nrf24_frame.PKT_Counter;
    }

    if (millis() - tick > nrf24_frame.LTM_Period){
        writeTelemetryAckPayload();
        tick = millis();
    }

    // Reset the frameDone flag - tell ISR that we're ready to receive next frame
    frame_recieved = false;
    return RX_FRAME_COMPLETE;
}

static uint8_t nrf24FrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    if (!frame_recieved) {
        return RX_FRAME_PENDING;
    }

    return decodeNRF24Frame(rxRuntimeConfig); // decode the new frame and return the status
}

static uint8_t inavNrf24Setup(void)
{

	  DRIVER_NRF24_LINK_INIT				(&nrf24_handler, nrf24_handle_t);
	  DRIVER_NRF24_LINK_CE_PIN_WRITE		(&nrf24_handler, nrf24_interface_ce_pin_write);
	  DRIVER_NRF24_LINK_CSN_PIN_WRITE		(&nrf24_handler, nrf24_interface_csn_pin_write);
	  DRIVER_NRF24_LINK_DELAY_MS			(&nrf24_handler, nrf24_interface_delay_ms);
	  DRIVER_NRF24_LINK_RECEIVE_CALLBACK	(&nrf24_handler, nrf24_interface_receive_callback);
	  DRIVER_NRF24_LINK_SPI_READ			(&nrf24_handler, nrf24_interface_spi_read);
	  DRIVER_NRF24_LINK_SPI_WRITE			(&nrf24_handler, nrf24_interface_spi_write);

	  nrf24_handler.RF_channel   	    	= 		CONFIG_RX_TX_FREQUENCY_CHANNEL;
	  nrf24_handler.address_width  	        =	    CONFIG_RX_TX_ADDRESS_LEN;
	  nrf24_handler.tx_pwr 		            = 	    CONFIG_RX_TX_POWER;
	  nrf24_handler.data_rate 		        = 		CONFIG_RX_TX_BAUD_RATE;
	  nrf24_handler.crc_mode				= 		CONFIG_RX_TX_CRC_MODE;
	  nrf24_handler.ard 					= 	    CONFIG_RX_TX_AURO_RETRANSMIT_DELAY;
	  nrf24_handler.arc 					=		CONFIG_RX_TX_AURO_RETRANSMIT_COUNT;
    
	  if (nrf24_init(&nrf24_handler))                                                  return 0;
	  if(nrf24_set_ack_payload_mode(NRF24_ACK_PAYLOAD_ON))                             return 0;
      if(nrf24_set_interrupt_masks(NRF24_MASK_MAX_RT|NRF24_MASK_TX_DS))                return 0;
	  if (nrf24_open_rx_pipe( NRF24_PIPE1 ,(uint8_t*)CONFIG_RX_TX_ADDRESS, NRF24_AUTO_ACK_ON,
							NRF24_DPL_ON, 0))                           return 0;
	  if (nrf24_set_power_mode(NRF24_PWR_UP))                                          return 0;                            
	  if (nrf24_set_radio_mode(NRF24_MODE_RX))              	                       return 0;

	  
	  return 1;
}


bool inavNrf24Init(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxConfig);
    rxRuntimeCnfg = rxRuntimeConfig;

    rxRuntimeCnfg->channelData = nrf24ChannelData;

    nrf24ChannelsInit(rxRuntimeCnfg);

    rxRuntimeCnfg->channelCount = NRF24_MAX_CHANNEL;

    rxRuntimeCnfg->rcFrameStatusFn = nrf24FrameStatus;

    if (rxSpiDeviceInit() == false) return false; // Initialize the spi device and the related hardware 
    
    if (inavNrf24Setup() == false)  return false;

    return true;
}

