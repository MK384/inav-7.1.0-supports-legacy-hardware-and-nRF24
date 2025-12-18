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

#pragma once

#define TARGET_BOARD_IDENTIFIER "BLCKP"
#define USBD_PRODUCT_STRING  "BLACKPILL_F411"

#define LED0                    PC13

#define BEEPER                  PB1
#define BEEPER_INVERTED


// *************** UART *****************************

#define USE_VCP

#define USE_UART1
#define UART1_TX_PIN            PA15
#define UART1_RX_PIN            PB3

#define USE_UART2
#define UART2_TX_PIN            PA2
#define UART2_RX_PIN            PA3


#define SERIAL_PORT_COUNT       3

// *************** RX / NRF24 ********************

#define USE_SPI
#define USE_SPI_DEVICE_2
#define     RX_IRQ_PIN          PA9
#define     RX_CE_PIN           PA8
#define     RX_CSN_PIN          PB12
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15


#define USE_SPI_RX
#define DEFAULT_RX_TYPE         RX_TYPE_SPI
#define RX_SPI_INSTANCE            SPI2 

#define USE_NRF24RX_INAV
#define RX_SPI_DEFAULT_PROTOCOL     NRF24RX_INAV

#define		CONFIG_RX_TX_FREQUENCY_CHANNEL					115
#define		CONFIG_RX_TX_ADDRESS_LEN						5
#define		CONFIG_RX_TX_ADDRESS  						   "INAV7"
#define	 	CONFIG_RX_TX_BAUD_RATE							NRF24_DR_1Mbps
#define		CONFIG_RX_TX_AURO_RETRANSMIT_DELAY				NRF24_ARD_1000us
#define		CONFIG_RX_TX_AURO_RETRANSMIT_COUNT				5
#define		CONFIG_RX_TX_CRC_MODE							NRF24_CRC_2byte
#define     CONFIG_RX_TX_POWER								NRF24_TXPWR_0dBm

#define USE_TELEMETRY_NRF24_LTM
// ********************* Black Box *************************
#define USE_SPI
#define USE_SPI_DEVICE_1

#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SDCARD
#define USE_SDCARD_SPI
#define SDCARD_SPI_BUS          BUS_SPI1
#define SDCARD_CS_PIN           PA4
#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// *************** OpticalFlow (ADNS3080) ********************

#define     USE_OPFLOW
#define     USE_OPFLOW_ADNS3080
#define     OPFLOW_SPI_INSTANCE     SPI1 

#define     OPFLOW_RST_PIN          PB2 
#define     OPFLOW_NCS_PIN          PB10

// *************** I2C /Baro/Mag/Pitot ********************
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9

#define DEFAULT_I2C_BUS         BUS_I2C1

#define USE_IMU_MPU6050
#define IMU_MPU6050_ALIGN       CW90_DEG
#define MPU6050_I2C_BUS         BUS_I2C1


#define USE_BARO
#define BARO_I2C_BUS            BUS_I2C1
#define USE_BARO_BMP280


#define USE_MAG
#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG_HMC5883

#define MAG_HMC5883_ALIGN CW90_DEG_FLIP

// --------- Rangefinder (VL53L1X) ----------
#define USE_RANGEFINDER
#define USE_RANGEFINDER_VL53L1X
#define RANGEFINDER_I2C_BUS    BUS_I2C1

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                ADC1
#define ADC1_DMA_STREAM             DMA2_Stream0

#define ADC_CHANNEL_1_PIN           PB0

#define VBAT_ADC_CHANNEL            ADC_CHN_1

// *************** LED2812 ************************
// #define USE_LED_STRIP
// #define WS2811_PIN                  PA8

// *************** PINIO ***************************
#define USE_PINIO
// #define USE_PINIOBOX
// #define PINIO1_PIN                  PA13 // Camera switcher

// ***************  OTHERS *************************

#define DEFAULT_FEATURES        (FEATURE_TX_PROF_SEL | FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_TELEMETRY | FEATURE_GPS | FEATURE_BLACKBOX)


#define USE_DSHOT
#define USE_ESC_SENSOR
#define USE_SERIAL_4WAY_BLHELI_INTERFACE


#define TARGET_IO_PORTA         0xffff
#define TARGET_IO_PORTB         0xffff
#define TARGET_IO_PORTC         0xffff
#define TARGET_IO_PORTD         (BIT(2))

#define MAX_PWM_OUTPUT_PORTS       8
