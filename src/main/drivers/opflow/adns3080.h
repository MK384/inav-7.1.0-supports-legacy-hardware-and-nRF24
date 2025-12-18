/**
 * File Name: adns3080.h
 * Created On: 2024-02-22
 * Author: Mohammed Khaled
 * Description:
 * This driver, ADNS3080 , is designed for image stabilization, and motion detection on the flight controller board.
 * It provides functions and definitions necessary for initializing, and controlling the Optical Flow.
 * License:
 * This software is open and free to use, modify, and integrate into other projects under the name of our team (MEDA Team) and our university (Fayoum University).
 */

#pragma once
/* Includes ---------------------------------------------------------------  */
#include <stdint.h>
#include "opflow.h"
// Add any additional includes here as needed
/* Macros ---------------------------------------------------------------  */
/* Defined Types -------------------------------------------------------------------------*/

// Type for Controlling the Resolution in counts per inch
typedef enum{

	Optical_400_CPI	 = 0x00,	// 400 counts per inch resolution
	Optical_1600_CPI = 0x10	// 160000 counts per inch resolution

}ADNS3080_Res_t;


// Type for Controlling the AGC (Automatic Gain Control)
typedef enum{

	Optical_AGC_On   = 0x00,	// the chip automatic adjust the shutter
	Optical_AGC_Off	 = 0x02// Shutter will be set to the value in the Shutter_Max_Bound registers.

}ADNS3080_AGC_t;


//Type for Controlling the frame rate config
typedef enum{

	Optical_FR_Auto   = 0x00 ,		// the chip automatic adjust the frame rate
	Optical_FR_Fixed  = 0x01,	//the frame rat will be determined by the value in the Frame_Period_Max_Bound registers.

}ADNS3080_FR_t;



/**
 * @defgroup Motions Bits Masks
 * @brief    defines the bits of the motion register in OpticalFlow_Motion_t
 * @{
 */

#define 	OF_MOTION_VALID_BIT		0x80				// If this bit is set the motion dx and dy is valid counts
#define 	OF_MOTION_OVF_BIT		0x10			    // If this bit is set there is overflow
#define 	OF_MOTION_RES_BIT		0x01				// If this bit is set the Resolution is 400 CPI otherwise 1600 CPI


/**
 * }@
 */

// struct type for Motion Detection
typedef struct
{
 uint8_t motion;		  	// see @defgroup Motions Bits Masks
 int8_t dx, dy;				// counts in x and y direction
 uint8_t squal;				// Surface Quality measure
 uint16_t shutter;			// clock cylces for the shutter
 uint8_t max_pixel;		    // maximum pixel in the frame

}ADNS3080_Motion_t;


/*
 * the three bound registers must follow this rule when set to non-default values:
 *	Frame_Period_Max_Bound ≥ Frame_Period_Min_Bound + Shutter_Max_Bound.
 * */
// struct type for configuration
typedef struct{

	ADNS3080_Res_t res;
	ADNS3080_AGC_t shutter_mode;
	ADNS3080_FR_t fr_mode;

	uint16_t frame_period_max;
	uint16_t frame_period_min;
	uint16_t shutter_max_bound;

}ADNS3080_Config_t;

#define     ADNS3080_RES_CPI        Optical_1600_CPI
/** Function Prototypes --------------------------------------------------------- */

/**
 * @brief Initializes the ADNS3080 sensor.
 *
 * This function initializes the SPI communication, resets the sensor,
 * checks the product ID, and sets the sensor to sensitive mode.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool ADNS3080_Init(void);

/**
 * @brief Resets the ADNS3080 sensor.
 *
 * This function resets the ADNS3080 sensor by toggling the reset pin.
 *
 */
void ADNS3080_Reset(void);

/**
 * @brief Captures a frame from the ADNS3080 sensor.
 *
 * This function captures a frame from the ADNS3080 sensor and stores it in the provided buffer.
 *
 * @param pdata Pointer to an array where the frame data will be stored.
 * @return 0 if the frame capture is successful, -1 otherwise.
 */
int ADNS3080_FrameCapture(uint8_t*);

/**
 * @brief Reads motion data from the ADNS3080 sensor.
 *
 * This function reads motion data from the ADNS3080 sensor and stores it in the provided structure.
 *
 * @param p Pointer to a struct where the motion data will be stored.
 */
void ADNS3080_getMotion(ADNS3080_Motion_t*);
