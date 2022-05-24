/*!
 * \file           sbgEComCmdOutput.h
 * \author         SBG Systems
 * \date           11 June 2014
 *
 * \brief          This file implements SbgECom commands related to outputs.
 *
 * \section CodeCopyright Copyright Notice
 * The MIT license
 *
 * Copyright (C) 2007-2020, SBG Systems SAS. All rights reserved.
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
 */
#ifndef SBG_ECOM_CMD_OUTPUT_H
#define SBG_ECOM_CMD_OUTPUT_H

// sbgCommonLib heaaders
//#include "../../common/sbgCommon.h"

// Project headers
//#include "../sbgECanId.h"
#include "../sbgEComIds.h"

//----------------------------------------------------------------------//
//- Public definitions												   -//
//----------------------------------------------------------------------//

/*!
 * List of ouput ports available.
 */
typedef enum _SbgEComOutputPort
{
	SBG_ECOM_OUTPUT_PORT_A = 0,				/*!< Main output port. */
	SBG_ECOM_OUTPUT_PORT_C = 2,				/*!< Secondary output port only available on Ellipse-E devices */
	SBG_ECOM_OUTPUT_PORT_E = 4				/*!< Secondary output port only available on B1 devices */
} SbgEComOutputPort;

/*!
 * List of output modes available.
 */
typedef enum _SbgEComOutputMode
{
	SBG_ECOM_OUTPUT_MODE_DISABLED 		= 0,		/*!< This output is disabled. */
	SBG_ECOM_OUTPUT_MODE_MAIN_LOOP 		= 1,		/*!< Output the message every main loop (ie 200 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_2 			= 2,		/*!< Output the message every 2 main loops (ie 100 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_4 			= 4,		/*!< Output the message every 4 main loops (ie 50 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_5 			= 5,		/*!< Output the message every 4 main loops (ie 40 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_8 			= 8,		/*!< Output the message every 8 main loops (ie 25 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_10 		= 10,		/*!< Output the message every 10 main loops (ie 20 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_20 		= 20,		/*!< Output the message every 20 main loops (ie 10 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_40 		= 40,		/*!< Output the message every 40 main loops (ie 5 Hz). */
	SBG_ECOM_OUTPUT_MODE_DIV_200 		= 200,		/*!< Output the message every 200 main loops (ie 1 Hz). */
	SBG_ECOM_OUTPUT_MODE_PPS 			= 10000,	/*!< Output the message on a Pulse Per Second event. */
	SBG_ECOM_OUTPUT_MODE_NEW_DATA 		= 10001,	/*!< Output sent when a new data is available. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_A		= 10003,	/*!< Output the message when a Sync A is received. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_B		= 10004,	/*!< Output the message when a Sync B is received. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_C		= 10005,	/*!< Output the message when a Sync C is received. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_D		= 10006,	/*!< Output the message when a Sync D is received. */
	SBG_ECOM_OUTPUT_MODE_EVENT_IN_E		= 10007,	/*!< Output the message when a Sync E is received. */
	SBG_ECOM_OUTPUT_MODE_HIGH_FREQ_LOOP = 20001		/*!< Output the message in the 1KHz IMU loop */
} SbgEComOutputMode;

/*!
 * Defines which monitoring point to use for an output port.
 * This feature enabled deporting measurements at a specific monitoring point.
 */
typedef enum _SbgEComOutputMonitoringPoint
{
	SBG_ECOM_OUTPUT_MONITORING_POINT_IMU	= 0,		/*!< Output measurements at the IMU location. */
	SBG_ECOM_OUTPUT_MONITORING_POINT_COG	= 1,		/*!< Output measurements at the center of rotation. */
	SBG_ECOM_OUTPUT_MONITORING_POINT_1		= 2,		/*!< Output measurements at the user deported location 1 (only for Ekinox and Apogee). */
	SBG_ECOM_OUTPUT_MONITORING_POINT_2		= 3,		/*!< Output measurements at the user deported location 2 (only for Ekinox and Apogee). */
	SBG_ECOM_OUTPUT_MONITORING_POINT_3		= 4,		/*!< Output measurements at the user deported location 3 (only for Ekinox and Apogee). */
	SBG_ECOM_OUTPUT_MONITORING_NUM						/*!< Number of output monitoring points. */
} SbgEComOutputMonitoringPoint;

#endif // SBG_ECOM_CMD_OUTPUT_H
