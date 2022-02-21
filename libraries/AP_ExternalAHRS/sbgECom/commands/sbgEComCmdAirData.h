/*!
 * \file           sbgEComCmdAirData.h
 * \author         SBG Systems
 * \date           18 February 2019
 *
 * \brief          This file implements sbgECom commands related to AirData module.
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

#ifndef SBG_ECOM_CMD_AIR_DATA_H
#define SBG_ECOM_CMD_AIR_DATA_H

// sbgCommonLib headers
//#include <sbgCommon.h>

// Local headers
#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * This enum defines the different AirData model IDs available in standard
 */
typedef enum _SbgEComAirDataModelsIds
{
	SBG_ECOM_AIR_DATA_MODEL_INTERNAL		= 1,		/*!< Use the internal barometer sensor if available. */
	SBG_ECOM_AIR_DATA_MODEL_GENERIC_ECOM	= 2,		/*!< Generic AirData model using sbgECom input protocol format. */
	SBG_ECOM_AIR_DATA_MODEL_AHRS_500		= 3,		/*!< Crossbow AHRS-500 compatible input for barometric altitude and airspeed. */
} SbgEComAirDataModelsIds;

/*!
 * Holds all necessary information for AirData module data rejection.
 */
typedef struct _SbgEComAirDataRejectionConf
{
	SbgEComRejectionMode	airspeed;					/*!< Rejection mode for the true air speed measurement. */
	SbgEComRejectionMode	altitude;					/*!< Rejection mode for the barometric altitude measurement. */
} SbgEComAirDataRejectionConf;

#endif
