/*!
 * \file           sbgEComCmdDvl.h
 * \author         SBG Systems
 * \date           13 December 2018
 *
 * \brief          This file implements sbgECom commands related to DVL module.
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

#ifndef SBG_ECOM_CMD_DVL_H
#define SBG_ECOM_CMD_DVL_H

/* sbgCommonLib headers */
//#include <sbgCommon.h>

/* Local headers */
#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * This enum defines the different DVL model IDs available in standard
 */
typedef enum _SbgEComDvlModelsIds
{
	SBG_ECOM_DVL_MODEL_GENERIC_PD6	= 202,		/*!< Generic DVL using PD6 protocol format. */
} SbgEComDvlModelsIds;

/*!
 * DVL mechanical installation parameters such as lever arm and alignment
 */
typedef struct _SbgEComDvlInstallation
{
	float	leverArm[3];						/*!< X, Y, Z DVL lever arm in meters expressed from the DVL to the IMU. */
	float	alignment[3];						/*!< Roll, pitch, yaw DVL alignment expressed in radians. */
	bool	preciseInstallation;				/*!< Set to true if both the DVL lever arm and DVL alignment are precise and don't require in-run estimation. */
} SbgEComDvlInstallation;

/*!
 * Holds all necessary information for DVL module data rejection.
 */
typedef struct _SbgEComDvlRejectionConf
{
	SbgEComRejectionMode	bottomLayer;		/*!< Rejection mode for the bottom tracking (ie when the velocity  measurement is in respect to the seabed). */
	SbgEComRejectionMode	waterLayer;			/*!< Rejection mode for the water tracking (ie when the velocity measurement is relative to a water layer). */
} SbgEComDvlRejectionConf;

#endif
