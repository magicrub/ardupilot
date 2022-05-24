/*!
 * \file           sbgEComCmdGnss.h
 * \author         SBG Systems
 * \date           11 June 2014
 *
 * \brief          This file implements SbgECom commands related to GNSS module.
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

#ifndef SBG_ECOM_CMD_GNSS_H
#define SBG_ECOM_CMD_GNSS_H

// sbgCommonLib headers
//#include "../../common/sbgCommon.h"/sbgCommon.h"/sbgCommon.h"

// Local headers
#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * This enum defines the different GNSS model IDs available in standard
 */
typedef enum _SbgEComGnssModelsStdIds
{
	SBG_ECOM_GNSS_MODEL_INTERNAL					= 101,		/*!< Default internal GNSS for ELLIPSE-N and ELLIPSE-D */
	SBG_ECOM_GNSS_MODEL_NMEA						= 102,		/*!< ELLIPSE-E to accept an external GNSS using NMEA protocol */
	SBG_ECOM_GNSS_MODEL_UBLOX_GPS_BEIDOU			= 103,		/*!< Only for ELLIPSE-N hardware 1 & 2 to select GPS+BEIDOU instead of the default GPS+GLONASS */
	SBG_ECOM_GNSS_MODEL_UBLOX_EXTERNAL				= 104,		/*!< ELLIPSE-E to accept an external Ublox GNSS (receive only - passive) */
	SBG_ECOM_GNSS_MODEL_RESERVED_01					= 105,		/*!< Reserved, do not use */
	SBG_ECOM_GNSS_MODEL_NOVATEL_EXTERNAL			= 106,		/*!< ELLIPSE-E to accept an external Novatel GNSS (receive only - passive) */
	SBG_ECOM_GNSS_MODEL_RESERVED_02					= 107,		/*!< Reserved, do not use */
	SBG_ECOM_GNSS_MODEL_RESERVED_03					= 108,		/*!< Reserved, do not use */
	SBG_ECOM_GNSS_MODEL_SEPTENTRIO_EXTERNAL			= 109,		/*!< ELLIPSE-E to accept an external Septentrio GNSS(receive only - passive) */
	SBG_ECOM_GNSS_MODEL_RESERVED_04					= 110		/*!< Reserved, do not use */
} SbgEComGnssModelsStdIds;

/*!
 * GNSS mechanical installation modes for the dual antenna mode.
 */
typedef enum _SbgEComGnssInstallationMode
{
	SBG_ECOM_GNSS_INSTALLATION_MODE_SINGLE			= 1,		/*!< The GNSS will be used in single antenna mode only and the secondary lever arm is not used. */
	SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_AUTO		= 2,		/*!< [Reserved] The GNSS dual antenna information will be used but the secondary lever arm is not known. */
	SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_ROUGH		= 3,		/*!< The GNSS dual antenna information will be used and we have a rough guess for the secondary lever arm. */
	SBG_ECOM_GNSS_INSTALLATION_MODE_DUAL_PRECISE	= 4			/*!< The GNSS dual antenna information will be used and the secondary lever arm is accurately entered and doesn't need online re-estimation. */
} SbgEComGnssInstallationMode;

/*!
 * GNSS mechanical installation parameters to be used with command SBG_ECOM_CMD_GNSS_#_INSTALLATION
 */
typedef struct _SbgEComGnssInstallation
{
	float						leverArmPrimary[3];				/*!< GNSS primary antenna lever arm in IMU X, Y, Z axis in meters */
	bool						leverArmPrimaryPrecise;			/*!< If set to true, the primary lever arm has been accurately entered and doesn't need online re-estimation. */

	float						leverArmSecondary[3];			/*!< GNSS secondary antenna lever arm in IMU X, Y, Z axis in meters */
	SbgEComGnssInstallationMode	leverArmSecondaryMode;			/*!< Define the secondary antenna (dual antenna) operating mode. */
} SbgEComGnssInstallation;

/*!
 * Holds all necessary information for GNSS module data rejection.
 */
typedef struct _SbgEComGnssRejectionConf
{
	SbgEComRejectionMode	position;							/*!< Rejection mode for position. */
	SbgEComRejectionMode	velocity;							/*!< Rejection mode for velocity. */
	SbgEComRejectionMode	hdt;								/*!< Rejection mode for true heading. */
} SbgEComGnssRejectionConf;

#endif /* SBG_ECOM_CMD_GNSS_H */
