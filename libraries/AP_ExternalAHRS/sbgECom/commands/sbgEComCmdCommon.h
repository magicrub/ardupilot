/*!
 * \file           sbgEComCmdCommon.h
 * \author         SBG Systems
 * \date           11 June 2014
 *
 * \brief          This file groups all common definitions required by all commands.
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

#ifndef __SBG_ECOM_CMD_COMMON_H__
#define __SBG_ECOM_CMD_COMMON_H__

//#include "../sbgECom.h"

//----------------------------------------------------------------------//
//- Defintions                                                         -//
//----------------------------------------------------------------------//

#define SBG_ECOM_DEFAULT_CMD_TIME_OUT	(500)	/*!< Default time out in ms for commands reception. */

/*!
 * List of all rejection modes for aiding inputs.
 */
typedef enum _SbgEComRejectionMode
{
	SBG_ECOM_NEVER_ACCEPT_MODE		= 0,		/*!< Measurement is not taken into account. */
	SBG_ECOM_AUTOMATIC_MODE			= 1,		/*!< Measurement is accepted and rejected automatically depending on consistency checks */
	SBG_ECOM_ALWAYS_ACCEPT_MODE		= 2			/*!< Measurement is always accepted. Should be used with caution */
} SbgEComRejectionMode;

/*!
 * List of all axis directions for modules/sensor alignment.
 */
typedef enum _SbgEComAxisDirection
{
	SBG_ECOM_ALIGNMENT_FORWARD		= 0,		/*!< IMU/module Axis is turned in vehicle's forward direction. */
	SBG_ECOM_ALIGNMENT_BACKWARD		= 1,		/*!< IMU/module Axis is turned in vehicle's backward direction. */
	SBG_ECOM_ALIGNMENT_LEFT			= 2,		/*!< IMU/module Axis is turned in vehicle's left direction. */
	SBG_ECOM_ALIGNMENT_RIGHT		= 3,		/*!< IMU/module Axis is turned in vehicle's right direction. */
	SBG_ECOM_ALIGNMENT_UP			= 4,		/*!< IMU/module Axis is turned in vehicle's up direction. */
	SBG_ECOM_ALIGNMENT_DOWN			= 5			/*!< IMU/module Axis is turned in vehicle's down direction. */
} SbgEComAxisDirection;

#endif
