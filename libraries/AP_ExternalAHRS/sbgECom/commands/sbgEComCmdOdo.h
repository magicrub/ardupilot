/*!
 * \file           sbgEComCmdOdo.h
 * \author         SBG Systems
 * \date           11 June 2014
 *
 * \brief          This file implements SbgECom commands related to Odometer module.
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
#ifndef SBG_ECOM_CMD_ODO_H
#define SBG_ECOM_CMD_ODO_H

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Public definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * Holds all necessary information for Odometer module parameter configuration.
 */
typedef struct _SbgEComOdoConf
{
	float		gain;						/*!< Odometer's gain in pulses / meter. */
	uint8_t		gainError;					/*!< User gain average error in % */
	bool		reverseMode;				/*!< Whether the odometer is in reverse mode or not. */
} SbgEComOdoConf;

/*!
 * Holds all necessary information for Odometer module data rejection.
 */
typedef struct _SbgEComOdoRejectionConf
{
	SbgEComRejectionMode	velocity;		/*!< Rejection mode for velocity. */
} SbgEComOdoRejectionConf;

/*!
 * CAN odometer channels definition
 * A channel is an inforamtion that can be decoded / used by the device.
 */
typedef enum _SbgEComCmdOdoCanChannel
{
	SBG_ECOM_CMD_ODO_CAN_CH_VELOCITY	= 0,							/*!< Channel used to decode the vehicle velocity information */
	SBG_ECOM_CMD_ODO_CAN_CH_REVERSE		= 1								/*!< Channel used to decode the vehicle velocity reverse info (if available). */
} SbgEComCmdOdoCanChannel;

/*
 * Define CAN odometer options bitmask
 */
#define SBG_ECOM_CMD_ODO_CAN_ENABLE				(uint16_t)(0x0001 << 0)	/*!< Set to enable CAN odometer information decoding. */
#define SBG_ECOM_CMD_ODO_CAN_ID_EXTENDED		(uint16_t)(0x0001 << 1)	/*!< Set for a 29 bit extended CAN message, otherwise standard 11 bit */
#define SBG_ECOM_CMD_ODO_CAN_BIG_ENDIAN			(uint16_t)(0x0001 << 2)	/*!< Set if the velocity is encoded in big endian, otherwise little endian */
#define SBG_ECOM_CMD_ODO_CAN_SIGNED				(uint16_t)(0x0001 << 3)	/*!< Set to interpret the parsed value as signed, otherwise unsigned. */

/*!
 * Holds all necessary information for CAN Odometer parameter configuration.
 * This format is very similar to info contained in a DBC file.
 */
typedef struct _SbgEComCmdOdoCanConf
{
	uint16_t	options;				/*!< Set of options bit masks such as CAN extended. */
	uint32_t	canId;					/*!< CAN message ID from which the odometer velocity will be parsed. */

	size_t		startBit;				/*!< Index of field MSB in big endian or LSB in little endian within the payload (any value from 0 to 63). */
	size_t		dataSize; 				/*!< Length in bits of the odometer velocity field (any value from 1 to 64 minus dataOffset). */

	float		scale;					/*!< Value to multiply the parsed field with to get physical unit^in m.s-1. */
	float		offset;					/*!< Offset to add on the scaled velocity information in m.s-1 (after applying scale factor). */
	float		minValue;				/*!< The minimum velocity to consider the message valid in m.s-1 */
	float		maxValue;				/*!< The maximum velocity to consider the message valid in m.s-1 */
} SbgEComCmdOdoCanConf;

#endif /* SBG_ECOM_CMD_ODO_H */
