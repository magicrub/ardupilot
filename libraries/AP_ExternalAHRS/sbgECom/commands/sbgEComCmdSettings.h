/*!
 * \file           sbgEComCmdSettings.h
 * \author         SBG Systems
 * \date           11 June 2014
 *
 * \brief          This file implements SbgECom commands related to settings.
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
#ifndef __SBG_ECOM_CMD_SETTINGS_H__
#define __SBG_ECOM_CMD_SETTINGS_H__

#include "sbgEComCmdCommon.h"

//----------------------------------------------------------------------//
//- Settings action definition										   -//
//----------------------------------------------------------------------//

/*!
 * Defintion of all the settings actions available.
 */
typedef enum _SbgEComSettingsAction
{
	SBG_ECOM_REBOOT_ONLY 				= 0,		/*!< Only reboot the device. */
	SBG_ECOM_SAVE_SETTINGS				= 1,		/*!< Save the settings to non-volatile memory and then reboot the device. */
	SBG_ECOM_RESTORE_DEFAULT_SETTINGS	= 2			/*!< Restore default settings, save them to non-volatile memory and reboot the device. */
} SbgEComSettingsAction;

#endif
