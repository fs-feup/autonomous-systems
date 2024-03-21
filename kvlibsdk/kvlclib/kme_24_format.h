/*
**             Copyright 2017 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/

/*
** Description:
**   Definitions for the log file format used by Memorator (.KME)
**   This is an older version active only for reasons of compability.
** -----------------------------------------------------------------------------
*/

#ifndef _KME_24_FORMAT_H_
#define _KME_24_FORMAT_H_
#include "common_defs.h"

#include "compilerassert.h"

#include <pshpack1.h>


#define KME24_LOG_FILE_MAJOR_VERSION 2     // Must fit in one byte
#define KME24_LOG_FILE_MINOR_VERSION 2     // Must fit in one byte
#define KME24_LOG_FILE_VERSION32 (((unsigned long)KME24_LOG_FILE_MAJOR_VERSION << 24)+((unsigned long)KME24_LOG_FILE_MINOR_VERSION << 16)) // Room for e.g. a build number



// Logfile-entry types
#define KME24_LOG_TYPE_EXTENDED     0
#define KME24_LOG_TYPE_CAN_CH_0     1
#define KME24_LOG_TYPE_TIME         2
#define KME24_LOG_TYPE_VERSION      3
#define KME24_LOG_TYPE_TRIGGER      4
#define KME24_LOG_TYPE_CAN_CH_1     5
// 6,7,8 are reserved
#define KME24_LOG_TYPE_LOGINFO      9
#define KME24_LOG_TYPE_WALLCLOCK    10
#define KME24_LOG_TYPE_CAN_CH_2     11
#define KME24_LOG_TYPE_CAN_CH_3     12
#define KME24_LOG_TYPE_CAN_CH_4     13
#define KME24_LOG_TYPE_RESERVED     0xFF

/*
 * Trigger types (used in TriggerEvent.type). This is a bit mask.
 */
#define KME24_LOG_TRIG_LOG_ALL             1
#define KME24_LOG_TRIG_ON_EXTERNAL         2
#define KME24_LOG_TRIG_ON_MESSAGE          4
#define KME24_LOG_TRIG_LOG_FIFO            0x80




/*
 * Message flags. The DLC flags are used in the log file but not in
 * the triggers.
 */

#define KME24_LOG_MSG_NERR_FLAG        0x40000000       // In the id-field
#define KME24_LOG_MSG_EXT_FLAG         0x80000000       // In the id-field
#define KME24_LOG_MSG_RTR_FLAG         0x80             // In the dlc-field
#define KME24_LOG_MSG_ERROR_FLAG       0x40             // In the dlc-field
#define KME24_LOG_MSG_OVR_FLAG         0x20             // In the DLC field

/*
 * Logger information codes; used in LoggerInfoT records.
 */
#define KME24_LOG_INFO_OVERRUN     1

/*
 * Events to log.
 */
typedef struct
{
    unsigned char dlc;
    uint32_t id;
    unsigned char data[8];
    unsigned short time;
} kme24_CanRxEvent;

/*
 * File version data
 */
typedef struct
{
    unsigned char major_version;
    unsigned char minor_version;
    unsigned char reserved[13];
} kme24_FileVersion;

/*
 * Information about the trigger event
 */
typedef struct
{
    unsigned char  type;
    unsigned short unused;
    uint32_t  preTrigger;
    uint32_t  postTrigger;
    unsigned char  trigNo;      // A bit mask showing which trigger was activated
    unsigned short time;
} kme24_TriggerEvent;

/*
 * The frame counter says how many frames have been received since
 * the last Time Event. If overflow -> 0xFFFF.
 *
 * YMD is coded as YYYYYYYMMMMDDDDD. Year 2000 = 0. No, 1980 = 0.
 * HMD is coded as HHHHHMMMMMMSSSSS. Seconds are divided by 2.
 * (For code snippets, look in rt_clock.c, routines packTime() et al.)
 */
typedef struct
{
    unsigned char res;
    unsigned short date; // Real time date (YMD)
    unsigned short hiResTimeStampLo;
    unsigned short hiResTimeStampHi;
    uint32_t loResTimeStamp;
    unsigned short time; // Real time time (HMS)
    unsigned short frameCounter;
} kme24_TimeEvent;

/*
 * A record for "internal" logger events & things like that, for
 * example, buffer overrun.
 */
typedef struct {
  unsigned char type;
  unsigned char padding[14];
} kme24_LoggerInfoT;


/*
** Wall-clock time.
*/
typedef struct {
  unsigned char year;       // (20)00 - (20)99
  unsigned char month;      // 1-12
  unsigned char date;       // 1-31
  unsigned char hour;       // 0-23
  unsigned char minute;     // 0-59
  unsigned char second;     // 0-59
  uint32_t rtc;        // Real-time clock at the same time
  unsigned char padding[4];
} kme24_WallClockT;

/*
 * Format of a logfile-entry.
 */
typedef struct
{
    unsigned char type;
    union
    {
        kme24_CanRxEvent    frame;
        kme24_TriggerEvent  trig;
        kme24_TimeEvent     clock;
        kme24_FileVersion   version;
        kme24_LoggerInfoT   loggerInfo;
        kme24_WallClockT    wallclock;
        unsigned char byte[15];
    } payload;
} kme24_logStruct;

CompilerAssert(sizeof(kme24_logStruct) == 16);


#include <poppack.h>


#endif // _KME_24_FORMAT_H_
