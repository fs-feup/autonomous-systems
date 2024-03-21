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
**   Definitions for the log file format used by Memorator (.KME),
**   should be the same as m16firm\Logger\loggerDefinitions.h
**
** -----------------------------------------------------------------------------
*/

#ifndef _LOGGER_DEFINITIONS_H_
#define _LOGGER_DEFINITIONS_H_

#include "common_defs.h"

#include "compilerassert.h"

#include <pshpack1.h>


#define LOG_FILE_MAJOR_VERSION 3     // Must fit in one byte
#define LOG_FILE_MINOR_VERSION 0     // Must fit in one byte
#define LOG_FILE_VERSION32 (((unsigned long)LOG_FILE_MAJOR_VERSION << 24)+((unsigned long)LOG_FILE_MINOR_VERSION << 16)) // Room for e.g. a build number


//
// Logfile entry types
//
#define LOG_TYPE_CAN_CH_0               1
#define LOG_TYPE_TIME                   2
#define LOG_TYPE_TRIGGER                4
#define LOG_TYPE_CAN_CH_1               5
#define LOG_TYPE_CLOCK_OVERFLOW         6
#define LOG_TYPE_CLOCK_DUMMY            7
#define LOG_TYPE_CLOCK_PERIODIC         8
#define LOG_TYPE_CAN_CH_2               9
#define LOG_TYPE_CAN_CH_3               10
#define LOG_TYPE_CAN_CH_4               11

#define LOG_TYPE_RESERVED               0xFF

/*
 * Trigger types (used in TriggerEvent.type). This is a bit mask.
 */
#define LOG_TRIG_LOG_ALL             1
#define LOG_TRIG_ON_EXTERNAL         2
#define LOG_TRIG_ON_MESSAGE          4
#define LOG_TRIG_LOG_FIFO            0x80

#if defined(TRIG_LOG_ALL)
// LOG_TRIG_xxx symbols must be congruent with TRIG_xxx in the parameter
// file, so make a sanity check here.
CompilerAssert(TRIG_LOG_ALL == LOG_TRIG_LOG_ALL);
CompilerAssert(TRIG_ON_EXTERNAL == LOG_TRIG_ON_EXTERNAL);
CompilerAssert(TRIG_ON_MESSAGE == LOG_TRIG_ON_MESSAGE);
CompilerAssert(TRIG_LOG_FIFO == LOG_TRIG_LOG_FIFO);
#endif



/*
 * Message flags. The DLC flags are used in the log file but not in
 * the triggers.
 */
#define LOG_MSG_NERR_FLAG        0x40000000       // In the id field
#define LOG_MSG_EXT_FLAG         0x80000000       // In the id field
#define LOG_MSG_RTR_FLAG         0x80             // In the DLC field
#define LOG_MSG_ERROR_FLAG       0x40             // In the DLC field
#define LOG_MSG_OVR_FLAG         0x20             // In the DLC field
#define LOG_MSG_WAKEUP_FLAG      LOG_MSG_NERR_FLAG



/*
 * Logger information codes; used in LoggerInfoT records.
 */
#define LOG_INFO_OVERRUN     1


/*
 * Single wire CAN - this value is in
 * RtcTimeEvent.canDriverType1 to indicate if SWC
 */
#define TRANSCEIVER_TYPE_SWC            6


/*
 * Events to log.
 */
typedef struct
{
    unsigned char dlc;
    uint32_t      id;
    unsigned char data[8];
    unsigned short time;
} CanRxEvent;


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
} TriggerEvent;

/*
 * YMD is coded as YYYYYYYMMMMDDDDD. Year 2000 = 0.
 * HMS is coded as HHHHHMMMMMMSSSSS. Seconds are divided by 2.
 * (For code snippets, look in rt_clock.c, routines packTime() et al.)
 */
typedef struct
{
    unsigned short date;                // Real time date (YMD)
    unsigned short time;                // Real time time (HMS)
    unsigned char  verMajor;            // File version number
    unsigned char  verMinor;            // File version number
    unsigned char  canDriverType1;      // Transceiver type for channel 1.
    unsigned char  padding[8];
} RtcTimeEvent;


/*
 * A record for "internal" logger events & things like that, for
 * example, buffer overrun.
 */
typedef struct {
  unsigned char type;
  unsigned char padding[14];
} LoggerInfoT;


/*
 * 16-bit clock overflow.
 */
typedef struct {
  uint32_t     currentTime;
  unsigned char padding[10];
} ClockOverflowT;


typedef struct {
  unsigned short timeLo;
  unsigned short timeHi;  // For debugging only - not valid under all circumstances.
  unsigned char padding[10];
} ClockDummyT;



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
} WallClockT;

/*
 * Format of a logfile-entry.
 */
typedef struct
{
    unsigned char type;
    union
    {
        CanRxEvent      frame;
        TriggerEvent    trig;
        RtcTimeEvent    rtc;
        LoggerInfoT     loggerInfo;
        ClockOverflowT  clockOverflow;
        ClockDummyT     clockDummy;
        unsigned char byte[15];
    } payload;
} logStruct;

CompilerAssert(sizeof(logStruct) == 16);


#include <poppack.h>


#endif // _LOGGER_DEFINITIONS_H_
