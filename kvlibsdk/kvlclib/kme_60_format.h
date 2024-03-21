/*
 *             Copyright 2019 by Kvaser AB, Molndal, Sweden
 *                         http://www.kvaser.com
 *
 * This software is dual licensed under the following two licenses:
 * BSD-new and GPLv2. You may use either one. See the included
 * COPYING file for details.
 *
 * License: BSD-new
 * ==============================================================================
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the \<organization\> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * License: GPLv2
 * ==============================================================================
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 *
 *
 * IMPORTANT NOTICE:
 * ==============================================================================
 * This source code is made available for free, as an open license, by Kvaser AB,
 * for use with its applications. Kvaser AB does not accept any liability
 * whatsoever for any third party patent or other immaterial property rights
 * violations that may result from any usage of this source code, regardless of
 * the combination of source code and various applications that it can be used
 * in, or with.
 *
 * -----------------------------------------------------------------------------
 */

#ifndef KME_60_FORMAT_H_
#define KME_60_FORMAT_H_


#include <cinttypes>
#include <cstring>
#include <string>
#include <iostream>


#define KME60_MSG_CAN 1
#define KME60_MSG_IMU 3
#define KME60_MSG_NMEA 4
#define KME60_MSG_EVENT 5
#define KME60_MSG_PROPRIETARY 6
#define KME60_MSG_METADATA 7
#define KME60_MSG_VERSION 100
#define KME60_MSG_CLOCK 101
#define KME60_MSG_TRIGGER 102


typedef struct alignas(4) Kme60_MsgHead {
  uint16_t msgLen;
  uint8_t msgType;
  uint8_t reserved;
  Kme60_MsgHead()
  {
    msgLen = 0;
    msgType = reserved = 0;
  }
} Kme60_MsgHead_t;


// -----------------------------------------------------------------------------
/** KME60 Version */
// -----------------------------------------------------------------------------
#define KME_VERSION_MAJOR_ 6
#define KME_VERSION_MINOR_ 0

typedef struct alignas(4) Kme60_Version {
  uint32_t kmeVersion;
  uint32_t firmwareVersion;
  uint32_t serialNumber;    // Serial number
  uint32_t eanHi;           // EAN high bytes
  uint32_t eanLo;           // EAN low bytes
  Kme60_Version() { kmeVersion = firmwareVersion = serialNumber = eanHi = eanLo = 0; }
} Kme60_Version_t;


// -----------------------------------------------------------------------------
/** KME60 Real-time clock */
// -----------------------------------------------------------------------------

typedef struct alignas(4) Kme60_RealTimeClock {
  uint16_t reserved2;
  uint16_t hiresTimerFqMHz;      // Used to convert time[] to ns
  uint32_t unixTime[2];          // Seconds since 1970
  uint32_t unixTimeSubsecond[2]; // Fraction of second in ns
  uint32_t timestamp[2];         // Timestamp in clock cycles
  Kme60_RealTimeClock() : unixTime{0}, unixTimeSubsecond{0}, timestamp{0}
  {
    reserved2 = hiresTimerFqMHz = 0;
  }

  void setUnixTimeSeconds(uint64_t time)
  {
    unixTime[1] = ((time >> 32) & 0xFFFFFFFF);
    unixTime[0] = ((time >> 0) & 0xFFFFFFFF);
  }

  uint64_t getUnixTimeSeconds() const { return ((uint64_t)unixTime[1]) << 32 | unixTime[0]; }

  void setUnixTimeFractionsNs(uint64_t time)
  {
    unixTimeSubsecond[1] = ((time >> 32) & 0xFFFFFFFF);
    unixTimeSubsecond[0] = ((time >> 0) & 0xFFFFFFFF);
  }

  uint64_t getUnixTimeFractionsNs() const
  {
    return ((uint64_t)unixTimeSubsecond[1]) << 32 | unixTimeSubsecond[0];
  }

  void setTimestamp(uint64_t time)
  {
    timestamp[1] = ((time >> 32) & 0xFFFFFFFF);
    timestamp[0] = ((time >> 0) & 0xFFFFFFFF);
  }

  uint64_t getTimestamp() const { return ((uint64_t)timestamp[1]) << 32 | timestamp[0]; }
} Kme60_RealTimeClock_t;


// -----------------------------------------------------------------------------
/** KME60 log trigger */
// -----------------------------------------------------------------------------

typedef struct alignas(4) Kme_60_Trigger {
  uint8_t type;
  uint8_t channel;
  uint16_t trigNo;                    // Bit mask showing which trigger was activated
  uint32_t timestamp[2];
  uint32_t preTrigger;
  uint32_t postTrigger;
  Kme_60_Trigger() : timestamp{0}
  {
    type = channel = 0;
    trigNo = 0;
    preTrigger = postTrigger = 0;
  }
  void setTimestamp(uint64_t time)
  {
    timestamp[1] = ((time >> 32) & 0xFFFFFFFF);
    timestamp[0] = ((time >> 0) & 0xFFFFFFFF);
  }

  uint64_t getTimestamp() const { return ((uint64_t)timestamp[1]) << 32 | timestamp[0]; }
} Kme60_Trigger_t;


const unsigned int MAX_DATA_BYTES = 64;

typedef struct alignas(4) Kme60_CanMsg {
  uint8_t channel;                    // CAN channel
  uint8_t dlc;                        // Data Length Code
  uint8_t padding[2];
  uint32_t id;                        // CAN identifier including extended bit
  uint32_t flags;                     // Flags defined in hydra_host_cmds.h
  uint32_t timestamp[2];              // Timestamp in clock cycles
  uint8_t data[MAX_DATA_BYTES]; // Data bytes
  Kme60_CanMsg() : padding{0}, timestamp{0}, data{0}
  {
    channel = dlc = 0;
    id = flags = 0;
  };


  void setTimestamp(uint64_t time)
  {
    timestamp[1] = ((time >> 32) & 0xFFFFFFFF);
    timestamp[0] = ((time >> 0) & 0xFFFFFFFF);
  }

  uint64_t getTimestamp() const { return ((uint64_t)timestamp[1]) << 32 | timestamp[0]; }

} Kme60_CanMsg_t;


// -----------------------------------------------------------------------------
/** A container for predefined data */
// -----------------------------------------------------------------------------
typedef struct alignas(4) Kme60Msg {

  uint16_t msgLen;
  uint8_t msgType;
  uint8_t reserved;

  union {
    Kme60_CanMsg_t canMsg;              // CAN/CANFD message
    Kme60_Version_t verMsg;             // KME60 version message
    Kme60_RealTimeClock_t clockMsg;     // KME60 Clock message
    Kme60_Trigger_t triggerMsg;         // KME60 Trigger message
  };

  Kme60Msg()
  {
    msgLen = 0;
    msgType = reserved = 0;
    this->canMsg = Kme60_CanMsg();
    this->verMsg = Kme60_Version();
    this->clockMsg = Kme60_RealTimeClock();
    this->triggerMsg = Kme_60_Trigger();
  }
} Kme60Msg_t;


#endif
