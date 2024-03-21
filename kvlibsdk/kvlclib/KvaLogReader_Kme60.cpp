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


#include "common_defs.h"
#include "kvaConverter.h"
#include "KvaLogReader_Kme60.h"
#include "KvaConverterMisc.h"
#include "kvdebug.h"


uint64_t KvaLogReader_Kme60::toNs(uint64_t ticks) {
  uint64_t ns = 0;

  if (hiresTimerFqMHz) {

    if (hiresTimerFqMHz == 1000) {
      ns = ticks;
    }
    else {
      ns = (ticks * 1000LL) / hiresTimerFqMHz;
    }
  }
  return ns;
}

KvlcStatus KvaLogReader_Kme60::interpret_event(Kme60Msg_t *kmeEvent,
					       imLogData      *logEvent)
{
  uint64_t ticks;

  logEvent->common.event_counter = ++current_eventno;

  switch (kmeEvent->msgType) {
    case KME60_MSG_CAN:
    {
      if (checkEventMsgLength(kmeEvent) != kvlcOK) {
        return kvlcERR_INVALID_LOG_EVENT;
      }
      logEvent->msg.frame_counter = ++current_frameno;
      logEvent->common.new_data   = true;
      logEvent->common.type       = ILOG_TYPE_MESSAGE;
      logEvent->msg.flags         = 0;

      logEvent->msg.id = kmeEvent->canMsg.id & 0x1fffffff; // max 29 bit
      if (kmeEvent->canMsg.id & 0x80000000) {
        logEvent->msg.flags |= canMSG_EXT;
      } else {
        logEvent->msg.flags |= canMSG_STD;
      }

      logEvent->msg.dlc = kmeEvent->canMsg.dlc & 0x0f;

      logEvent->msg.channel = kmeEvent->canMsg.channel;

      memcpy(&ticks, kmeEvent->canMsg.timestamp, sizeof(ticks));

      logEvent->common.time64 = toNs(ticks);

      logEvent->msg.flags = kmeEvent->canMsg.flags;
      {
        int dataLen;

        if (kmeEvent->canMsg.flags & canFDMSG_FDF) {
          dataLen = dlcToNumBytesFD(kmeEvent->canMsg.dlc);
        } else {
          dataLen = MIN(kmeEvent->canMsg.dlc, 8);
        }

        memcpy(logEvent->msg.data, kmeEvent->canMsg.data, dataLen);
      }

      break;
    }
    case KME60_MSG_CLOCK:
    {
      if (checkEventMsgLength(kmeEvent) != kvlcOK) {
        return kvlcERR_INVALID_LOG_EVENT;
      }
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_RTC;

      if (kmeEvent->clockMsg.hiresTimerFqMHz) {
        hiresTimerFqMHz = kmeEvent->clockMsg.hiresTimerFqMHz;
      }


      ticks = kmeEvent->clockMsg.getTimestamp();
      logEvent->common.time64 = toNs(ticks);


      logEvent->common.nanos_since_1970 = kmeEvent->clockMsg.getUnixTimeSeconds() * ONE_BILLION +
        kmeEvent->clockMsg.getUnixTimeFractionsNs();

      if (start_of_measurement64 == 0) {
        start_of_measurement64 = logEvent->common.nanos_since_1970;
        PRINTF(("Start of measurement read  = %lu", start_of_measurement64));
      }
      break;
    }
    case KME60_MSG_VERSION:
    {
      if (checkEventMsgLength(kmeEvent) != kvlcOK) {
        return kvlcERR_INVALID_LOG_EVENT;
      }
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_VERSION;
      logEvent->ver.fwMajor = (kmeEvent->verMsg.firmwareVersion & 0xff000000) >> 24;
      logEvent->ver.fwMinor = (kmeEvent->verMsg.firmwareVersion & 0x00ff0000) >> 16;
      logEvent->ver.fwBuild = kmeEvent->verMsg.firmwareVersion & 0x0000ffff;
      logEvent->ver.eanHi = kmeEvent->verMsg.eanHi;
      logEvent->ver.eanLo = kmeEvent->verMsg.eanLo;
      logEvent->ver.serialNumber = kmeEvent->verMsg.serialNumber;
      logEvent->ver.lioMajor = (kmeEvent->verMsg.kmeVersion & 0x0000ff00) >> 8;
      logEvent->ver.lioMinor = kmeEvent->verMsg.kmeVersion & 0x000000ff;
      logEvent->common.time64 = 0;
      break;
    }
    case KME60_MSG_TRIGGER:
    {
      if (checkEventMsgLength(kmeEvent) != kvlcOK) {
        return kvlcERR_INVALID_LOG_EVENT;
      }
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_TRIGGER;
      logEvent->trig.active = true;
      logEvent->trig.postTrigger = kmeEvent->triggerMsg.postTrigger;
      logEvent->trig.preTrigger = kmeEvent->triggerMsg.preTrigger;
      logEvent->trig.type = kmeEvent->triggerMsg.type;
      logEvent->trig.trigNo = (unsigned char)kmeEvent->triggerMsg.trigNo;

      uint64_t ticks = kmeEvent->triggerMsg.getTimestamp();
      logEvent->common.time64 = toNs(ticks);
      break;
    }
    case KME60_MSG_IMU:
    case KME60_MSG_NMEA:
    case KME60_MSG_PROPRIETARY:
    case KME60_MSG_EVENT:
    case KME60_MSG_METADATA:
    {
      PRINTF(("Unsuported kme60 event %d", kmeEvent->msgType));
      logEvent->common.new_data = false;            // signal skip to kvmlib
      logEvent->common.type = ILOG_TYPE_EXTERNAL;   // signal skip to kvlclib
      return kvlcOK;
      break;
    }
    default:
    {
      PRINTF(("Unknown type of kme60 event %d", kmeEvent->msgType));
      return kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  }

  return kvlcOK;
}
KvlcStatus KvaLogReader_Kme60::checkEventMsgLength(Kme60Msg_t *kmeEvent) {
  unsigned int msgBytes;

  msgBytes = kmeEvent->msgLen - sizeof(Kme60_MsgHead_t);
  if (msgBytes <= 0 || msgBytes > sizeof(*kmeEvent)) {
    PRINTF(("Error: Got msgType=%d and invalid cmdLen=%d.",
      kmeEvent->msgType,
      kmeEvent->msgLen));
    return kvlcERR_INVALID_LOG_EVENT;
  }

  return kvlcOK;
}

bool KvaLogReader_Kme60::should_flush(Kme60Msg_t *kmeEvent){

  switch(kmeEvent->msgType){
    case KME60_MSG_IMU:
    case KME60_MSG_NMEA:
    case KME60_MSG_PROPRIETARY:
    case KME60_MSG_EVENT:
    case KME60_MSG_METADATA:
    {
      return true;
      break;
    }
    default:
      return false;
  }
}


KvlcStatus KvaLogReader_Kme60::read_row(imLogData *logEvent) {
  unsigned int msgBytes;

  KvlcStatus status;
  if (!isOpened) {
    PRINTF(("KvaLogReader_Kme60::read_row, file not open"));
    return kvlcERR_FILE_ERROR;
  }

  // Read the event head with msgType and msgLen
  status = read_file((char *)&current_kmeEvent, sizeof(Kme60_MsgHead_t));
  if (kvlcOK != status) {
    return status;
  }
  msgBytes = current_kmeEvent.msgLen - sizeof(Kme60_MsgHead_t);

  // Read the rest of the event
  if (should_flush(&current_kmeEvent)) {

    status = move_fpos(msgBytes);

    PRINTF(("Unsuported kme60 event %d", current_kmeEvent.msgType));
    logEvent->common.new_data = false;            // signal skip to kvmlib
    logEvent->common.type = ILOG_TYPE_EXTERNAL;   // signal skip to kvlclib
    return kvlcOK;
  }

  status = read_file((char *)&current_kmeEvent + sizeof(Kme60_MsgHead_t), msgBytes);

  if (kvlcOK != status) {
    return status;
  }

  return interpret_event(&current_kmeEvent, logEvent);
}

uint64 KvaLogReader_Kme60::event_count()
{
  return (uint64) ((file_size-file_position) / sizeof(Kme60Msg_t));
}


KvaLogReader_Kme60::KvaLogReader_Kme60()
{
  PRINTF(("KvaLogReader_Kme60::KvaLogReader_Kme60()"));
  current_eventno = 0;
  current_frameno = 0;
  hiresTimerFqMHz = DEFAULT_HI_RES_TIMER_FQ_MHZ;
}

KvaLogReader_Kme60::~KvaLogReader_Kme60() {
  PRINTF(("~KvaLogReader_Kme60"));
}

class KvaReaderMaker_Kme60 : public KvaReaderMaker
{
  public:
    KvaReaderMaker_Kme60() : KvaReaderMaker(KVLC_FILE_FORMAT_KME60) {}
    int getName        (char *str) { return sprintf(str, "%s", "KME 6.0"); }
    int getExtension   (char *str) { return sprintf(str, "%s", "kme60"); }
    int getDescription (char *str) { return sprintf(str, "%s", "Kvaser binary format (KME 6.0) (experimental)"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_Kme60();
    }
} registerKvaLogReader_Kme60;

