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
#include "KvaLogReader_Kme50.h"
#include "KvaConverterMisc.h"
#include "kvdebug.h"

uint64_t KvaLogReader_Kme50::toNs(uint64_t ticks) {
  uint64_t ns = 0;
  if (hiresTimerFqMHz) {
    ns = (ticks*1000LL) / hiresTimerFqMHz;
  }
  return ns;
}


KvlcStatus KvaLogReader_Kme50::interpret_event(
      hydraHostCmdLogFD *kmeEvent,
      imLogData *logEvent)
{
  int dataLen = 0;
  uint64_t ticks = 0;
  logEvent->common.event_counter = ++current_eventno;
  switch (kmeEvent->logHead.cmdNo) {

    case CMD_RX_MESSAGE_FD:
    {
      logEvent->msg.frame_counter = ++current_frameno;
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_MESSAGE;
      logEvent->msg.flags = 0;

      logEvent->msg.id =
          kmeEvent->logMessageFD.id & 0x1fffffff; // max 29 bit
      if (kmeEvent->logMessageFD.id & 0x80000000) {
        logEvent->msg.flags |= canMSG_EXT;
      }
      else {
        logEvent->msg.flags |= canMSG_STD;
      }

      logEvent->msg.dlc = kmeEvent->logMessageFD.dlc & 0x0f;

      logEvent->msg.channel = kmeEvent->logMessageFD.channel;

      memcpy(&ticks, kmeEvent->logMessageFD.time, sizeof(ticks));
      logEvent->common.time64 = toNs(ticks);

      if (kmeEvent->logMessageFD.flags & MSGFLAG_OVERRUN) {
        logEvent->msg.flags |= canMSGERR_HW_OVERRUN;
        logEvent->msg.flags |= canMSGERR_SW_OVERRUN;
      }
      if (kmeEvent->logMessageFD.flags & MSGFLAG_NERR) {
          logEvent->msg.flags |= canMSG_NERR;
      }
      if (kmeEvent->logMessageFD.flags & MSGFLAG_WAKEUP) {
          logEvent->msg.flags |= canMSG_WAKEUP;
      }
      if (kmeEvent->logMessageFD.flags & MSGFLAG_REMOTE_FRAME) {
        logEvent->msg.flags |= canMSG_RTR;
      }
      if (kmeEvent->logMessageFD.flags & MSGFLAG_ERROR_FRAME) {
        logEvent->msg.flags |= canMSG_ERROR_FRAME;
      }
      if (kmeEvent->logMessageFD.flags & MSGFLAG_TX) {
        logEvent->msg.flags |= canMSG_TXACK;
      }

      if (kmeEvent->logMessageFD.flags & MSGFLAG_BRS) {
        logEvent->msg.flags |= canFDMSG_BRS;
      }
      if (kmeEvent->logMessageFD.flags & MSGFLAG_ESI) {
        logEvent->msg.flags |= canFDMSG_ESI;
      }
      if (kmeEvent->logMessageFD.flags & MSGFLAG_FDF) {
        logEvent->msg.flags |= canFDMSG_FDF;
        dataLen = dlcToNumBytesFD(kmeEvent->logMessageFD.dlc);
      } else {
        dataLen = MIN(kmeEvent->logMessageFD.dlc, 8);
      }
      memcpy(logEvent->msg.data, kmeEvent->logMessageFD.data, dataLen);

      break;
    }

    case CMD_LOG_RTC_TIME_FD:
    {
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_RTC;
      if (kmeEvent->logRtcTimeFD.hiresTimerFqMHz) {
        hiresTimerFqMHz = kmeEvent->logRtcTimeFD.hiresTimerFqMHz;
        //PRINTF(("Updated hiresTimerFqMHz = %d", hiresTimerFqMHz));
      }

      memcpy(&ticks, kmeEvent->logRtcTimeFD.time, sizeof(ticks));
      logEvent->common.time64 = toNs(ticks);

      logEvent->common.nanos_since_1970 = (uint64_t)kmeEvent->logRtcTimeFD.unixTime * ONE_BILLION +
        (uint64_t) kmeEvent->logRtcTimeFD.unixTimeSubsecond;

      if (start_of_measurement64 == 0) {
        start_of_measurement64 = logEvent->common.nanos_since_1970;
        PRINTF(("Start of measurement read  = %lu", start_of_measurement64));
      }
      break;
    }

    case CMD_LOG_TRIG_FD:
    {
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_TRIGGER;
      logEvent->trig.active = true;
      logEvent->trig.postTrigger = kmeEvent->logTrigFD.postTrigger;
      logEvent->trig.preTrigger = kmeEvent->logTrigFD.preTrigger;
      logEvent->trig.type = kmeEvent->logTrigFD.type;
      logEvent->trig.trigNo = (unsigned char)kmeEvent->logTrigFD.trigNo;

      memcpy(&ticks, kmeEvent->logTrigFD.time, sizeof(ticks));
      logEvent->common.time64 = toNs(ticks);
      break;
    }

    case CMD_LOG_VERSION_FD:
    {
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_VERSION;
      logEvent->ver.lioMajor = kmeEvent->logVerFD.lioMajor;
      logEvent->ver.lioMinor = kmeEvent->logVerFD.lioMinor;
      logEvent->ver.fwMajor = kmeEvent->logVerFD.fwMajor;
      logEvent->ver.fwMinor = kmeEvent->logVerFD.fwMinor;
      logEvent->ver.fwBuild = kmeEvent->logVerFD.fwBuild;
      logEvent->ver.serialNumber = kmeEvent->logVerFD.serialNumber;
      logEvent->ver.eanHi = kmeEvent->logVerFD.eanHi;
      logEvent->ver.eanLo = kmeEvent->logVerFD.eanLo;
      logEvent->common.time64 = 0;
      break;
    }

    default:
    {
      PRINTF(("Unknown type of kme50 event %d", kmeEvent->logHead.cmdNo));
      return kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  }
  return kvlcOK;
}

KvlcStatus KvaLogReader_Kme50::read_row(imLogData *logEvent)
{
  unsigned int msgBytes;

  KvlcStatus status;
  if (!isOpened) {
    PRINTF(("KvaLogReader_Kme50::read_row, file not open"));
    return kvlcERR_FILE_ERROR;
  }

  // Read the event head with cmdNo and cmdLen
  status = read_file((char *)&current_kmeEvent, sizeof(hcmdLogHead));
  if (kvlcOK != status) {
    return status;
  }

  msgBytes = current_kmeEvent.logHead.cmdLen - sizeof(hcmdLogHead);
  if (msgBytes <= 0 || msgBytes > sizeof(current_kmeEvent)) {
    PRINTF(("Error: Got cmdNo=%d and invalid cmdLen=%d.",
      current_kmeEvent.logHead.cmdNo,
      current_kmeEvent.logHead.cmdLen));
      return kvlcERR_INVALID_LOG_EVENT;
  }

  // Read the rest of the event
  status = read_file((char *)&current_kmeEvent + sizeof(hcmdLogHead), msgBytes);
  if (kvlcOK != status) {
    return status;
  }

  return interpret_event(&current_kmeEvent, logEvent);
}

uint64 KvaLogReader_Kme50::event_count()
{
  return (uint64) ((file_size-file_position) / 20); //sizeof(hydraHostCmdLogFD));
}


KvaLogReader_Kme50::KvaLogReader_Kme50()
{
  PRINTF(("KvaLogReader_Kme50::KvaLogReader_Kme50()"));
  current_eventno = 0;
  current_frameno = 0;
  hiresTimerFqMHz = DEFAULT_HI_RES_TIMER_FQ_MHZ;
}

KvaLogReader_Kme50::~KvaLogReader_Kme50() { 
  PRINTF(("~KvaLogReader_Kme50")); 
}

class KvaReaderMaker_Kme50 : public KvaReaderMaker
{
  public:
    KvaReaderMaker_Kme50() : KvaReaderMaker(KVLC_FILE_FORMAT_KME50) {}
    int getName(char *str) { return sprintf(str, "%s", "KME 5.0"); }
    int getExtension(char *str) { return sprintf(str, "%s", "kme50"); }
    int getDescription(char *str) { return sprintf(str, "%s", "Kvaser binary format (KME 5.0)"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_Kme50();
    }
}  registerKvaLogReader_Kme50;
