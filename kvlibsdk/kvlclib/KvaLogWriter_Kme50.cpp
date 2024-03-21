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
#include "KvaLogWriter_Kme50.h"
#include "kvdebug.h"

KvaLogWriter_Kme50::KvaLogWriter_Kme50()
{
  PRINTF(("KvaLogWriter_Kme50::KvaLogWriter_Kme50()\n"));
  memset(&mLogVerFD, 0, sizeof(hcmdLogVerFD));
  mLogVerFD.len = sizeof(hcmdLogVerFD);
  mLogVerFD.logCmdNo = CMD_LOG_VERSION_FD;
  state = WRITE_VERSION;
}

KvlcStatus KvaLogWriter_Kme50::write_header()
{
  state = WRITE_VERSION;
  return kvlcOK;
}

void KvaLogWriter_Kme50::eventToVersion (imLogData *logEvent, hcmdLogVerFD* logVerFD)
{
  logVerFD->logCmdNo = CMD_LOG_VERSION_FD;
  logVerFD->len = sizeof(hcmdLogVerFD);
  logVerFD->lioMajor = logEvent->ver.lioMajor;
  logVerFD->lioMinor = logEvent->ver.lioMinor;
  logVerFD->fwMajor = logEvent->ver.fwMajor;
  logVerFD->fwMinor = logEvent->ver.fwMinor;
  logVerFD->fwBuild = logEvent->ver.fwBuild;
  logVerFD->serialNumber = logEvent->ver.serialNumber;
  logVerFD->eanHi = logEvent->ver.eanHi;
  logVerFD->eanLo = logEvent->ver.eanLo;
  memcpy(&mLogVerFD, logVerFD, sizeof(hcmdLogVerFD));
}

void KvaLogWriter_Kme50::eventToRtc (imLogData *logEvent, hcmdLogRtcTimeFD* logRtcTimeFD)
{
  logRtcTimeFD->logCmdNo = CMD_LOG_RTC_TIME_FD;
  logRtcTimeFD->len = sizeof(hcmdLogRtcTimeFD);
  logRtcTimeFD->hiresTimerFqMHz = DEFAULT_HI_RES_TIMER_FQ_MHZ;
  logRtcTimeFD->unixTime = (uint32_t) (logEvent->common.nanos_since_1970 / ONE_BILLION);
  logRtcTimeFD->unixTimeSubsecond = (uint32_t) (logEvent->common.nanos_since_1970 % ONE_BILLION);
  uint64_t tmp = toTicks(logEvent->common.time64);
  memcpy(logRtcTimeFD->time, &tmp, sizeof(logRtcTimeFD->time));
}

KvlcStatus KvaLogWriter_Kme50::write_row(imLogData *logEvent)
{
  KvlcStatus kvstatus = kvlcOK;

  int dataLen = 0;
  hydraHostCmdLogFD L;
  memset(&L, 0, sizeof(L));

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Kme50::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  // Every file begins with version
  if (WRITE_VERSION == state) {
    state = WRITE_RTC;
    if ( ILOG_TYPE_VERSION == logEvent->common.type ) {
      PRINTF(("ILOG_TYPE_VERSION is first message"));
      eventToVersion(logEvent, &L.logVerFD);
      kvstatus = write_file(&L, sizeof(mLogVerFD));
      logEvent->common.new_data = false;
      return kvstatus;
    }
    // The first message is not version; add one
    PRINTF(("ILOG_TYPE_VERSION is FAKE"));
    memcpy(&L.logVerFD, &mLogVerFD, sizeof(mLogVerFD));
    kvstatus = write_file(&L, sizeof(mLogVerFD));
    logEvent->common.new_data = false;
    if (kvstatus != kvlcOK) {
      return kvstatus;
    }
    memset(&L, 0, sizeof(L));
  }

  // then RTC
  if (WRITE_RTC == state) {
    state = WRITE_ANY;
    if (!start_of_logging) setStartOfLogging(logEvent);

    eventToRtc(logEvent, &L.logRtcTimeFD);
    kvstatus = write_file(&L, L.logRtcTimeFD.len);

    if ( ILOG_TYPE_RTC == logEvent->common.type ) {
      PRINTF(("Event was ILOG_TYPE_RTC; done"));
      logEvent->common.new_data = false;
      return kvstatus;
    }
    if (kvstatus != kvlcOK) {
      return kvstatus;
    }
    memset(&L, 0, sizeof(L));
  }

  uint64_t tmp = toTicks(logEvent->common.time64);
  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        return kvlcOK;
      }

      L.logMessageFD.logCmdNo = CMD_RX_MESSAGE_FD;

      memcpy(L.logMessageFD.time, &tmp, sizeof(L.logMessageFD.time));
      L.logMessageFD.channel = logEvent->msg.channel;

      L.logMessageFD.id = logEvent->msg.id  & 0x1fffffff;
      if (logEvent->msg.flags & canMSG_EXT) {
        L.logMessageFD.id |= 0x80000000;
      }

      L.logMessageFD.dlc = 0x0f & logEvent->msg.dlc;

      if (logEvent->msg.flags & canMSG_NERR) {
        L.logMessageFD.flags |= MSGFLAG_NERR;
      }
      else if (logEvent->msg.flags & canMSG_WAKEUP) {
        L.logMessageFD.flags |= MSGFLAG_WAKEUP;
      }

      if (logEvent->msg.flags & canMSG_RTR) {
        L.logMessageFD.flags |= MSGFLAG_REMOTE_FRAME;
      }
      if (logEvent->msg.flags & canMSG_ERROR_FRAME) {
        L.logMessageFD.flags |= MSGFLAG_ERROR_FRAME;
      }
      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        L.logMessageFD.flags |= MSGFLAG_OVERRUN;
        overrun_occurred = true;
      }
      if (logEvent->msg.flags & canMSG_TXACK) {
        L.logMessageFD.flags |= MSGFLAG_TX;
      }
      if (logEvent->msg.flags & canFDMSG_BRS) {
        L.logMessageFD.flags |= MSGFLAG_BRS;
      }
      if (logEvent->msg.flags & canFDMSG_ESI) {
        L.logMessageFD.flags |= MSGFLAG_ESI;
      }
      if (logEvent->msg.flags & canFDMSG_FDF) {
        L.logMessageFD.flags |= MSGFLAG_FDF;
        dataLen = dlcToNumBytesFD(L.logMessageFD.dlc);
      }
      else {
        dataLen = MIN(L.logMessageFD.dlc, 8);
      }
      memcpy(L.logMessageFD.data, logEvent->msg.data, dataLen);

      L.logMessageFD.len = 20 + dataLen;

      kvstatus = write_file(&L, L.logMessageFD.len);
      break;
    }

    case ILOG_TYPE_TRIGGER:
    {
      L.logTrigFD.logCmdNo = CMD_LOG_TRIG_FD;
      L.logTrigFD.len = sizeof(hcmdLogTrigFD);

      memcpy(L.logTrigFD.time, &tmp, sizeof(L.logTrigFD.time));
      L.logTrigFD.postTrigger = logEvent->trig.postTrigger;
      L.logTrigFD.preTrigger  = logEvent->trig.preTrigger;
      L.logTrigFD.type        = logEvent->trig.type;
      L.logTrigFD.trigNo      = logEvent->trig.trigNo;
      L.logTrigFD.channel     = CHANNEL_UNDEFINED;
      kvstatus = write_file(&L, sizeof(hcmdLogTrigFD));
      break;
    }

    case ILOG_TYPE_VERSION:
    {
      eventToVersion(logEvent, &L.logVerFD);
      kvstatus = write_file(&L, sizeof(mLogVerFD));
      break;
    }

    case ILOG_TYPE_RTC:
    {
      eventToRtc(logEvent, &L.logRtcTimeFD);
      kvstatus = write_file(&L, L.logRtcTimeFD.len);
      break;
    }

    default:
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n\n\n",
              logEvent->common.type, logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;
  }
  logEvent->common.new_data = false;
  return kvstatus;
}

#define NAME        "KME 5.0"
#define EXTENSION   "kme50"
#define DESCRIPTION "Kvaser binary format (KME 5.0)"

class KvaWriterMaker_Kme50 : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Kme50() : KvaWriterMaker(KVLC_FILE_FORMAT_KME50) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_SIZE_LIMIT] = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      return new KvaLogWriter_Kme50();
    }
}  registerKvaLogWriter_Kme50;

