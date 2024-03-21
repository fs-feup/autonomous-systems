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
#include "KvaLogWriter_Kme60.h"
#include "kvdebug.h"

KvaLogWriter_Kme60::KvaLogWriter_Kme60()
{
  PRINTF(("KvaLogWriter_Kme60::KvaLogWriter_Kme60()\n"));
  // memset(&mLogVerFD, 0, sizeof(hcmdLogVerFD));
  // mLogVerFD.len = sizeof(hcmdLogVerFD);
  // mLogVerFD.logCmdNo = CMD_LOG_VERSION_FD;
  state = WRITE_VERSION;
}

KvlcStatus KvaLogWriter_Kme60::write_header()
{
  state = WRITE_VERSION;
  return kvlcOK;
}

void KvaLogWriter_Kme60::eventToVersion (imLogData *logEvent, Kme60Msg_t* msg)
{
    msg->msgLen = sizeof(Kme60_Version_t) + sizeof(Kme60_MsgHead_t);
    msg->msgType = KME60_MSG_VERSION;
    msg->verMsg.eanHi = logEvent->ver.eanHi;
    msg->verMsg.eanLo = logEvent->ver.eanLo;
    msg->verMsg.firmwareVersion =  (logEvent->ver.fwMajor << 24) |
                                  (logEvent->ver.fwMinor << 16) |
                                  (logEvent->ver.fwBuild);
    msg->verMsg.serialNumber = logEvent->ver.serialNumber;
    msg->verMsg.kmeVersion = (KME_VERSION_MAJOR_ << 8) | KME_VERSION_MINOR_;
}

void KvaLogWriter_Kme60::eventToRtc (imLogData *logEvent, Kme60Msg_t* msg)
{
  msg->msgLen = sizeof(Kme60_RealTimeClock_t) + sizeof(Kme60_MsgHead_t);
  msg->msgType = KME60_MSG_CLOCK;
  uint64_t tmp = toTicks(logEvent->common.time64);
  msg->clockMsg.setTimestamp(tmp);
  msg->clockMsg.hiresTimerFqMHz = DEFAULT_HI_RES_TIMER_FQ_MHZ;
  msg->clockMsg.setUnixTimeSeconds(logEvent->common.nanos_since_1970 / ONE_BILLION);

  msg->clockMsg.setUnixTimeFractionsNs(logEvent->common.nanos_since_1970 % ONE_BILLION);
}

KvlcStatus KvaLogWriter_Kme60::write_row(imLogData *logEvent)
{
  KvlcStatus kvstatus = kvlcOK;

  int dataLen = 0;
  Kme60Msg_t msg;
  msg = Kme60Msg();

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Kme60::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  // Every file begins with version
  if (WRITE_VERSION == state) {
    state = WRITE_RTC;
    if ( ILOG_TYPE_VERSION == logEvent->common.type ) {
      PRINTF(("ILOG_TYPE_VERSION is first message"));

      eventToVersion(logEvent, &msg);

      kvstatus = write_file(&msg, msg.msgLen);
      logEvent->common.new_data = false;
      return kvstatus;
    }
    // The first message is not version; add one
    PRINTF(("ILOG_TYPE_VERSION is FAKE"));
    msg.msgLen = sizeof(Kme60_Version_t) + sizeof(Kme60_MsgHead_t);
    msg.msgType = KME60_MSG_VERSION;
    msg.verMsg.kmeVersion = (KME_VERSION_MAJOR_ << 8) | KME_VERSION_MINOR_;
    logEvent->common.new_data = false;
    kvstatus = write_file(&msg, msg.msgLen);
    if (kvstatus != kvlcOK) {
      return kvstatus;
    }
    msg = Kme60Msg();

  }
  // then RTC
  if (WRITE_RTC == state) {
    state = WRITE_ANY;
    if (!start_of_logging) setStartOfLogging(logEvent);
      eventToRtc(logEvent, &msg);
      kvstatus = write_file(&msg, msg.msgLen);

    if ( ILOG_TYPE_RTC == logEvent->common.type ) {
      PRINTF(("Event was ILOG_TYPE_RTC; done"));
      logEvent->common.new_data = false;
      return kvstatus;
    }
    if (kvstatus != kvlcOK) {
      return kvstatus;
    }
    msg = Kme60Msg();

  }

  uint64_t tmp = toTicks(logEvent->common.time64);
  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        return kvlcOK;
      }

      msg.msgLen = sizeof(Kme60_MsgHead_t);
      msg.msgType = KME60_MSG_CAN;

      msg.canMsg.setTimestamp(tmp);

      msg.canMsg.channel = logEvent->msg.channel;

      msg.canMsg.id = logEvent->msg.id  & 0x1fffffff;

      if (logEvent->msg.flags & canMSG_EXT) {
        msg.canMsg.id |= 0x80000000;
      }

      msg.canMsg.dlc = 0x0f & logEvent->msg.dlc;

      msg.canMsg.flags = logEvent->msg.flags;

      if (logEvent->msg.flags & canFDMSG_FDF) {
        dataLen = dlcToNumBytesFD(msg.canMsg.dlc);
      }
      else {
        dataLen = MIN(msg.canMsg.dlc, 8);
      }
      memcpy(msg.canMsg.data, logEvent->msg.data, dataLen);
      msg.msgLen += static_cast<uint16_t>(sizeof(Kme60_CanMsg_t) - sizeof(msg.canMsg.data) + dataLen);

      kvstatus = write_file(&msg, msg.msgLen);
      break;
    }

    case ILOG_TYPE_TRIGGER:
    {
      msg.msgLen = sizeof(Kme60_Trigger_t) + sizeof(Kme60_MsgHead_t);
      msg.msgType = KME60_MSG_TRIGGER;

      msg.triggerMsg.setTimestamp(tmp);
      msg.triggerMsg.postTrigger = logEvent->trig.postTrigger;
      msg.triggerMsg.preTrigger = logEvent->trig.preTrigger;
      msg.triggerMsg.type = logEvent->trig.type;
      msg.triggerMsg.trigNo = logEvent->trig.trigNo;
      msg.triggerMsg.channel = CHANNEL_UNDEFINED;
      kvstatus = write_file(&msg, msg.msgLen);
      break;
    }

    case ILOG_TYPE_VERSION:
    {
      eventToVersion(logEvent, &msg);
      kvstatus = write_file(&msg,  msg.msgLen);
      break;
    }

    case ILOG_TYPE_RTC:
    {
      eventToRtc(logEvent, &msg);
      kvstatus = write_file(&msg,  msg.msgLen);
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

#define NAME        "KME 6.0"
#define EXTENSION   "kme60"
#define DESCRIPTION "Kvaser binary format (KME 6.0) (experimental)"

class KvaWriterMaker_Kme60 : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Kme60() : KvaWriterMaker(KVLC_FILE_FORMAT_KME60) {
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
      return new KvaLogWriter_Kme60();
    }
}  registerKvaLogWriter_Kme60;

