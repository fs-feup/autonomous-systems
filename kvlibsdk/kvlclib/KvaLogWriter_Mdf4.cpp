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
**  Writer for MDF (Vector's MDF, their old binary format)
** ---------------------------------------------------------------------------
*/

#include "KvaLogWriter.h"
#include "common_defs.h"
#include "KvaLogWriter_Mdf4.h"
#include "TimeConv.h"
#include "kvdebug.h"

KvaLogWriter_Mdf4::KvaLogWriter_Mdf4()
{
  mdf = new Mdf4();
  mdf->initAndSetVersion(410);
  mdf->useVariableRecordSize(true);
  isSetup = false;

  mdf->setCompressionLevel(mProperties.DEFAULT_COMPRESSION_LEVEL);
}

KvlcStatus KvaLogWriter_Mdf4::open_file()
{
  KvlcStatus status = kvlcOK;
  const char *filename = get_filename();

  if (split_files()) {
    filename = get_next_filename();
  }

  status = file_status();
  if (status != kvlcOK) {
    return status;
  }

  if (mdf == NULL) {
      mdf = new Mdf4();
      mdf->initAndSetVersion(410);
      mdf->useVariableRecordSize(true);
      isSetup = false;
  }

  if (mdf->create(filename) == 0) {
    isOpened = true;
  } else {
    return kvlcERR_FILE_ERROR;
  }
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf4::close_file()
{

  if (start_of_logging) {
    unsigned int year, month, day, hour, minute, second;

    unix_time(start_of_logging / ONE_BILLION,
                &year,
                &month,
                &day,
                &hour,
                &minute,
                &second);
    mdf->setStartOfRecording(
            year,
            month,
            day,
            hour,
            minute,
            second
          );
  }

  if (isOpened) {
    mdf->write_header();
    mdf->write_data();
    mdf->write_header();
    mdf->close();
  }

  isOpened = false;
  delete mdf;
  mdf = NULL;
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf4::write_header()
{
  PRINTF(("KvaLogWriter_Mdf4::write_header"));
  if (!isSetup) {
    mdf->write_header();
    isSetup = true;
  }
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf4::write_row(imLogData *logEvent)
{
  time_uint64 lastTime = 0;
  KvlcStatus kvstatus = kvlcOK;

  MdfCanFrameType frame;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Mdf::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  mdf->setCompressionLevel(mProperties.mCompressionLevel);

  memset(&frame, 0, sizeof(frame));
  int dataLen = 0;

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        return kvlcOK;
      }
      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }
      if (lastTime > logEvent->common.time64) {
        PRINTF(("WARNING, decreasing time\n"));
      }
      lastTime = logEvent->common.time64;

      // Copy data from message
      frame.dlc = logEvent->msg.dlc;
      frame.canId = logEvent->msg.id;
      frame.flags = logEvent->msg.flags;
      if (logEvent->msg.flags & canMSG_EXT) {
        frame.canId |= MDF_ID_EXT;
      }

      if (logEvent->msg.flags & canFDMSG_FDF) {
        dataLen = dlcToNumBytesFD(logEvent->msg.dlc);
      }
      else {
        dataLen = MIN(logEvent->msg.dlc, 8);
      }
      memcpy(frame.data, logEvent->msg.data, dataLen);
      frame.datalength = dataLen;

      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        // CAN Error Frame
        mdf->addFrame(logEvent->msg.channel + 1, MDF_ERROR_FRAME_TYPE,
          logEvent->common.time64, &frame);
      }
      else if ((logEvent->msg.flags & canMSG_RTR) != 0) {
        // CAN Remote Frame
        mdf->addFrame(logEvent->msg.channel + 1, MDF_REMOTE_FRAME_TYPE,
          logEvent->common.time64, &frame);
      }
      else {
        // CAN Data Frame
        mdf->addFrame(logEvent->msg.channel + 1, MDF_CAN_FRAME_TYPE,
          logEvent->common.time64, &frame);
      }
      break;
    }
    case ILOG_TYPE_CANOTHER:
    case ILOG_TYPE_RTC:
    case ILOG_TYPE_TRIGGER:
    case ILOG_TYPE_VERSION:
    {
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
KvlcStatus KvaLogWriter_Mdf4::attach_file(const char *filename)
{
  if (!mdf) {
      mdf = new Mdf4();
      mdf->initAndSetVersion(410);
      mdf->useVariableRecordSize(true);
      isSetup = false;
  }



  if ( mdf->attach(filename) ) {
    return kvlcERR_FILE_ERROR;
  }
  return kvlcOK;
}
#define NAME        "MDF v4.1"
#define EXTENSION   "mf4"
#define DESCRIPTION "CAN frames in MDF v4.1 for Vector CANalyzer"


class KvaWriterMaker_Mdf4 : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Mdf4() : KvaWriterMaker(KVLC_FILE_FORMAT_MDF_4X) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER]        = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK]         = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER]      = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT]           = true;
      propertyList[KVLC_PROPERTY_ATTACHMENTS]          = true;
      propertyList[KVLC_PROPERTY_OVERWRITE]            = true;
      propertyList[KVLC_PROPERTY_COMPRESSION_LEVEL]    = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      return new KvaLogWriter_Mdf4();
    }
}  registerKvaLogWriter_Mdf4;
