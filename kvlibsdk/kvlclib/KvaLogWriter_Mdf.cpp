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

#include "common_defs.h"
#include "KvaLogWriter_Mdf.h"
#include "TimeConv.h"
#include "kvdebug.h"


KvaLogWriter_Mdf::KvaLogWriter_Mdf()
{
  PRINTF(("KvaLogWriter_Mdf::KvaLogWriter_Mdf()"));
  timeLo = 0;
  timeHi = 0;
  mdf = NULL;
}

KvlcStatus KvaLogWriter_Mdf::open_file()
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
      mdf = new MDFfile;
  }

  if (mdf->create(filename, get_wallclock_time()) == 0) {
    isOpened = true;
  } else {
    return kvlcERR_FILE_ERROR;
  }
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf::close_file()
{
  PRINTF(("KvaLogWriter_Mdf::close_file()"));
  if (mdf) mdf->close();
  isOpened = false;
  if (mdf) delete mdf;
  mdf = NULL;
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf::write_header()
{
  return kvlcOK;
}

KvlcStatus KvaLogWriter_Mdf::write_row(imLogData *logEvent)
{
  time_uint64 lastTime = 0;
  KvlcStatus kvstatus = kvlcOK;

  mdf_evtMsg_block msgBlock;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_Mdf::write_row, file was not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (!mdf) {
    PRINTF(("KvaLogWriter_Mdf::write_row, MDF class not created\n"));
    return kvlcERR_FILE_ERROR;
  }

  memset(&msgBlock, 0, sizeof(msgBlock));

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {

    case ILOG_TYPE_MESSAGE:
    {
      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // Don't use this channel so bail out
        PRINTF(("Not use this channel\n"));
        return kvlcOK;
      }

      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }

      // Error frame
      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {

        //mdf_evtErrorFrame_block *errMsgBlock = new mdf_evtErrorFrame_block;
        mdf_evtErrorFrame_block errMsgBlock;

        errMsgBlock.z1 = 1;

        if (lastTime > logEvent->common.time64) {
          PRINTF(("WARNING, decreasing time\n"));
        }
        lastTime = logEvent->common.time64;

        errMsgBlock.type = MDF_EVTTYPE_ERRORFRAME;

        // The msg timestamp divided by 10 000 to get the timestamp in 10 ms
        errMsgBlock.time = (DWORD) (logEvent->common.time64 / 10000);

        // Channel
        errMsgBlock.channel = logEvent->msg.channel + 1;

        msgBlock.z1 = 1;

        // Id for error frame
        errMsgBlock.canId = 0x0000ffff;

        // dlc always 8
        errMsgBlock.dlc = 8;

        // rtr always 0
        errMsgBlock.rtr = 0;

        // TX ACK (message was really sent)
        if (logEvent->msg.flags & canMSG_TXACK) {
          errMsgBlock.dir = MDF_EVTDIR_TX;
        }
        // TX REQ (message was transferred to the CAN controller)
        else if (logEvent->msg.flags & canMSG_TXRQ) {
          errMsgBlock.dir = MDF_EVTDIR_TXRQ;
        }
        else errMsgBlock.dir = MDF_EVTDIR_RX;

        mdf->writeEvent((mdf_event_block*)&errMsgBlock);
      }
      else {
        mdf_evtMsg_block msgBlock;

        msgBlock.z1 = 1;

        if (lastTime > logEvent->common.time64) {
          PRINTF(("WARNING, decreasing time\n"));
        }
        lastTime = logEvent->common.time64;

        msgBlock.type = MDF_EVTTYPE_MESSAGE;

        // The msg timestamp divided by 10 000 to get the timestamp in 10 ms
        msgBlock.time = (DWORD) (logEvent->common.time64 / 10000);

        // Channel
        msgBlock.channel = logEvent->msg.channel + 1;

        // Id and flags in id field
        msgBlock.canId = logEvent->msg.id;
        if (logEvent->msg.flags & canMSG_EXT) {
          msgBlock.canId |= MDF_ID_EXT;
        }

        // dlc and flags in the dlc field
        msgBlock.dlc = logEvent->msg.dlc;

        // Remote request
        if ((logEvent->msg.flags & canMSG_RTR) != 0) {
          msgBlock.rtr = 1;
        }
        else msgBlock.rtr = 0;

        // TX ACK (message was really sent)
        if (logEvent->msg.flags & canMSG_TXACK) {
          msgBlock.dir = MDF_EVTDIR_TX;
        }
        // TX REQ (message was transferred to the CAN controller)
        else if (logEvent->msg.flags & canMSG_TXRQ) {
          msgBlock.dir = MDF_EVTDIR_TXRQ;
        }
        else msgBlock.dir = MDF_EVTDIR_RX;

        memcpy(msgBlock.data, logEvent->msg.data, 8);

        mdf->writeEvent((mdf_event_block*)&msgBlock);

      }
      break;
    }
    case ILOG_TYPE_CANOTHER:
    {
      PRINTF(("MDF ILOG_TYPE_CANOTHER\n"));
      //mdf_evtBusstat_block busstatBlock;
      //busstatBlock.type = MDF_EVTTYPE_BUSSTAT;
      break;
    }
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

  // Is it time to split the file?
  if (split_on_size(mdf->getBytesWritten())) {
    kvstatus = split_file();
    if (kvstatus != kvlcOK) return kvstatus;
  }

  return kvstatus;
}

#define NAME        "MDF"
#define EXTENSION   "log"
#define DESCRIPTION "CAN frames in Vector Mdf"

class KvaWriterMaker_Mdf : public KvaWriterMaker
{
  public:
    KvaWriterMaker_Mdf() : KvaWriterMaker(KVLC_FILE_FORMAT_MDF) {
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
      return new KvaLogWriter_Mdf();
    }
}  registerKvaLogWriter_Mdf;
