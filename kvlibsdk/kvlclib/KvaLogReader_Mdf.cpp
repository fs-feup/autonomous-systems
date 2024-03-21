/*
**             Copyright 2018 by Kvaser AB, Molndal, Sweden
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
**  Reader for MDF (Vector's MDF, their old binary format)
** ---------------------------------------------------------------------------
*/

#include "KvaLogReader_Mdf.h"
#include "KvaConverterMisc.h"
#include "common_defs.h"
#include "os_util.h"
#include "kvdebug.h"

#include <cstdlib>
#include <cstdio>

// =============================================================================
//  KvaLogReader_Mdf
// =============================================================================

KvlcStatus KvaLogReader_Mdf::open_file(const char *filename)
{
  KvlcStatus status = KvaLogReader::open_file(filename);
  if(status!=kvlcOK)
    return status;

  mdf.reset(new MDFfile());
  if (mdf->handle (infile) != 0) {
    KvaLogReader::close_file();
    return kvlcERR_FILE_ERROR;
  }

  eventNo = 0;
  makeClkEvt = true;

  return kvlcOK;
}

KvlcStatus KvaLogReader_Mdf::close_file()
{
  mdf.release();
  return KvaLogReader::close_file();
}

KvlcStatus KvaLogReader_Mdf::interpret_event(void *x, imLogData *logEvent)
{
  mdf_event_block *event = (mdf_event_block *)x;

  memset (logEvent, 0, sizeof (imLogData));
  logEvent->common.new_data = true;

  switch (event->type)
    {
    case MDF_EVTTYPE_MESSAGE: {
      mdf_evtMsg_block *msg = (mdf_evtMsg_block *)x;
      logEvent->common.type = ILOG_TYPE_MESSAGE;
      logEvent->common.time64 = ((unsigned long long)msg->time) * 10000;
      logEvent->msg.channel = msg->channel - 1;
      logEvent->msg.id = msg->canId & 0x1FFFFFFF;
      logEvent->msg.dlc = msg->dlc;
      if (msg->rtr)
        logEvent->msg.flags |= canMSG_RTR;
      if (msg->canId & MDF_ID_EXT)
        logEvent->msg.flags |= canMSG_EXT;
      else
        logEvent->msg.flags |= canMSG_STD;
      switch (msg->dir)
        {
        case MDF_EVTDIR_RX:
          //logEvent->msg.flags |= canMSG_RX;
          break;
        case MDF_EVTDIR_TX:
          logEvent->msg.flags |= canMSG_TXACK;
          break;
        case MDF_EVTDIR_TXRQ:
          logEvent->msg.flags |= canMSG_TXRQ;
          break;
        }
      memcpy (logEvent->msg.data, msg->data, 8);
      break;
    }

    case MDF_EVTTYPE_ERRORFRAME: {
      mdf_evtErrorFrame_block *error = (mdf_evtErrorFrame_block *)x;
      logEvent->common.type = ILOG_TYPE_MESSAGE;
      logEvent->common.time64 = ((unsigned long long)error->time) * 10000;
      logEvent->msg.flags |= canMSG_ERROR_FRAME;
      logEvent->msg.channel = error->channel - 1;
      switch (error->dir)
        {
        case MDF_EVTDIR_RX:
          //logEvent->msg.flags |= canMSG_RX;
          break;
        case MDF_EVTDIR_TX:
          logEvent->msg.flags |= canMSG_TXACK;
          break;
        case MDF_EVTDIR_TXRQ:
          logEvent->msg.flags |= canMSG_TXRQ;
          break;
        }
      break;
    }

    case MDF_EVTTYPE_BUSSTAT:
      //mdf_evtBusstat_block *busstat = x;
      break;

    default:
      break;
    }

  return kvlcOK;
}

// Parse out the MDF Date and Time fields as seconds. */
static time_uint64 get_timestamp (MDFfile *mdf)
{
  char *day, *month, *year;
  char *hour, *minute, *second;
  struct tm tm;

  memset (&tm, 0, sizeof tm);

  day = mdf->getDate();
  month = strchr (day, ':');
  *month++ = 0;
  year = strchr (month, ':');
  *year++ = 0;
  tm.tm_year = atoi (year) - 1900;
  tm.tm_mon = atoi (month) - 1;
  tm.tm_mday = atoi (day);

  hour = mdf->getTime();
  minute = strchr (hour, ':');
  *minute++ = 0;
  second = strchr (minute, ':');
  *second++ = 0;
  tm.tm_hour = atoi (hour);
  tm.tm_min = atoi (minute);
  tm.tm_sec = atoi (second);

  return mktime (&tm);
}

KvlcStatus KvaLogReader_Mdf::make_clockevent(imLogData *logEvent)
{
  logEvent->common.type = ILOG_TYPE_RTC;
  logEvent->common.new_data = true;
  logEvent->common.time64 = get_timestamp (mdf.get()) * ONE_BILLION;
  return kvlcOK;
}

KvlcStatus KvaLogReader_Mdf::read_row(imLogData *logEvent)
{

  if (!isOpened) {
    PRINTF(( "KvaLogReader_Mdf::read_row, file not open" ));
    return kvlcERR_FILE_ERROR;
  }

  if (makeClkEvt) {
    makeClkEvt = false;
    return make_clockevent(logEvent);
  }

  if (eventNo >= mdf->eventCount())
    return kvlcEOF;

  mdf_event_block* event = mdf->readEvent (eventNo);
  if (event == NULL)
    return kvlcEOF;

  eventNo++;

  return interpret_event(event, logEvent);
}

uint64 KvaLogReader_Mdf::event_count()
{
  if (!isOpened) {
    PRINTF(( "KvaLogReader_Mdf::event_count, file not open" ));
    return 0;
  }

  /* Add one to accomodate the synthetic trigger event. */
  return mdf->eventCount() + 1;
}

KvaLogReader_Mdf::KvaLogReader_Mdf()
: eventNo(0), makeClkEvt(false)
{
  PRINTF(("KvaLogReader_Mdf::KvaLogReader_Mdf()\n"));
}

KvaLogReader_Mdf::~KvaLogReader_Mdf()
{
  PRINTF(("KvaLogReader_Mdf::~KvaLogReader_Mdf()\n"));
}

class KvaReaderMaker_Mdf : public KvaReaderMaker
{
  public:
    KvaReaderMaker_Mdf() : KvaReaderMaker(KVLC_FILE_FORMAT_MDF) {}

    int getName(char *str) { return sprintf(str, "%s", "MDF"); }
    int getExtension(char *str) { return sprintf(str, "%s", "log"); }
    int getDescription(char *str) { return sprintf(str, "%s", "CAN frames in Vector Mdf"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_Mdf();
    }
}  registerKvaLogReader_Mdf;


