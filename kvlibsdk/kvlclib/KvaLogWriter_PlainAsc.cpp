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
**   Writer for "Kvaser ASCII", i.e. the text file format once defined
**   by Kvaser Navigator.
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "KvaLogWriter_PlainAsc.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "os_util.h"

#define   idMask         0x1FFFFFFF

//--------------------------------------------------------------------------
uint32_t KvaLogWriter_PlainAsc::print_file_date (char *outbuffer)
{
  time_t aclock;
  time_uint64 unix_seconds;
  const char  *p = outbuffer;

  unix_seconds = get_property_logfile_creation_date();
  if (unix_seconds) {
    outbuffer += sprintf(outbuffer, "Memorator Binary logfile created at: ");
    outbuffer += printCalendarDate(outbuffer, unix_seconds * ONE_BILLION);
    outbuffer += sprintf(outbuffer, "\n\n");
  }
  else {
    outbuffer += sprintf(outbuffer, "Converted from Memorator Binary format at: ");
    aclock = get_wallclock_time();
    outbuffer += printCalendarDate(outbuffer, aclock * ONE_BILLION);
    outbuffer += sprintf(outbuffer, "\n\n");
  }

  return (uint32_t)(outbuffer - p);  /* pointer aritmetic = strlen(p) */
}

//--------------------------------------------------------------------------
KvaLogWriter_PlainAsc::KvaLogWriter_PlainAsc()
{
  PRINTF(("KvaLogWriter_PlainAsc::KvaLogWriter_PlainAsc()\n"));
  mMaxDataBytes = get_property_limit_databytes();
}

KvaLogWriter_PlainAsc::~KvaLogWriter_PlainAsc()
{
  PRINTF(("~KvaLogWriter_PlainAsc\n"));
}


//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_PlainAsc::write_header()
{
  char outbuf[2000];
  char *outbuffer = outbuf;
  char *p = outbuffer;
  char timerow[80];
  int i;
  int length = 0;

  mMaxDataBytes = get_property_limit_databytes();

  if (!isOpened) {
    PRINTF(("KvaLogWriter_PlainAsc::write_header, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  PRINTF(("KvaLogWriter_PlainAsc::write_header\n"));
  outbuffer[0] = '\0';
  if (get_property_write_header()) {
    os_rewind(outfile);
    outbuffer += sprintf(outbuffer, "                              Kvaser Memorator Log\n"
                     "                              ====================\n\n");
    // Use property logfile_creation_date (or wall clock if property is zero) as
    // file creation/extraction date
    outbuffer += print_file_date(outbuffer);

    if (overrun_occurred) {
      outbuffer += sprintf(outbuffer, PLAIN_ASC_OVERRUN_STRING"\n");
    }
    else {
      for (i = (int)strlen(PLAIN_ASC_OVERRUN_STRING); i > 0; i--) {
        outbuffer += sprintf(outbuffer, " ");
      }
      outbuffer += sprintf(outbuffer, "\n");
    }
    outbuffer += sprintf(outbuffer,
      "Settings:\n   Format of data field%c %s\n",
       ':', get_property_data_in_hex()?"HEX":"DEC");
    outbuffer += sprintf(outbuffer,
      "   Format of id field%c   %s\n",
       ':', get_property_id_in_hex()?"HEX":"DEC");

    printDecimalNumber(timerow, (time_int64) get_property_offset(), get_property_time_decimals());

    outbuffer += sprintf(outbuffer, "   Timestamp Offset%c     %-18s s\n",
      ':', timerow);

    outbuffer += sprintf(outbuffer, "   CAN channel:         ");

    for (i=0; i<MAX_CHANNELS; i++) {
      if (get_property_channel_mask() & (unsigned long)(1<<i)) {
        outbuffer += sprintf(outbuffer, " %d", i+1);
      }
    }
    outbuffer += sprintf(outbuffer, "\n\n");

    char underline[318]; // 64 * 4 + 62
    memset(underline, '=', 62 + mMaxDataBytes*4);
    underline[62 + mMaxDataBytes*4 - 1] = '\0';
    // A data frame in hex needs 64 lesser characters

    outbuffer += sprintf(outbuffer, " %11s %4s %12s %-12s %3s ", 
                     "Time", "Chan", "Identifier", "Flags", "DLC");
    char format[64];
    if (get_property_data_in_hex()) {
        underline[60+mMaxDataBytes*3] = '\0';
        sprintf(format, " %%-%ds", 6 + mMaxDataBytes*3);
        outbuffer += sprintf(outbuffer, format, "Data");
    } else {
      sprintf(format, " %%-%ds", 8 + mMaxDataBytes*4);
      outbuffer += sprintf(outbuffer, format, "Data");
    }

    length = sprintf(outbuffer, " %7s\n%s\n", "Counter", underline);

    outbuffer[length] = '\0';
    outbuffer += length;
    return write_file(p, (size_t)(outbuffer - p));
  }
  else {
     PRINTF(("skip informational header. \n"));
  }

  return kvlcOK;
}

//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_PlainAsc::write_row(imLogData *logEvent)
{
  char outbuf[1000];
  char *outbuffer = outbuf;
  char *ptr = outbuffer;
  char timerow[80];
  int  length = 0;
  KvlcStatus kvstatus = kvlcOK;

  time_uint64 lastTime = 0; 

  if (!isOpened) {
    PRINTF(("KvaLogWriter_PlainAsc::write_row, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  mMaxDataBytes = get_property_limit_databytes();

  outbuffer[0] = '\0';

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {

    case ILOG_TYPE_TRIGGER:
    {
      time_int64 msgTime = ((time_int64) logEvent->common.time64 +
                           (time_int64) get_property_offset());

      printDecimalNumber(timerow, msgTime, get_property_time_decimals());

      length = sprintf(outbuffer,
        "%12s  Trigger (type=0x%x, active=0x%02x, pre-trigger=%d, post-trigger=%d)\n",
        timerow,
        logEvent->trig.type,
        logEvent->trig.trigNo,
        (int) logEvent->trig.preTrigger,
        (int)logEvent->trig.postTrigger
      );

      outbuffer += length;
      kvstatus = write_file(ptr, (size_t)(outbuffer - ptr));
      break;
    }

    case ILOG_TYPE_MESSAGE:
    {
      if (lastTime > logEvent->common.time64) {
        PRINTF(("WARNING, decreasing time\n"));
      }
      lastTime = logEvent->common.time64;

      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        PRINTF((" dont use this channel so bail out \n"));
        return kvlcOK;
      }
      time_int64 msgTime = ((time_int64) logEvent->common.time64 +
                         (time_int64) get_property_offset());

      char *p = timerow;

      p += printDecimalNumber(timerow, msgTime, get_property_time_decimals());

      outbuffer += sprintf(outbuffer, "%12s  ", timerow);
      outbuffer += sprintf(outbuffer, "%1d  ", logEvent->msg.channel+1);

      // CAN Identifier
      bool data_in_hex = get_property_data_in_hex();
      bool id_in_hex   = get_property_id_in_hex();

      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        // Error frame.
        outbuffer += sprintf(outbuffer, "ErrorFrame");
      }
      else {
        unsigned int id = (unsigned int) logEvent->msg.id;
        if ((logEvent->msg.flags & canMSG_EXT) != 0) {
          id |= 0x80000000;
        }
        if (get_property_hlp_j1939()) {
          id = get_J1939_id(id);
        }
        if (id_in_hex) {
          outbuffer += sprintf(outbuffer, "%10X", (int)(id & idMask));
        }
        else {
          outbuffer += sprintf(outbuffer, "%10d", (int)(id & idMask));
        }
      }

      // flags
      if ((logEvent->msg.flags & canMSG_EXT) != 0) {
        outbuffer += sprintf(outbuffer, "x   ");
      }
      else {
        outbuffer += sprintf(outbuffer, "    ");
      }

      if (logEvent->msg.flags & canMSG_TXACK) {
        outbuffer += sprintf(outbuffer, "Tx   ");
      }
      else if (logEvent->msg.flags & canMSG_TXRQ) {
        outbuffer += sprintf(outbuffer, "TxRq ");
      }
      else {
        outbuffer += sprintf(outbuffer, "Rx   ");
      }

      if ((logEvent->msg.flags & canMSG_NERR) != 0) {
        outbuffer += sprintf(outbuffer, "E");
      }
      else {
        outbuffer += sprintf(outbuffer, " ");
      }
      // Message is an FD message (CAN FD)
      if ((logEvent->msg.flags & canFDMSG_FDF) != 0) {
        outbuffer += sprintf(outbuffer, "F");
      } else {
        outbuffer += sprintf(outbuffer, " ");
      }
      // Message is sent/received with bit rate switch (CAN FD)
      if ((logEvent->msg.flags & canFDMSG_BRS) != 0) {
        outbuffer += sprintf(outbuffer, "B");
      } else {
        outbuffer += sprintf(outbuffer, " ");
      }
      // Sender of the message is in error passive mode (CAN FD)
      if ((logEvent->msg.flags & canFDMSG_ESI) != 0) {
        outbuffer += sprintf(outbuffer, "P");
      } else {
        outbuffer += sprintf(outbuffer, " ");
      }

      if ((logEvent->msg.flags & canMSG_RTR) != 0) {
        outbuffer += sprintf(outbuffer, "R");
      }
      else {
        outbuffer += sprintf(outbuffer, " ");
      }

      if ((logEvent->msg.flags & canMSG_WAKEUP) != 0) {
        outbuffer += sprintf(outbuffer, "W");
      }
      else {
        outbuffer += sprintf(outbuffer, " ");
      }

      if ((logEvent->msg.flags & canMSGERR_OVERRUN) != 0) {
        outbuffer += sprintf(outbuffer, "#");
      }
      else {
        outbuffer += sprintf(outbuffer, " ");
      }

      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }

      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        outbuffer += sprintf(outbuffer, "     ");
      }
      else {
        outbuffer += sprintf(outbuffer, "%3d  ", logEvent->msg.dlc);
      }

      // A data frame takes dlc * 4 characters in decimal, and dlc * 3
      // characters in hex.  We assign 20 + 1 characters to write
      // errorframe/rtr below.
      char data_spaces[236];
      int  index;

      memset(data_spaces, ' ', sizeof(data_spaces));
      
      if (data_in_hex) {
	index = (int)mMaxDataBytes * 3 - 21;
      } else {
	index = (int)mMaxDataBytes * 4 - 21;
      }

      if (index > ((int)sizeof(data_spaces) - 1)) {
	kvstatus = kvlcERR_INTERNAL_ERROR;
      } else {

	if (index < 0) {
	  index = 0;
	}

	data_spaces[index] = '\0';

	if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
	  outbuffer += sprintf(outbuffer, "%-20s %s", "ERROR FRAME", data_spaces);
	}
	else if ((logEvent->msg.flags & canMSG_RTR) != 0) {
	  outbuffer += sprintf(outbuffer, "%-20s %s", "RTR", data_spaces);
	}
	else {
	  // A standard data frame.
	  
	  unsigned int num_bytes = logEvent->msg.dlc;
	  if (num_bytes > 8) {
	    num_bytes = 8;
	  }
	  if (logEvent->msg.flags & canFDMSG_FDF) {
	    num_bytes = dlcToNumBytesFD(logEvent->msg.dlc);
	  }
	  if (num_bytes > mMaxDataBytes) {
	    data_truncation_occurred = true;
	  }
	  
	  for (unsigned int i = 0; i < mMaxDataBytes; i++) {
	    if (i >= num_bytes) {
	      if (data_in_hex)
		outbuffer += sprintf(outbuffer, "   ");
	      else
		outbuffer += sprintf(outbuffer, "    ");
	    }
	    else {
	      if (data_in_hex)
		outbuffer += sprintf(outbuffer, "%02X ", (unsigned char)logEvent->msg.data[i]);
	      else
		outbuffer += sprintf(outbuffer, "%3d ", (unsigned char)logEvent->msg.data[i]);
	    }
	  }
	}
	
	// counter
	outbuffer += sprintf(outbuffer, " %12lu\n", logEvent->msg.frame_counter);
	
	kvstatus = write_file(ptr, (size_t)(outbuffer - ptr));
      }
      break;
    }

    case ILOG_TYPE_RTC:
    {
      // NOTE: this is the low res calendar time stamp from the
      // log file (unless KvaConverter or the reader changes it)
      if (get_property_calendar_time_stamps()) {
        outbuffer += sprintf(outbuffer, "DateTime:");
        outbuffer += printCalendarDate(outbuffer, logEvent->common.nanos_since_1970);
        outbuffer += sprintf(outbuffer, "\n");
        kvstatus = write_file(ptr, (size_t)(outbuffer - ptr));
      }
      break;
    }

    case ILOG_TYPE_VERSION:
    {
      // Version is not implemented in this format.
      break;
    }

    default:
    {
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n", logEvent->common.type, logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  }
  logEvent->common.new_data = false;
  return kvstatus;
}

#define NAME        "Plain text"
#define EXTENSION   "txt"
#define DESCRIPTION "CAN frames in plain text format"

class KvaWriterMaker_PlainAsc : public KvaWriterMaker
{
  public:
    KvaWriterMaker_PlainAsc() : KvaWriterMaker(KVLC_FILE_FORMAT_PLAIN_ASC) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_HLP_J1939] = true;
      propertyList[KVLC_PROPERTY_CALENDAR_TIME_STAMPS] = true;
      propertyList[KVLC_PROPERTY_WRITE_HEADER] = true;
      propertyList[KVLC_PROPERTY_ID_IN_HEX] = true;
      propertyList[KVLC_PROPERTY_DATA_IN_HEX] = true;
      propertyList[KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS ] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_SIZE_LIMIT] = true;
      propertyList[KVLC_PROPERTY_TIME_LIMIT] = true;
      propertyList[KVLC_PROPERTY_LIMIT_DATA_BYTES] = true;
      propertyList[KVLC_PROPERTY_CREATION_DATE] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
      propertyList[KVLC_PROPERTY_TIMEZONE] = true;

      int defaultValue = 6;
      properties.set_property_and_default_value(KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS, &defaultValue, sizeof(defaultValue));

    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      KvaLogWriter *writer = new KvaLogWriter_PlainAsc();
      writer->mProperties = properties;
      return writer;
    }
}  registerKvaLogWriter_PlainAsc;
