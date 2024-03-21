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
**  Writer for "Vector Ascii" i.e. ASC files created by CANalyzer et al
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "KvaLogWriter_VectorAsc.h"
#include "TimeConv.h"
#include "os_util.h"
#include "kvdebug.h"


#define   MAX_CHANNELS   16
#define   idMask         0x1FFFFFFF
#define   HEX_OUT        16
#define   DEC_OUT        10
#define   FILL(PTR,COUNT,CHAR)    { int cnt = (COUNT); while (cnt--) *(PTR)++ = (CHAR); }

//--------------------------------------------------------------------------
static int format_int_left (char *buffer, int value, int width,
                            int radix = 10, bool uppercase = true)
{
  int len;
  switch(radix) {
    case HEX_OUT:
      if (uppercase) {
	len = sprintf(buffer, "%-*X", width, value);
      } else {
	len = sprintf(buffer, "%-*x", width, value);
      }
      break;

    case DEC_OUT:
      len = sprintf(buffer, "%-*d", width, value);
      break;

    default:
      len = 0;
  }
  return len;
}



//--------------------------------------------------------------------------
KvaLogWriter_VectorAsc::KvaLogWriter_VectorAsc()
{
  PRINTF(("KvaLogWriter_VectorAsc::KvaLogWriter_VectorAsc()\n"));
}

KvaLogWriter_VectorAsc::~KvaLogWriter_VectorAsc()
{ 
  PRINTF(("~KvaLogWriter_VectorAsc\n"));
}


bool KvaLogWriter_VectorAsc::get_property_hlp_j1939()
{
  return false;
}

//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_VectorAsc::write_header()
{
  char outbuf[2008];
  char *outbuffer = outbuf;
  time_t aclock;
  struct tm newtime;
  int i;
  int length = 0;
  const  char * weekday[] = {"Sun", "Mon", "Tue", "Wed",
			     "Thu", "Fri", "Sat"};
  const char * month[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
			  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  const char * ampm[] = {"am", "pm"};
  int hour[] = {12, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  time_uint64 unix_seconds = 0;


  if (!isOpened) {
    PRINTF(("KvaLogWriter_VectorAsc::write_header, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  outbuffer[0] = '\0';
  PRINTF(("KvaLogWriter_VectorAsc::write_header\n"));
  if (get_property_write_header()) {

    unix_seconds = get_property_logfile_creation_date();
    if (unix_seconds) {
      get_calendar_time(unix_seconds, &newtime);
    } else {
      aclock = get_wallclock_time();
      get_calendar_time(aclock, &newtime);
    }

    os_rewind(outfile);

    //printCalendarDate()?
    outbuffer += sprintf(outbuffer, "date %s %s %2u %02u:%02u:%02u %s %04u\n",
			 weekday[newtime.tm_wday],
			 month[newtime.tm_mon],
			 newtime.tm_mday,
			 hour[(newtime.tm_hour % 12)],
			 newtime.tm_min,
			 newtime.tm_sec,
			 ampm[(newtime.tm_hour < 12) ? 0 : 1],
			 newtime.tm_year+1900
			 );

    outbuffer += sprintf(outbuffer, "base %s timestamps %s\n", "hex",
			 get_property_start_of_measurement() ? "absolute" : "relative");

    outbuffer += sprintf(outbuffer, "// CAN channel:");
    for (i=0;i<MAX_CHANNELS;i++) {
      if (get_property_channel_mask()& (unsigned long)(1<<i)) {
        outbuffer += sprintf(outbuffer, " %d", i+1);
      }
    }
    outbuffer += sprintf(outbuffer, "\n");

    outbuffer += sprintf(outbuffer, "Begin Triggerblock\n");
    length = sprintf(outbuffer, "// ;    time  can %s           attr dlc data ...  \n", get_property_hlp_j1939()?"PGN":"ident");

    outbuffer[length] = '\0';
    outbuffer += length;
    return write_file(outbuf, (size_t)(outbuffer - outbuf));
  }
  else {
    PRINTF(("skip informational header. \n"));
  }

  return kvlcOK;
}


//--------------------------------------------------------------------------

KvlcStatus KvaLogWriter_VectorAsc::write_row_classic (imLogData *logEvent)
{
  char outbuffer[1000];
  size_t  length = 0;
  char timerow[80];
  KvlcStatus kvstatus = kvlcOK;
  char *obufp = outbuffer;

  time_uint64 lastTime = 0;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_VectorAsc::write_row, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  outbuffer[0] = '\0';

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {

    case ILOG_TYPE_TRIGGER:
    {
      time_int64 msgTime = (time_int64) logEvent->common.time64 +
	(time_int64) get_property_offset();

      printDecimalNumber(timerow, msgTime, get_property_time_decimals());

      length = sprintf(outbuffer, "%17s Log Trigger Event (type=0x%x, active=0x%02x, pre-trigger=%d, post-trigger=%d)\n",
                       timerow,
                       logEvent->trig.type,
                       logEvent->trig.trigNo,
                       (int)logEvent->trig.preTrigger,
                       (int)logEvent->trig.postTrigger);

      kvstatus = write_file(outbuffer, length);
      break;
    }

    case ILOG_TYPE_MESSAGE:
    {
      if (lastTime > logEvent->common.time64) {
        PRINTF(("WARNING, decreasing time\n"));
      }
      lastTime = logEvent->common.time64;

      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        PRINTF(("dont use this channel so bail out\n"));
        return kvlcOK;
      }

      time_int64 msgTime = (time_int64) logEvent->common.time64 +
	(time_int64) get_property_offset();

      printDecimalNumber(timerow, msgTime, get_property_time_decimals());
      obufp += sprintf(obufp, "%17s ", timerow);

      // CAN Identifier
      bool data_in_hex = true; //always on - get_property_data_in_hex();
      bool id_in_hex   = true; //always on - get_property_id_in_hex();

      obufp += sprintf(obufp, "%1d  ", logEvent->msg.channel+1);

      // Error frame.
      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        obufp += sprintf(obufp, "  ErrorFrame  ");
      }
      else {
        // message
        unsigned int id = (unsigned int) logEvent->msg.id;

        if ((logEvent->msg.flags & canMSG_EXT) != 0) {
          id |= 0x80000000;
        }

        // id in hex is the only possible mode.
        if (id_in_hex) {
          char      tmpbuf[30] = "                             ";
          char      *bufptr = tmpbuf;
          int       n;
          const int width = 10;

          n = format_int_left(tmpbuf, (int)(id & idMask), 0, HEX_OUT);
          if ((logEvent->msg.flags & canMSG_EXT) != 0) {
	    tmpbuf[n] = 'x';
          }
          else {
            tmpbuf[n] = ' ';
          }
          tmpbuf[n+1] = '\0';

          bufptr += n+1;
          FILL(bufptr, width-(n+1), ' ');
          tmpbuf[width+1] = '\0';
          obufp += sprintf(obufp, "%s  ", tmpbuf);
        }
        else { // not used
          obufp += sprintf(obufp, "%-10d ", (int)(id & idMask));
        }
      }

      // flags
      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) == 0) {

        if (logEvent->msg.flags & canMSG_TXACK) {
          obufp += sprintf(obufp, "Tx   ");
        }
        else if (logEvent->msg.flags & canMSG_TXRQ) {
          obufp += sprintf(obufp, "TxRq ");
        }
        else {
          obufp += sprintf(obufp, "Rx   ");
        }

        if ((logEvent->msg.flags & canMSG_RTR) != 0) {
          obufp += sprintf(obufp, "r");
        }
        else {
          obufp += sprintf(obufp, "d");
        }
      }

      if ((logEvent->msg.flags & canMSGERR_OVERRUN) != 0) {
        obufp += sprintf(obufp, " #");
      }
      else {
        obufp += sprintf(obufp, "  ");
      }

      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }
      if (logEvent->msg.flags & canFDMSG_FDF) {
        if (dlcToNumBytesFD(logEvent->msg.dlc) > 8) {
          data_truncation_occurred = true;
        }
      }

      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        obufp += sprintf(obufp, "   ");
      }
      else {
        obufp += sprintf(obufp, " %1d ",
			 logEvent->msg.dlc>8 ? 8 : logEvent->msg.dlc);
      }

      // A standard data frame.
      for (int i=0; i<8; i++) {
        if (i >= logEvent->msg.dlc) {
          if (data_in_hex)
            obufp += sprintf(obufp, "   ");
          else
            obufp += sprintf(obufp, "    ");
        }
        else {
          if (data_in_hex)
            obufp += sprintf(obufp, "%02X ",
			     (unsigned char)logEvent->msg.data[i]);
          else
            obufp += sprintf(obufp, "%3d ",
			     (unsigned char)logEvent->msg.data[i]);
        }
      }

      obufp += sprintf(obufp, "\n");

      length = obufp - outbuffer;

      outbuffer[length] = '\0';
      kvstatus = write_file(outbuffer, length);
      break;
    }

    case ILOG_TYPE_RTC:
    {
      if (get_property_calendar_time_stamps()) {
        // This value can be less than previous nanos_since_1970 or since this
        // value currently is low resolution (and probably larger than next)
        length = sprintf(outbuffer, "DateTime:");
        length += printCalendarDate(outbuffer + length, logEvent->common.nanos_since_1970);
        length += sprintf(outbuffer + length, "\n");
        kvstatus = write_file(outbuffer, length);
        if (kvstatus != kvlcOK) {
          break;
        }
        //PRINTF(("time lap - %s\n", outbuffer));
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
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n",
              logEvent->common.type,
              logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  } //switch

  logEvent->common.new_data = false;
  return kvstatus;
}

KvlcStatus KvaLogWriter_VectorAsc::write_row_canfd (imLogData *logEvent)
{
  char outbuffer[1000];
  size_t  length = 0;
  char timerow[80];
  KvlcStatus kvstatus = kvlcOK;
  char *obufp = outbuffer;

  time_uint64 lastTime = 0;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_VectorAsc::write_row, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  outbuffer[0] = '\0';

  if (!start_of_logging) setStartOfLogging(logEvent);

  switch (logEvent->common.type) {

    case ILOG_TYPE_TRIGGER:
    {
      time_int64 msgTime = (time_int64) logEvent->common.time64 +
	(time_int64) get_property_offset();

      printDecimalNumber(timerow, msgTime, get_property_time_decimals());

      length = sprintf(outbuffer, "%17s Log Trigger Event (type=0x%x, active=0x%02x, pre-trigger=%d, post-trigger=%d)\n",
                       timerow,
                       logEvent->trig.type,
                       logEvent->trig.trigNo,
                       (int)logEvent->trig.preTrigger,
                       (int)logEvent->trig.postTrigger);

      kvstatus = write_file(outbuffer, length);
      break;
    }

    case ILOG_TYPE_MESSAGE:
    {
      int brs = 0;
      int esi = 0;
      int fdf = 0;

      if ((logEvent->msg.flags & canFDMSG_FDF) != 0) {
	fdf = 1;
	
	if ((logEvent->msg.flags & canFDMSG_BRS) != 0) {
	  brs = 1;
	}
	
	if ((logEvent->msg.flags & canFDMSG_ESI) != 0) {
	  esi = 1;
	}
      }

      if (lastTime > logEvent->common.time64) {
        PRINTF(("WARNING, decreasing time\n"));
      }
      lastTime = logEvent->common.time64;

      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        PRINTF(("dont use this channel so bail out\n"));
        return kvlcOK;
      }

      time_int64 msgTime = (time_int64) logEvent->common.time64 + (time_int64) get_property_offset();

      printDecimalNumber(timerow, msgTime, get_property_time_decimals());
      obufp += sprintf(obufp, "   %1s ", timerow);

      if (((logEvent->msg.flags & canMSG_ERROR_FRAME) == 0) &&
	  ((logEvent->msg.flags & canMSG_RTR)         == 0)) { 
	obufp += sprintf(obufp, "CANFD %3d ", logEvent->msg.channel+1);

        if (logEvent->msg.flags & canMSG_TXACK) {
          obufp += sprintf(obufp, "Tx   ");
        }
        else if (logEvent->msg.flags & canMSG_TXRQ) {
          obufp += sprintf(obufp, "TxRq ");
        }
        else {
          obufp += sprintf(obufp, "Rx   ");
        }
      }

      if ((logEvent->msg.flags & canMSG_RTR) != 0) {
	obufp += sprintf(obufp, "%4d  ", logEvent->msg.channel+1);
      } 

      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        obufp += sprintf(obufp, " %d  ErrorFrame  ", logEvent->msg.channel+1);
      }
      else {
        // message
        unsigned int id = (unsigned int) logEvent->msg.id;

        if ((logEvent->msg.flags & canMSG_EXT) != 0) {
          id |= 0x80000000;
        }

	{
	  char char_id[10];
          char tmpbuf[30] = "                             ";
	  int  n_byte_id;

          n_byte_id = sprintf(char_id, "%x", (int)(id & idMask));
	  memcpy(&tmpbuf[8-n_byte_id], char_id, (size_t)n_byte_id);

          if ((logEvent->msg.flags & canMSG_EXT) != 0) {
	    tmpbuf[8] = 'x';

          }

	  tmpbuf[10] = '\0';

          obufp += sprintf(obufp, "%10s  ", tmpbuf);

	  if ((logEvent->msg.flags & canMSG_RTR) != 0) {
	    sprintf(tmpbuf, "%d",  (int)(logEvent->msg.id & idMask));
	    obufp += sprintf(obufp, "            --  r  %d  Length = 0 BitCount = 0 ID = %s", (int)logEvent->msg.dlc, tmpbuf);
	    if ((logEvent->msg.flags & canMSG_EXT) != 0) {
	      obufp += sprintf(obufp, "x");
	    }
	  } 
        }
      }

      if (((logEvent->msg.flags & canMSG_ERROR_FRAME) == 0) &&
	  ((logEvent->msg.flags & canMSG_RTR)         == 0)) {

	if ((logEvent->msg.flags & canMSGERR_OVERRUN) != 0) {
	  obufp += sprintf(obufp, " #");
	}
	
	if (logEvent->msg.flags & canMSGERR_OVERRUN) {
	  overrun_occurred = true;
	}
  
	//<BRS> <ESI>
	obufp += sprintf(obufp, "%32d %d", brs, esi);
	
	int num_bytes;
	num_bytes = dlcToNumBytesFD(logEvent->msg.dlc);
	
	obufp += sprintf(obufp, "%2x%3d ", logEvent->msg.dlc, num_bytes);
	
	//print data
	for (int i=0; i<num_bytes; i++) {
	  obufp += sprintf(obufp, "%02x ", (unsigned char)logEvent->msg.data[i]);
	}
	
	obufp += sprintf(obufp, "       0    0  ");

	//<FDF> <BRS> <ESI>
	{
	  int tmp;
	  tmp = fdf;
	  tmp |= (brs << 1);
	  tmp |= (esi << 2);
	  obufp += sprintf(obufp,   " 20%d000        0        0        0        0        0", tmp);
	}
      }

      obufp += sprintf(obufp, "\n");

      length = obufp - outbuffer;

      outbuffer[length] = '\0';
      kvstatus = write_file(outbuffer, length);
      break;
    }

    case ILOG_TYPE_RTC:
    {
      if (get_property_calendar_time_stamps()) {
        // This value can be less than previous nanos_since_1970 or since this
        // value currently is low resolution (and probably larger than next)
        length = sprintf(outbuffer, "DateTime:");
        length += printCalendarDate(outbuffer + length, logEvent->common.nanos_since_1970);
        length += sprintf(outbuffer + length, "\n");
        kvstatus = write_file(outbuffer, length);
        if (kvstatus != kvlcOK) {
          break;
        }
        //PRINTF(("time lap - %s\n", outbuffer));
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
      PRINTF(("logEvent->type = %d, frame_counter = %lu\n",
              logEvent->common.type,
              logEvent->common.event_counter));
      kvstatus = kvlcERR_INVALID_LOG_EVENT;
      break;
    }
  } //switch

  logEvent->common.new_data = false;
  return kvstatus;
}

KvlcStatus KvaLogWriter_VectorAsc::write_row(imLogData *logEvent)
{
  int format = get_property_limit_databytes ();

  if (format == 8) {
    return write_row_classic (logEvent);
  } else {
    return write_row_canfd (logEvent);
  }
}


// we override close_file so that we can add stuff at the very end of the file.
KvlcStatus KvaLogWriter_VectorAsc::close_file()
{
  char outbuffer[100];
  int  length;
  KvlcStatus kvstatus = kvlcOK;

  PRINTF(("KvaLogWriter_VectorAsc::close_file\n"));

  // move to the end (write_header moves it to the beginning)
  // fseek(...SEEK_END) is not guaranteed to work for > 4 Gbyte.
  // This is supposed to be fine under Win32.
  if (isOpened) {
    fflush(outfile);
    os_fseek(outfile, 0, SEEK_END);

    length = sprintf(outbuffer, "End Triggerblock\n");
    kvstatus = write_file(outbuffer, length);
    if (kvstatus != kvlcOK) {
      KvaLogWriter::close_file(); // ignore this error since an error already found
      return kvstatus;
    }
  }
  return KvaLogWriter::close_file();
}


// ****************************************************************************

/*
  The format of a "Vector Ascii" log file is this (based on
  reverse-engineering but we believe it's correct):

  date <date>
  base <number-base>  timestamps <timestamp-type>
  <internal-event-indicator>
  <trigger-block>
  <trigger-block>
  <trigger-block>
  ...


  Comment lines can be inserted in the file. A comment start with a semicolon ';'
  or with two slashes '//' and extends to the end of the line.



  <date>
  The date. Apparently unused because it can appear in several different formats.
  Examples:
  Tue Nov 14 11:02:02 2000
  Wed Jun 5 03:18:58 pm 2002
  Die Nov 14 11:02:02 2000
  Note the German Die(nstag) in the last example.

  <number-base>
  Either "hex" or "dec" (without quotes)

  <timestamp-type>
  Either "absolute" or "relative" (without quotes)

  <internal-event-indicator>
  This is one of the two following strings (without quotes)
  "no internal events logged"
  "internal events logged"


  <trigger-block>
  This is sequence of

  <begin-trigger-block>
  <message-line>
  <message-line>
  <message-line>
  ...
  <end-trigger-block>


  <begin-trigger-block>
  The string "Begin Triggerblock" (without quotes)

  <end-trigger-block>
  The string "End Triggerblock" (without quotes)

  <message-line>
  A line representing a message or an internal event. See below.


  The format of a line for a CAN message is this:
  <timestamp> <channel> <id> <dir> <flags> <dlc> <db0> <db1> <db2> <db3> <db4> <db5> <db6> <db7>

  The fields are separated by one space plus any number of spaces dictated by the field
  widths documented below. Note that the file format is apparently not dependent
  on the fields having exactly the widths described below.

  <timestamp>
  The time stamp of the message, expressed in seconds. The field is 9 characters
  wide with four decimal places and a period as decimal separator. Right-adjusted.
  Example: "   1.0234" (without quotes)
  If the <timestamp-type> (see above) is "absolute" the time values are relative to
  the first time stamp in the file (which is then always 0.0000). If the
  <timestamp-value> is "relative" then the time value on a certain line is
  relative to the preceeding line's time stamp.

  <channel>
  The channel number, 1, 2, 3, ... The field is two characters wide and left-adjusted.

  <id>    The CAN identifier, using the base indicated by the <number-base> field (see above.)
  The field is 15 characters wide and left-adjusted.
  Extended (29-bit) CAN frames are immediately followed by the letter 'x'
  (e.g. 234x)
  Note: if a CAN database is present, this field contains the message's
  symbolic name (if any.)

  <dir>   "Tx" or "Rx" (without quotes) depending on whether the message is transmitted
  or received.

  <flags> A three-character right-adjusted field containing "  d" for data frames
  and "  r" for remote frames (without quotes).

  <dlc>   Data length code - the length of the CAN message. This one-character field
  contains the length of the message, 0-8.

  <db0>   Data byte 0, using the base indicated by the <number-base> field (see above.)
  For hex numbers, this field is two characters wide and contains always
  two characters. This field is not present
  if the <dlc> is shorther than 1, or if the frame is remote.

  <db1>   Analogous to <db0>
  <db2>   Analogous to <db0>
  <db3>   Analogous to <db0>
  <db4>   Analogous to <db0>
  <db5>   Analogous to <db0>
  <db6>   Analogous to <db0>
  <db7>   Analogous to <db0>



  The format of a line for an internal event is this:

  <timestamp> <internal-event>

  <timestamp>
  Same as the time stamps for CAN messages. See above.

  <internal-event>
  A string describing an internal event. Some examples follow:
  "Start of measurement"
  "1  Statistic: D 0 R 0 XD 0 XR 0 E 0 O 0 B 0.0%"
  "CAN 1 Status:chip status error active"

  The internal event strings are not yet fully analyzed by us so no further
  documentation is presently available.


  Example file:

  date Wed Jun 5 03:18:58 pm 2002
  base hex  timestamps absolute
  internal events logged
  Begin Triggerblock
  ;  time  can ident        attr dlc data ...Wed Jun 5 03:18:58 pm 2002
  0.0000 Start of measurement
  1.0000 1  Statistic: D 0 R 0 XD 0 XR 0 E 0 O 0 B 0.0%
  1.0000 2  Statistic: D 0 R 0 XD 0 XR 0 E 0 O 0 B 0.0%
  1.0008 CAN 1 Status:chip status error active
  1.0009 CAN 2 Status:chip status error active
  1.5506 2  123             Rx   d 7 00 00 00 00 00 00 00
  1.5506 1  123             Tx   d 7 00 00 00 00 00 00 00
  1.7329 2  333             Rx   r
  1.7329 1  333             Tx   r
  1.8776 2  3423x           Rx   d 5 00 00 00 00 00
  1.8776 1  3423x           Tx   d 5 00 00 00 00 00
  2.0000 1  Statistic: D 1 R 1 XD 1 XR 0 E 0 O 0 B 0.5%
  2.0000 2  Statistic: D 1 R 1 XD 1 XR 0 E 0 O 0 B 0.5%
  2.0208 2  123             Rx   d 7 00 00 00 00 00 00 00
  2.0208 1  123             Tx   d 7 00 00 00 00 00 00 00
  2.1663 2  333             Rx   r
  2.1663 1  333             Tx   r
  2.3124 2  3423x           Rx   d 5 00 00 00 00 00
  2.3124 1  3423x           Tx   d 5 00 00 00 00 00
  2.4577 2  123             Rx   d 7 00 00 00 00 00 00 00
  2.4577 1  123             Tx   d 7 00 00 00 00 00 00 00
  End TriggerBlock

*/

#define NAME        "Vector ASCII"
#define EXTENSION   "asc"
#define DESCRIPTION "CAN frames in Vector ASCII format"

class KvaWriterMaker_VectorAsc : public KvaWriterMaker
{
public:
  KvaWriterMaker_VectorAsc() : KvaWriterMaker(KVLC_FILE_FORMAT_VECTOR_ASC) {

    // Supported properties
    propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT]     = true;
    propertyList[KVLC_PROPERTY_FIRST_TRIGGER]            = true;
    propertyList[KVLC_PROPERTY_CHANNEL_MASK]             = true;
    propertyList[KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS ] = true;
    propertyList[KVLC_PROPERTY_CROP_PRETRIGGER]          = true;
    propertyList[KVLC_PROPERTY_SIZE_LIMIT]               = true;
    propertyList[KVLC_PROPERTY_TIME_LIMIT]               = true;
    propertyList[KVLC_PROPERTY_CREATION_DATE]            = true;
    propertyList[KVLC_PROPERTY_OVERWRITE]                = true;
    propertyList[KVLC_PROPERTY_TIMEZONE]                 = true;
    propertyList[KVLC_PROPERTY_LIMIT_DATA_BYTES]         = true;

    // Default property values
    properties.set_property_and_default_value(KVLC_PROPERTY_WRITE_HEADER, &PROPERTY_ON, sizeof(PROPERTY_ON));
    properties.set_property_and_default_value(KVLC_PROPERTY_ID_IN_HEX,    &PROPERTY_ON, sizeof(PROPERTY_ON));
    properties.set_property_and_default_value(KVLC_PROPERTY_DATA_IN_HEX,  &PROPERTY_ON, sizeof(PROPERTY_ON));

    int defaultValue = 9;
    properties.set_property_and_default_value(KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS, &defaultValue, sizeof(defaultValue));

    defaultValue = 8; //set classic as default
    properties.set_property_and_default_value(KVLC_PROPERTY_LIMIT_DATA_BYTES, &defaultValue, sizeof(defaultValue));
  }
  int getName(char *str) { return sprintf(str, "%s", NAME); }
  int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
  int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

private:
  KvaLogWriter *OnCreateWriter() {
    KvaLogWriter *writer = new KvaLogWriter_VectorAsc();
    writer->mProperties = properties;
    return writer;
  }
}  registerKvaLogWriter_VectorAsc;

