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
**  Writer for "Vector Ascii J1587" i.e. ASC files created by CANalyzer et al
** ---------------------------------------------------------------------------
*/

#include "common_defs.h"
#include "KvaLogWriter_VectorJ1587Asc.h"
#include "TimeConv.h"
#include "os_util.h"
#include "kvdebug.h"


#define MAX_CHANNELS   16
#define idMask         0x1FFFFFFF
#define FILL(PTR,COUNT,CHAR)    { int cnt = (COUNT); while (cnt--) *(PTR)++ = (CHAR); }

#define J1587_PCI_TYPE_SINGLE       0
#define J1587_PCI_TYPE_FIRST        5
#define J1587_PCI_TYPE_CONSECUTIVE  6

#define J1587_CAN_ID      0x770 // 1904
#define J1587_TIMING0_ID  0x7F8
#define J1587_DEBUG_ID    0x100 // 256


//--------------------------------------------------------------------------
KvaLogWriter_VectorJ1587Asc::KvaLogWriter_VectorJ1587Asc()
{
  PRINTF(("KvaLogWriter_VectorJ1587Asc::KvaLogWriter_VectorJ1587Asc()\n"));

  for (int i = 0; i < NR_OF_CHANNELS; i++) {
    J1587channel[i].msgComplete = 0;
    J1587channel[i].lastSeqNr = 0;
    J1587channel[i].totalMsgLen = 0;
    J1587channel[i].written = 0;
    J1587channel[i].buf[0] = '\0';
    J1587channel[i].outbuffer[0] = '\0';
    J1587channel[i].lastMsgTime = 0;
  }
}

//--------------------------------------------------------------------------
KvaLogWriter_VectorJ1587Asc::~KvaLogWriter_VectorJ1587Asc()
{ 
  PRINTF(("~KvaLogWriter_VectorJ1587Asc\n"));
}

//--------------------------------------------------------------------------
KvlcStatus KvaLogWriter_VectorJ1587Asc::write_header()
{
  char outbuf[2000];
  char *outbuffer = outbuf;
  time_t aclock;
  struct tm newtime;
  int i;
  int length = 0;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_VectorJ1587Asc::write_header, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  PRINTF(("KvaLogWriter_VectorJ1587Asc::write_header\n"));
  if (get_property_write_header()) {

    aclock = get_wallclock_time();
    get_calendar_time(aclock, &newtime);

    os_rewind(outfile);

    outbuffer[0] = '\0';

    outbuffer += sprintf(outbuffer, "date %2u/%02u/%04u %02u:%02u:%02u\n",
      (newtime.tm_mon+1) % 100U,
      newtime.tm_mday % 100U,
      (newtime.tm_year+1900) % 10000U,
      newtime.tm_hour % 100U,
      newtime.tm_min % 100U,
      newtime.tm_sec % 100U
      );

    outbuffer += sprintf(outbuffer, "base %s timestamps %s\n", get_property_data_in_hex() ? "hex" : "dec",
      get_property_start_of_measurement() ? "absolute" : "relative");

    outbuffer += sprintf(outbuffer, "// CAN channel:");
    for (i=0;i<MAX_CHANNELS;i++) {
      if (get_property_channel_mask()& (unsigned long)(1<<i)) {
        outbuffer += sprintf(outbuffer, " %d", i+1);
      }
    }
    outbuffer += sprintf(outbuffer, "\n");

    length = sprintf(outbuffer, "Begin Triggerblock\n");


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
KvlcStatus KvaLogWriter_VectorJ1587Asc::write_row(imLogData *logEvent)
{
  char outbuf[1000];
  char *outbuffer = outbuf;
  int  length = 0;
  int  i = 0;

  KvlcStatus kvstatus = kvlcOK;

  time_uint64 lastTime = 0;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_VectorJ1587Asc::write_row, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  outbuffer[0] = '\0';

  if (!start_of_logging) setStartOfLogging(logEvent);
  switch (logEvent->common.type) {

    case ILOG_TYPE_TRIGGER:
    {
      // currenty no triggering is supported
      PRINTF(("Err: Got trigger event. Not implemented yet.\n"));
      break;
    }

    case ILOG_TYPE_MESSAGE:
    {
      bool data_in_hex = get_property_data_in_hex();

      if (lastTime > logEvent->common.time64) {
        PRINTF(("WARNING, decreasing time\n"));
      }
      lastTime = logEvent->common.time64;

      if (!((1<<logEvent->msg.channel) & get_property_channel_mask())) {
        // dont use this channel so bail out
        return kvlcOK;
      }

      // Error frame.
      if ((logEvent->msg.flags & canMSG_ERROR_FRAME) != 0) {
        //sprintf(outbuffer, "%sErrorFrame  ", outbuffer);
        PRINTF(("ERROR FRAME\n"));
        break;
      }
      else {
        // message
        unsigned int id = (unsigned int) logEvent->msg.id;

        if (logEvent->msg.channel < NR_OF_CHANNELS) {
          curChan = &J1587channel[logEvent->msg.channel];
        }
        else {
          return kvlcERR_INVALID_LOG_EVENT;
        }

        if ((id == J1587_CAN_ID)) {
          char      tmpbuf[30] = "                             ";
          char      *bufptr = tmpbuf;
          int       n;
          const int width = 10;

          //PRINTF(("CHAN %d\n", logEvent->msg.channel));

          // Channel
          outbuffer += sprintf(outbuffer, "%1d  ", logEvent->msg.channel+1);

          curChan->lastMsgTime = lastTime;

          n = sprintf(tmpbuf, "%s", "J1587");

          bufptr += n+1;
          FILL(bufptr, width-(n+1), ' ');
          tmpbuf[width+1] = '\0';
          outbuffer += sprintf(outbuffer, "%s  ", tmpbuf);
        }
        else if (id == J1587_TIMING0_ID) {
          PRINTF(("J1587_TIMING0_ID\n"));
          if (curChan->msgComplete) {
            time_int64 tOffset;
            tOffset = *(unsigned short*) logEvent->msg.data;
            tOffset = (tOffset * 1000)/16;

            outbuffer += localGetEventTime((time_int64)curChan->lastMsgTime - tOffset, outbuffer);


            // we now have a complete J1587 frame so write to file
            curChan->buf[sizeof(curChan->buf) - 1] = '\0';
            curChan->outbuffer[sizeof(curChan->outbuffer) - 1] = '\0';
            length = sprintf(outbuffer, " %s   %s\n", 
                             curChan->outbuffer, curChan->buf);
            //PRINTF(("out after %s\n", outbuffer));
            outbuffer[length] = '\0';
            outbuffer += length;
            kvstatus = write_file(outbuf, (size_t)(outbuffer - outbuf));
            curChan->msgComplete = false;
            break;
          }
          else {
            PRINTF(("ERROR: Timing message before msg complete "
                       "(outbuffer(%s))\n", curChan->outbuffer));
          }
        }
        else { // not used
          PRINTF(("No J1587 msg (id = %d)\n", id));
          break;
        }
      }

      if (logEvent->msg.flags & canMSGERR_OVERRUN) {
        overrun_occurred = true;
      }

      if (logEvent->msg.dlc > 0) {

        //PRINTF(("pci type = %d\n", (logEvent->msg.data[0] & 0xf0) >> 4));

        // find out which PCI type it is
        switch((logEvent->msg.data[0] & 0xf0) >> 4) {

          // Pci type is read in the first byte (bit 4-7).

          case J1587_PCI_TYPE_SINGLE:
          {
            int dlc = logEvent->msg.dlc;
            curChan->buf[0] = '\0';
            char *curChanBuf = curChan->buf;

            PRINTF(("J1587_PCI_TYPE_SINGLE\n"));

            if (curChan->msgComplete) {
              PRINTF(("ERROR: Message lost because there was no timing message\n"));
              curChan->msgComplete = false;
            }

            // single frame data length is read in first byte (bit 0-3)
            curChan->totalMsgLen = (logEvent->msg.data[0] & 0x0f);
            curChan->written = curChan->totalMsgLen;

            outbuffer += sprintf(outbuffer, "  %3d ", curChan->totalMsgLen);

            //PRINTF(("out dlc %s\n", outbuffer));

            outbuffer += sprintf(outbuffer, "Rx   ");

            //PRINTF(("out RX %s\n", outbuffer));

            // since first byte contains pci type and data length, start from byte 1
            for (i=1; i<dlc && curChan->written; i++) {

              if (data_in_hex) {
                /* guard preventing writing outside curChan->buf */
                if ((curChanBuf - curChan->buf) > ((int)sizeof(curChan->buf)-4))
                   curChanBuf = curChan->buf + ((int)sizeof(curChan->buf)-4);

                curChanBuf += sprintf(curChanBuf, " %02x", (unsigned char)logEvent->msg.data[i] % 0x100U);
              }
              else {
                /* guard preventing writing outside curChan->buf */
                if ((curChanBuf - curChan->buf) > ((int)sizeof(curChan->buf)-5))
                   curChanBuf = curChan->buf + ((int)sizeof(curChan->buf)-5);

                curChanBuf += sprintf(curChanBuf, " %03d", (unsigned char)logEvent->msg.data[i] % 1000U);
              }

              curChan->written--;
            }

            if (curChan->written == 0) {
              curChan->msgComplete = true;
              memcpy(curChan->outbuffer, outbuf, sizeof(curChan->outbuffer));
              curChan->outbuffer[sizeof(curChan->outbuffer) - 1] = '\0';
            }

            break;
         }

          case J1587_PCI_TYPE_FIRST:
          {
            int dlc = logEvent->msg.dlc;
            curChan->lastSeqNr = 0;
            curChan->buf[0] = '\0';
            char *curChanBuf = curChan->buf;

            PRINTF(("J1587_PCI_TYPE_FIRST\n"));

            if (curChan->msgComplete) {
              PRINTF(("ERROR: Message lost because there was no timing message\n"));
              curChan->msgComplete = false;
            }

            // first frame data length is read in first byte (bit 0-3) and second byte
            curChan->totalMsgLen = ((logEvent->msg.data[0] & 0x0f) << 8) +
              logEvent->msg.data[1];
            curChan->written = curChan->totalMsgLen;


            // since first and second byte contains pci type and data length, start from byte 2
            for (i=2; i<dlc && curChan->written; i++) {

              if (data_in_hex) {
                /* guard preventing writing outside curChan->buf */
                if ((curChanBuf - curChan->buf) > ((int)sizeof(curChan->buf) - 4))
                   curChanBuf = curChan->buf + ((int)sizeof(curChan->buf) - 4);

                curChanBuf += sprintf(curChanBuf, " %02x", (unsigned char)logEvent->msg.data[i] % 0x100U);
              }
              else {
                /* guard preventing writing outside curChan->buf */
                if ((curChanBuf - curChan->buf) > ((int)sizeof(curChan->buf) - 5))
                   curChanBuf = curChan->buf + ((int)sizeof(curChan->buf) - 5); 

                curChanBuf += sprintf(curChanBuf, " %03d", (unsigned char)logEvent->msg.data[i] % 1000U);
              }

              curChan->written--;
            }
            break;
          }

          case J1587_PCI_TYPE_CONSECUTIVE:
          {
            int dlc = logEvent->msg.dlc;
            int currentSeqNr = logEvent->msg.data[0] & 0x0f; // sequence number
            char *curChanBuf = &curChan->buf[strlen(curChan->buf)];

            PRINTF(("J1587_PCI_TYPE_CONSECUTIVE (sq = %d)\n", currentSeqNr));

            outbuffer += sprintf(outbuffer, "  %3d ", curChan->totalMsgLen);

            //PRINTF(("out dlc %s\n", outbuffer));

            outbuffer += sprintf(outbuffer, "Rx   ");

            //PRINTF(("out RX %s\n", outbuffer));

            // since first byte contains pci type and SN (sequence number), start from byte 1
            // SN is 1..N (bit 0-3) in first byte
            for (i=1; i<dlc && curChan->written; i++) {

              if (data_in_hex) {
                /* guard preventing writing outside curChan->buf */
                if ((curChanBuf - curChan->buf) > ((int)sizeof(curChan->buf)-4))
                   curChanBuf = curChan->buf + ((int)sizeof(curChan->buf)-4);

                curChanBuf += sprintf(curChanBuf, " %02x", (unsigned char)logEvent->msg.data[i] % 0x100U);
              }
              else {
                /* guard preventing writing outside curChan->buf */
                if ((curChanBuf - curChan->buf) > ((int)sizeof(curChan->buf)-5))
                   curChanBuf = curChan->buf + ((int)sizeof(curChan->buf)-5);

                curChanBuf += sprintf(curChanBuf, " %03d", (unsigned char)logEvent->msg.data[i] % 1000U);
              }

              curChan->written--;
            }


            // make sure SN is correct, increasing from 1 to N.
            if (!(currentSeqNr > curChan->lastSeqNr)) {
              PRINTF(("Err: Sequence number is wrong, should be %d (%d)",
                curChan->lastSeqNr + 1, currentSeqNr));
              return kvlcERR_INVALID_LOG_EVENT;
            }
            curChan->lastSeqNr = currentSeqNr;


            if (curChan->written == 0) {
              curChan->msgComplete = true;
              memcpy(curChan->outbuffer, outbuf, sizeof(curChan->outbuffer));
              curChan->outbuffer[sizeof(curChan->outbuffer) - 1] = '\0';
            }

            break;
          }

          default:
            PRINTF(("Err: Wrong PCI type (%d)\n", logEvent->msg.data[0]));
            break;
        }
      }
      break;
    }

    case ILOG_TYPE_RTC:
    {
      /*
      unsigned int year, month, day, hour, minute, second;

      if (get_property_calendar_time_stamps()) {
        unix_time(logEvent->rtc.rtc, &year, &month, &day, &hour, &minute, &second);
        outbuffer[0] = '\0';
        length = sprintf(outbuffer, "%sDateTime: %4d-%02d-%02d %02d:%02d:%02d\n", outbuffer,
        year, month, day, hour, minute, second);
        outbuffer[length] = '\0';
        kvstatus = write_file(outbuffer, length);
        if (kvstatus != kvlcOK) {
          break;
        }
        //PRINTF(("time lap - %s\n", outbuffer));
      }
      */
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
  } //switch

  logEvent->common.new_data = false;
  return kvstatus;
}


// we override close_file so that we can add stuff at the very end of the file.
KvlcStatus KvaLogWriter_VectorJ1587Asc::close_file()
{
  char outbuffer[100];
  int  length;
  KvlcStatus kvstatus = kvlcOK;

  PRINTF(("KvaLogWriter_VectorJ1587Asc::close_file\n"));

  // move to the end (write_header moves it to the beginning)
  // fseek(...SEEK_END) is not guaranteed to work for > 4 Gbyte.
  // This is supposed to be fine under Win32.
  if (isOpened) {
  fflush(outfile);
  os_fseek(outfile, 0, SEEK_END);

  length = sprintf(outbuffer, "End Triggerblock\n");
  kvstatus = write_file(outbuffer, (size_t)length);
  if (kvstatus != kvlcOK) {
    KvaLogWriter::close_file(); // ignore this error since an error already found
    return kvstatus;
  }
  }
  return KvaLogWriter::close_file();
}


// ****************************************************************************
void KvaLogWriter_VectorJ1587Asc::getEventTime(time_int64 time, char* timerow)
{
  localGetEventTime(time, timerow);
}


uint32_t KvaLogWriter_VectorJ1587Asc::localGetEventTime(time_int64 time, char* timerow)
{
  time_int64 msgTime = time + (time_int64)get_property_offset();
  bool isNegative = msgTime < 0;
  if (isNegative) msgTime = -msgTime;
  time_int64  t1 = msgTime / ONE_BILLION;
  time_uint64 t2 = msgTime % ONE_BILLION;
  if (isNegative) t1 = -t1;

  char *p = timerow;
  p += sprintf(p, "%9" PRId64 "%c", t1, get_property_decimal_char());

  switch (get_property_time_decimals()) {
    case 9: p += sprintf(p, "%09u", (unsigned int) t2); break;
    case 8: p += sprintf(p, "%08u", (unsigned int) t2 / 10); break;
    case 7: p += sprintf(p, "%07u", (unsigned int) t2 / 100); break;
    case 6: p += sprintf(p, "%06u", (unsigned int) t2 / 1000); break;
    case 5: p += sprintf(p, "%05u", (unsigned int) t2 / 10000); break;
    default:
    case 4: p += sprintf(p, "%04u", (unsigned int) t2 / 100000); break;
    case 3: p += sprintf(p, "%03u", (unsigned int) t2 / 1000000); break;
    case 2: p += sprintf(p, "%02u", (unsigned int) t2 / 10000000); break;
    case 1: p += sprintf(p, "%01u", (unsigned int) t2 / 100000000); break;
  }

  return (uint32_t)(p - timerow);  // pointer arithmetic = strlen (timerow)
}

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

#define NAME        "ASCII J1587"
#define EXTENSION   "asc"
#define DESCRIPTION "Vector ASCII J1587"

class KvaWriterMaker_VectorJ1587Asc : public KvaWriterMaker
{
  public:
    KvaWriterMaker_VectorJ1587Asc() : KvaWriterMaker(KVLC_FILE_FORMAT_J1587) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_CALENDAR_TIME_STAMPS] = true;
      propertyList[KVLC_PROPERTY_WRITE_HEADER] = true;
      propertyList[KVLC_PROPERTY_ID_IN_HEX] = true;
      propertyList[KVLC_PROPERTY_DATA_IN_HEX] = true;
      propertyList[KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS ] = true;
      propertyList[KVLC_PROPERTY_ISO8601_DECIMALS] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;

      // Default property values
      properties.set_property_and_default_value(KVLC_PROPERTY_WRITE_HEADER, &PROPERTY_ON, sizeof(PROPERTY_ON));
      properties.set_property_and_default_value(KVLC_PROPERTY_ID_IN_HEX, &PROPERTY_ON, sizeof(PROPERTY_ON));
      properties.set_property_and_default_value(KVLC_PROPERTY_DATA_IN_HEX, &PROPERTY_ON, sizeof(PROPERTY_ON));
      int defaultValue = 4;
      properties.set_property_and_default_value(KVLC_PROPERTY_NUMBER_OF_TIME_DECIMALS, &defaultValue, sizeof(defaultValue));
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
   KvaLogWriter *OnCreateWriter() {
      KvaLogWriter *writer = new KvaLogWriter_VectorJ1587Asc();
      writer->mProperties = properties;
      return writer;
   }
}  registerKvaLogWriter_VectorJ1587Asc;

