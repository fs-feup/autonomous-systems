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

#include <stdio.h>
#include <string.h>

#include "kvaConverter.h"
#include "KvaReaderMaker.h"
#include "KvaWriterMaker.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "kvassert.h"

//===========================================================================
KvaConverter::KvaConverter ()
{
  char tmp[] = "KvLogCnvTmp";
  time_uint64 first_jan_1980 = (time_uint64)315532800 * ONE_BILLION;
  PRINTF(("KvaConverter - constructor\n"));

  input_filename[0] = '\0';
  output_filename[0] = '\0';

  input_format = 0;
  output_format = 0;

  logReader = NULL;
  logWriter = NULL;
  input_file_open = false;
  output_file_open = false;

  event_buffer = new bque_imlogdata (65536, tmp);

  data_counter = 0;

  unknown_time_fix = TIME_FIX_UNKNOWN_HRT + TIME_FIX_UNKNOWN_ABS;
  hrt_last_known_time = 0;
  abs_last_known_time = first_jan_1980;

  abs_first_rtc = first_jan_1980;
  abs_first_trigger = first_jan_1980;
  hrt_first_rtc = 0;
  hrt_first_trigger = 0;

  unknown_first_hrt = true;
  unknown_first_rtc = true;

  abs_last_rtc = first_jan_1980;
  abs_last_trigger = first_jan_1980;
  hrt_last_rtc = 0;
  hrt_last_trigger = 0;

  unknown_last_hrt = true;
  unknown_last_rtc = true;

  preTriggerTime = -1;
  keepEvent = 0;
  written_frameno = 0;

  pLogEvent = &logEvent;
  memset(pLogEvent, 0, sizeof(logEvent));
  delta_time = 0;

  file_existence_checked = false;
}

//===========================================================================
KvaConverter::~KvaConverter ()
{
  PRINTF(("KvaConverter - destructor\n"));
  if (event_buffer) {
    PRINTF(("event_buffer->size = %d\n", event_buffer->get_n_element()));
  }
  if (unknown_last_hrt) {
    PRINTF(("unknown_last_hrt still unknown\n"));
  }
  if (unknown_last_rtc) {
    PRINTF(("unknown_last_rtc still unknown\n"));
  }
  if (unknown_first_hrt) {
    PRINTF(("unknown_first_hrt still unknown\n"));
  }
  if (unknown_first_rtc) {
    PRINTF(("unknown_first_rtc still unknown\n"));
  }
  if (logReader) logReader->close_file();
  PRINTF(("If there was a logReader, it has closed its file\n"));
  if (logWriter) logWriter->write_header();
  PRINTF(("If there was a logWriter, it had a header to write and is done\n"));
  if (logWriter) logWriter->close_file();
  PRINTF(("If there was a logWriter, it has closed its file\n"));
  if (logReader) delete logReader;
  PRINTF(("If there was a logReader, it has been deleted\n"));
  if (logWriter) delete logWriter;
  PRINTF(("If there was a logWriter, it has been deleted\n"));
  if (event_buffer) delete event_buffer;
}

//===========================================================================
KvlcStatus KvaConverter::select_reader_format (int format)
{
  if (logReader)
    return kvlcERR_CONVERTING;

  PRINTF(("KvaConverter::select_reader_format(%d)\n", format));

  logReader = KvaReaderMaker::CreateReader(format);
  if (logReader == 0) {
    PRINTF(("Unable to find reader of format %d\n", format));
    return kvlcERR_NOT_IMPLEMENTED;
  }

  input_format = format;
  return kvlcOK;
}

//===========================================================================
bool KvaConverter::is_reading_file ()
{
  return input_file_open;
}

//===========================================================================
KvlcStatus KvaConverter::set_input_file (const char *filename, int format)
{
  KvlcStatus retval;
  strncpy(input_filename, filename, KVACONV_MAX_PATH_LEN);

  retval = select_reader_format(format);
  if (retval != kvlcOK) return retval;

  PRINTF(("calls logRead->open_file\n"));
  retval = logReader->open_file(filename);
  input_file_open = retval == kvlcOK;
  if (retval != kvlcOK) {
    delete logReader;
    logReader = NULL;
  }
  PRINTF(("set_input_file: '%s', format = %d\n",
                input_filename, input_format));
  return retval;
}

//===========================================================================
KvlcStatus KvaConverter::select_writer_format (int format)
{
  if (logWriter)
    return kvlcERR_CONVERTING;

  PRINTF(("KvaConverter::select_writer_format(%d)\n", format));

  logWriter = KvaWriterMaker::CreateWriter(format);
  if (logWriter == 0) {
    PRINTF(("Unable to find writer of format %d\n", format));
    return kvlcERR_NOT_IMPLEMENTED;
  }

  output_format = format;
  return kvlcOK;
}

//===========================================================================
int KvaConverter::getFirstWriterFormat()
{
  return KvaWriterMaker::getFirstFormat();
}

int KvaConverter::getNextWriterFormat(int format)
{
  return KvaWriterMaker::getNextFormat(format);
}

int KvaConverter::getWriterName(int format, char *str)
{
  return KvaWriterMaker::getName(format, str);
}

int KvaConverter::getWriterExtension(int format, char *str)
{
  return KvaWriterMaker::getExtension(format, str);
}

int KvaConverter::getWriterDescription(int format, char *str)
{
  return KvaWriterMaker::getDescription(format, str);
}

int KvaConverter::isWriterPropertySupported(int format, unsigned int property, bool *supported)
{
  return KvaWriterMaker::isPropertySupported(format, property, supported);
}

int KvaConverter::getWriterPropertyDefault(int format, unsigned int property, void *buf, size_t len)
{
  return KvaWriterMaker::getPropertyDefault(format, property, buf, len);
}

//===========================================================================
int KvaConverter::getFirstReaderFormat()
{
  return KvaReaderMaker::getFirstFormat();
}

int KvaConverter::getNextReaderFormat(int format)
{
  return KvaReaderMaker::getNextFormat(format);
}

int KvaConverter::getReaderName(int format, char *str)
{
  return KvaReaderMaker::getName(format, str);
}

int KvaConverter::getReaderExtension(int format, char *str)
{
  return KvaReaderMaker::getExtension(format, str);
}

int KvaConverter::getReaderDescription(int format, char *str)
{
  return KvaReaderMaker::getDescription(format, str);
}

int KvaConverter::isReaderPropertySupported(int format, unsigned int property, bool *supported)
{
  return KvaReaderMaker::isPropertySupported(format, property, supported);
}

int KvaConverter::getReaderPropertyDefault(int format, unsigned int property, void *buf, size_t len)
{
  return KvaReaderMaker::getPropertyDefault(format, property, buf, len);
}

//===========================================================================
KvlcStatus KvaConverter::set_output_format (int format)
{
  PRINTF(("KvaConverter::set_output_format(%d)\n", format));
  return select_writer_format(format);
 }

//===========================================================================
KvlcStatus KvaConverter::get_output_format (int *format)
{
  if (!format) return kvlcERR_PARAM;
  if (!logWriter) return kvlcERR_NULL_POINTER;
  *format = output_format;
  return kvlcOK;
 }

//===========================================================================
KvlcStatus KvaConverter::set_output_file (const char *filename)
{
  if (!filename) return kvlcERR_PARAM;
  PRINTF(("KvaConverter::set_output_file(%s)\n", filename));
  if (logWriter == 0) return kvlcERR_NULL_POINTER;

  strncpy(output_filename, filename, KVACONV_MAX_PATH_LEN);
  return logWriter->set_filename((const char *) output_filename);
}

//===========================================================================
KvlcStatus KvaConverter::get_output_filename (
    char *filename,
    size_t len)
{
  if (logWriter == 0) return kvlcERR_NULL_POINTER;
  return logWriter->get_filename(filename, len);
}

//===========================================================================
KvlcStatus KvaConverter::get_output_filename_status (
    bool *updated)
{
  return logWriter->get_filename_status(updated);
}

//===========================================================================
KvlcStatus KvaConverter::feed_event (void *event)
{
  if (!logReader) {
    return kvlcERR_NO_INPUT_SELECTED;
  }
  else if (is_reading_file()) {
    return kvlcFail;
  }
  pLogEvent->common.new_data = false;
  return logReader->interpret_event(event, pLogEvent);
}

//===========================================================================
KvlcStatus KvaConverter::next_file ()
{
  PRINTF(("Next file - sets unknown_time to true\n"));
  if (!logReader || !logWriter) {
    PRINTF(("logReader is NULL\n"));
    return kvlcERR_NO_INPUT_SELECTED;
  }
  PRINTF(("event_buffer->size = %d", event_buffer->get_n_element()));
  if (unknown_last_hrt) {
    PRINTF(("unknown_last_hrt still unknown\n"));
  }
  if (unknown_last_rtc) {
    PRINTF(("unknown_last_rtc still unknown\n"));
  }
  if (unknown_first_hrt) {
    PRINTF(("unknown_first_hrt still unknown\n"));
  }
  if (unknown_first_rtc) {
    PRINTF(("unknown_first_rtc still unknown\n"));
  }
  preTriggerTime = -1;
  keepEvent = 0;
  unknown_last_hrt = true;
  unknown_last_rtc = true;
  unknown_time_fix = TIME_FIX_UNKNOWN_HRT + TIME_FIX_UNKNOWN_ABS;
  logWriter->next_file();
  return logReader->next_file();
}

//===========================================================================
KvlcStatus KvaConverter::next_input_file (const char *filename)
{
  PRINTF(("KvaConverter::next_input_file()\n"));

  KvlcStatus retval;

  if (!filename) {
    return kvlcERR_NULL_POINTER;
  }

  if (!logReader || !logWriter) {
    return kvlcERR_NO_INPUT_SELECTED;
  }

  PRINTF(("event_buffer->size = %d\n", event_buffer->get_n_element()));
  if (unknown_last_hrt) {
    PRINTF(("unknown_last_hrt still unknown\n"));
  }
  if (unknown_last_rtc) {
    PRINTF(("unknown_last_rtc still unknown\n"));
  }
  if (unknown_first_hrt) {
    PRINTF(("unknown_first_hrt still unknown\n"));
  }
  if (unknown_first_rtc) {
    PRINTF(("unknown_first_rtc still unknown\n"));
  }
  preTriggerTime = -1;
  keepEvent = 0;
  unknown_last_hrt = true;
  unknown_last_rtc = true;
  unknown_time_fix = TIME_FIX_UNKNOWN_HRT + TIME_FIX_UNKNOWN_ABS;
  logWriter->next_file();

  retval = logReader->close_file();
  if (kvlcOK != retval) return retval;

  retval = logReader->open_file(filename);
  if (kvlcOK != retval) return retval;

  return logReader->next_file();
}

//===========================================================================
KvlcStatus KvaConverter::row_count (uint64 *count)
{
  if (logReader) {
    *count = logReader->event_count();
    return kvlcOK;
  }
  else {
    return kvlcERR_NO_INPUT_SELECTED;
  }
}

//===========================================================================
KvlcStatus KvaConverter::write_header ()
{
  return logWriter->write_header();
}

//===========================================================================
void setAbsTime(imLogData *tmp, time_int64 dt, time_int64 abs_t)
{
   switch (tmp->common.type) {
    case ILOG_TYPE_TRIGGER: {
      tmp->common.nanos_since_1970 = tmp->common.time64 + abs_t;
      break;
    }
    case ILOG_TYPE_MESSAGE: {
      tmp->common.nanos_since_1970 = tmp->common.time64 + abs_t + dt;
      break;
    }
  }
}

//===========================================================================
KvlcStatus KvaConverter::convert_event ()
{
  using namespace std;
  imLogData tmp;
  time_int64 preTriggerEdge = 0;
  KvlcStatus stat;

  if (!logReader) {
    return kvlcERR_NO_INPUT_SELECTED;
  }

  // Open output file and write header if needed
  if (!output_file_open) {
    // Do a file existence check on the first invocation of convert_event
    if ( !file_existence_checked && logWriter->mProperties.mOverwrite == logWriter->mProperties.KVLC_FILE_DO_NOT_OVERWRITE) {
      stat = logWriter->file_existence_status();
      if (stat != kvlcOK) return stat;
    }
    file_existence_checked = true;
    stat = logWriter->open_file();
    if (stat != kvlcOK) return stat;
    output_file_open = true;
    stat = logWriter->write_header();
    if (stat != kvlcOK) return stat;
  }

  KvlcStatus reta = kvlcERR_INTERNAL_ERROR, retb = kvlcERR_INTERNAL_ERROR;
  if (is_reading_file()) {
    pLogEvent->common.new_data = false;
    reta = logReader->read_row(pLogEvent);

    if (reta == kvlcOK && pLogEvent->common.type == ILOG_TYPE_EXTERNAL){
      return reta;
    }
  }
  else {
    if (input_format != 0) {
      reta = kvlcOK; // Suppose that something has fed the reader
      //PRINTF(("Got %d from stream", pLogEvent->event_counter));
      //PRINTF(("New: %d Typ: %d Time64: %I64u", pLogEvent->new_data,
      // pLogEvent->type, pLogEvent->trig.time64));
    }

  }

  if (pLogEvent->common.new_data) {
    data_counter++;
    retb = kvlcOK;


    // If events should be buffered
    if (unknown_last_rtc || unknown_last_hrt || unknown_time_fix) {
      //PRINTF(("Push %d", pLogEvent->event_counter));
      if (!event_buffer->push(pLogEvent)) {
        return kvlcERR_INTERNAL_ERROR;
      }

      // Unknown abs time
      if (unknown_time_fix & TIME_FIX_UNKNOWN_ABS && pLogEvent->common.type == ILOG_TYPE_RTC) {
        PRINTF(("TIME_FIX_UNKNOWN_ABS, abs_last_known_time = %lu.%09lu\n", pLogEvent->common.nanos_since_1970 / ONE_BILLION, pLogEvent->common.nanos_since_1970 % ONE_BILLION));
        unknown_time_fix &= ~TIME_FIX_UNKNOWN_ABS;
        abs_last_known_time = pLogEvent->common.nanos_since_1970;
      }
      else
      // Unknown hires timer
      if (unknown_time_fix & TIME_FIX_UNKNOWN_HRT && pLogEvent->common.type != ILOG_TYPE_RTC) {
        unknown_time_fix &= ~TIME_FIX_UNKNOWN_HRT;
        switch (pLogEvent->common.type) {
        case ILOG_TYPE_TRIGGER:
        case ILOG_TYPE_MESSAGE:
        case ILOG_TYPE_CANOTHER:
          hrt_last_known_time = pLogEvent->common.time64;
          break;
        default: {
          PRINTF(("TIME_FIX_UNKNOWN failed\n"));
        }
        }
        PRINTF(("TIME_FIX_UNKNOWN_HRT, hrt_last_known_time = %lu.%09lu\n", hrt_last_known_time / ONE_BILLION, hrt_last_known_time % ONE_BILLION));
      }

      if (unknown_last_hrt && pLogEvent->common.type == ILOG_TYPE_TRIGGER) {
        PRINTF(("unknown_last-hrt && pLogEvent is a trigger event\n"));

        unknown_last_hrt = false;
        hrt_last_trigger = pLogEvent->common.time64;
        // Get the first active trigger with pre-trigger > 0 (memolib will se old versions to -1).
        if (pLogEvent->trig.active && (preTriggerTime < 0) && (pLogEvent->trig.preTrigger > 0)) {
          preTriggerTime = pLogEvent->trig.preTrigger;
          PRINTF(("preTriggerTime %ld\n",preTriggerTime));
          preTriggerEdge = pLogEvent->common.time64 - (time_int64)preTriggerTime * (time_int64)1000000;
          if (preTriggerEdge < 0) {
            // The first defined event time in the system is zero and a negative edge indicates that the
            // first pretrigger after power-on wasn't full when the first trigger arrived.
            preTriggerEdge = 0;
          }
        }
      }


      if (unknown_last_rtc && pLogEvent->common.type == ILOG_TYPE_RTC) {
        PRINTF(("Got first ILOG_TYPE_RTC\n"));
        unknown_last_rtc = false;
        abs_last_rtc = pLogEvent->common.nanos_since_1970;
      }

      if (!unknown_last_rtc && !unknown_last_hrt && !unknown_time_fix) {
        // Consider abs_last_known_time to have happen at the same time as
        // hrt_last_known_time.
        //
        // We know the following values:
        //   hrt_last_trigger
        //   abs_last_rtc
        //   hrt_last_known_time
        //   abs_last_known_time
        // We want to know the following values also:
        //   abs_last_trigger
        //   hrt_last_rtc
        //   delta_time
        time_int64 m = abs_last_known_time - hrt_last_known_time;
        abs_last_trigger = hrt_last_trigger + m;
        hrt_last_rtc = abs_last_rtc - m;
        if (unknown_first_hrt) {
          unknown_first_hrt = false;
          if (logWriter->get_property_start_of_measurement()) {
            hrt_first_trigger = hrt_last_trigger;
            abs_first_trigger = abs_last_trigger;
            PRINTF(("Trigger offset = %lu\n", (time_int64)0));
            logWriter->set_property_offset(0);
          }
          else {
            hrt_first_trigger = hrt_last_trigger;
            abs_first_trigger = abs_last_trigger;
            PRINTF(("Trigger offset = %lu\n", -hrt_first_trigger));
            logWriter->set_property_offset(-hrt_first_trigger);
          }
        }
        if (unknown_first_rtc) {
          unknown_first_rtc = false;
          abs_first_rtc = abs_last_rtc;
          hrt_first_rtc = hrt_last_rtc;
        }
        delta_time = abs_last_rtc - abs_first_trigger;

        PRINTF(("HAVE---------------------------------\n"));
        PRINTF(("hrt_first_trigger = %lu.%09lu\n", hrt_first_trigger / ONE_BILLION, hrt_first_trigger % ONE_BILLION));
        PRINTF(("abs_first_trigger = %lu.%09lu\n", abs_first_trigger / ONE_BILLION, abs_first_trigger % ONE_BILLION));
        PRINTF(("hrt_last_trigger = %lu.%09lu\n", hrt_last_trigger / ONE_BILLION, hrt_last_trigger % ONE_BILLION));
        PRINTF(("abs_last_rtc = %lu.%09lu\n", abs_last_rtc / ONE_BILLION, abs_last_rtc % ONE_BILLION));
        PRINTF(("hrt_last_known_time = %lu.%09lu\n", hrt_last_known_time / ONE_BILLION, hrt_last_known_time % ONE_BILLION));
        PRINTF(("abs_last_known_time = %lu.%09lu\n", abs_last_known_time / ONE_BILLION, abs_last_known_time % ONE_BILLION));
        PRINTF(("CALC---------------------------------\n"));
        PRINTF(("abs_last_trigger = %lu.%09lu\n", abs_last_trigger / ONE_BILLION, abs_last_trigger % ONE_BILLION));
        PRINTF(("hrt_last_rtc = %lu.%09lu\n", hrt_last_rtc / ONE_BILLION, hrt_last_rtc % ONE_BILLION));
        PRINTF(("delta_time = %lu.%09lu\n", delta_time / ONE_BILLION, delta_time % ONE_BILLION));
        PRINTF(("m = %lu.%09lu\n", m / ONE_BILLION, m % ONE_BILLION));

        PRINTF(("retb = %d before while with %d loops\n",
                 retb, event_buffer->get_n_element()));
        while(event_buffer->get_n_element() != 0 && retb == kvlcOK) {
          if (!event_buffer->pop(&tmp)) {
            PRINTF(("kvaConverter couldn't pop event_buffer, breaking\n"));
            ASSERT(false);
            return kvlcERR_INTERNAL_ERROR;
          }
          // Set ABS & TS on all events
          setAbsTime(&tmp, delta_time, abs_first_trigger - hrt_first_trigger);

          // Select pre-trigger events to keep
          if (logWriter->get_property_crop_pretrigger() && preTriggerTime > 0) {
            switch (tmp.common.type) {
              case ILOG_TYPE_TRIGGER:
              case ILOG_TYPE_MESSAGE:
              case ILOG_TYPE_CANOTHER:
                  if ( (preTriggerEdge<0) || (tmp.common.time64 >= (time_uint64)preTriggerEdge) ) keepEvent = 1;
                  break;
              case ILOG_TYPE_RTC:
              case ILOG_TYPE_VERSION: {
                } break;
            }
          }
          else {
            keepEvent = 1;
          }

          if (keepEvent) {
            if (tmp.common.type == ILOG_TYPE_MESSAGE) tmp.msg.frame_counter = ++written_frameno;
            retb = logWriter->write_event(&tmp);
            //PRINTF(("buffered write of event %d", tmp.event_counter));
          }
        }
      }
    }
    else
    {
      setAbsTime(pLogEvent, delta_time, abs_first_trigger - hrt_first_trigger);
      if (pLogEvent->common.type == ILOG_TYPE_MESSAGE) pLogEvent->msg.frame_counter = ++written_frameno;
      retb = logWriter->write_event(pLogEvent);
      //PRINTF(("fast write of event %d", pLogEvent->event_counter));
    }
  }
  else {
    // it is ok not to have new data all the time
    retb = kvlcOK;
  }
  // else ... not every event yields an imLogData
  if (reta == kvlcOK && retb == kvlcOK)
    return kvlcOK;
  else {
    if (reta != kvlcOK) {
      PRINTF(("logReader->read_row(pLogEvent) failed with error %d\n", reta));
      return reta;
    }
    if (retb != kvlcOK) {
      PRINTF(("logWriter->write_event(pLogEvent) failed with error %d\n", retb));
      return retb;
    }
    return kvlcERR_INTERNAL_ERROR;
  }
}

//===========================================================================
KvlcStatus KvaConverter::flush_events()
{
  // Memorator can sometimes produce files without any triggers,
  // so flush any messages in the queue before we are destroyed.
  using namespace std;
  imLogData tmp;
  KvlcStatus retb = kvlcOK;
  PRINTF(("KvaConverter::flush_events: Flushing...\n"));
  while(event_buffer->get_n_element() != 0 && retb == kvlcOK) {
    if (!event_buffer->pop(&tmp)) {
      PRINTF(("kvaConverter couldn't pop event_buffer, breaking\n"));
      ASSERT(false);
      return kvlcERR_INTERNAL_ERROR;
    }
    // Set ABS & TS on all events
    setAbsTime(&tmp, delta_time, abs_first_trigger - hrt_first_trigger);

    if (tmp.common.type == ILOG_TYPE_MESSAGE) {
      tmp.msg.frame_counter = ++written_frameno;
    }

    retb = logWriter->write_event(&tmp);
    //PRINTF(("buffered write of event %d", tmp.event_counter));
  }
  PRINTF(("KvaConverter::flush_events: Done.\n"));

  return retb;
}

//===========================================================================
KvlcStatus KvaConverter::add_database_file(const char *filename,
                                           unsigned int channel_mask)
{
  if (logWriter) {
    return logWriter->add_database_file(filename, channel_mask);
  }
  PRINTF(("KvaConverter::add_database_file : NULL POINTER\n"));
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::add_database_handle(KvaDbHnd dh,
                                             unsigned int channel_mask)
{
  if (logWriter) {
    return logWriter->add_database_handle(dh, channel_mask);
  }
  PRINTF(("KvaConverter::add_database_handle : NULL POINTER\n"));
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::get_status_overrun (bool *overrun)
{
  if (logWriter && overrun) {
    *overrun = logWriter->get_status_overrun();
    return kvlcOK;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::reset_status_overrun ()
{
  if (logWriter) {
    logWriter->reset_status_overrun();
    return kvlcOK;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::get_status_data_truncated (bool *truncated)
{
  if (logWriter && truncated) {
    *truncated = logWriter->get_status_data_truncated();
    return kvlcOK;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::reset_status_data_truncated ()
{
  if (logWriter) {
    logWriter->reset_status_data_truncated();
    return kvlcOK;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::get_status_dlc_mismatch(bool *dlc_mismatch)
{
	if (logWriter && dlc_mismatch) {
		*dlc_mismatch = logWriter->get_status_dlc_mismatch();
		return kvlcOK;
	}
	return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::reset_status_dlc_mismatch()
{
	if (logWriter) {
		logWriter->reset_status_dlc_mismatch();
		return kvlcOK;
	}
	return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::get_list_dlc_mismatch(uint32_t* MsgIds, uint32_t* MsgDlc, uint32_t* MsgOccurance, uint32_t* length)
{
	PRINTF(("KvaConverter::get_list_dlc_mismatch\n"));
	if (logWriter) {
		return logWriter->get_list_dlc_mismatch(MsgIds, MsgDlc, MsgOccurance, length);
	}
	return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::get_writer_size_estimate(long long *size_estimate)
{
  if (logWriter && size_estimate) {
    *size_estimate = logWriter->get_bytes_written();
    return kvlcOK;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::get_last_timestamp(time_uint64 *timestamp)
{
  if (NULL != timestamp && NULL != pLogEvent) {
    if (ILOG_TYPE_MESSAGE != pLogEvent->common.type) {
      return kvlcERR_NO_TIME_REFERENCE;
    }
    *timestamp = pLogEvent->common.time64;
    return kvlcOK;
  }
  return kvlcERR_NULL_POINTER;
}

//===========================================================================
KvlcStatus KvaConverter::attach_file (
    const char *filename)
{
  PRINTF(("KvaConverter::attach_file(%s)\n", filename));

  if (logWriter == 0) return kvlcERR_NULL_POINTER;

  return logWriter->attach_file(filename);
}

//===========================================================================
KvlcStatus KvaConverter::set_property_value (
    unsigned int property,
    void *buf, size_t len)
{
  if (logWriter) {
    return logWriter->set_property_value(property, buf, len);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

//===========================================================================
KvlcStatus KvaConverter::get_property_value (
    unsigned int property,
    void *buf, size_t len)
{
  if (logWriter) {
    return logWriter->get_property_value(property, buf, len);
  }
  else {
    return kvlcERR_NULL_POINTER;
  }
}

#if defined(TESTEXE)
int main (int argc, char **argv)
{
  int loopvar = 0;

  for (loopvar = 0; loopvar < 1; loopvar++) {

    KvaConverter *foo = new KvaConverter();
    bool progressBar = false;
    int i;
    KvlcStatus stat;
    char tmpstr[500];
    char tmpchar;
    char infile[KVACONV_MAX_PATH_LEN + 1];
    char outfile[KVACONV_MAX_PATH_LEN + 1];
    char dbfile[KVACONV_MAX_PATH_LEN + 1];

    infile[0] = '\0';
    outfile[0] = '\0';
    dbfile[0] = '\0';

    for (i = 1; i < argc; i++) {
      if (strcmp(argv[i], "-p") == 0) {
        progressBar = true;
      }
      else if (sscanf(argv[i], "-i%1000s%c", tmpstr, &tmpchar) == 1) {
        strncpy(infile, tmpstr, 1000);
      }
      else if (sscanf(argv[i], "-o%1000s%c", tmpstr, &tmpchar) == 1) {
        strncpy(outfile, tmpstr, 1000);    }
      else if (sscanf(argv[i], "-d%1000s%c", tmpstr, &tmpchar) == 1) {
        strncpy(dbfile, tmpstr, 1000);
      }
    }

    if (infile[0] == '\0' || outfile[0] == '\0') {
      strncpy(infile,
        "\\temp\\kvaconverter\\j_trig_rtc.kme",
        KVACONV_MAX_PATH_LEN);
      strncpy(outfile,
        "\\temp\\kvaconverter\\logfile.csv",
        KVACONV_MAX_PATH_LEN);
    }

    if (dbfile[0] = '\0') {
      //strncpy(dbfile, "\\temp\\kvaconverter\\J.dbc", KVACONV_MAX_PATH_LEN);
    }

    stat = foo->set_input_file (
          infile,
          KVLC_FILE_FORMAT_KME25
          );
    if (stat == kvlcOK) {
      printf("Input file '%s' was opened without problems\n", infile);
    }
    else {
      printf("Input file '%s' was not opened, %d\n", infile, i);
      delete foo;
      return kvlcERR_FILE_ERROR;
    }

    if (foo->set_output_format(KVLC_FILE_FORMAT_CSV) == kvlcOK) {
      printf("Output format KVLC_FILE_FORMAT_CSV was created without problems\n");
    }
    else {
      printf("Output format KVLC_FILE_FORMAT_CSV was not opened\n");
      delete foo;
      return kvlcERR_FILE_ERROR;
    }

    if (foo->set_output_file(outfile) == kvlcOK) {
      printf("Output file '%s' was opened without problems\n", outfile);
    }
    else {
      printf("Output file '%s' was not opened\n", outfile);
      delete foo;
      return kvlcERR_FILE_ERROR;
    }

    //foo->set all properties...
    //foo->set_property_calendar_time_stamps(true);
    //foo->set_property_write_header(true);
    //foo->set_offset(12346579000);

    // Convert file -> file
    {
      uint64 total, cnt, i;
      i = 0;
      foo->row_count(&total);
      if (foo->write_header() != kvlcERR_FILE_ERROR) {
	while(cnt > 0) {
	  if (progressBar) {
	    i++;
	    if (i % (total/78) == 0) {
	      printf("\r[");
	      for (uint64 j=1;j < 78;j++) {
		if (j <= (78*i)/total) {
		  printf( "*");
		}
		else {
		  printf(" ");
		}
	      }
	      printf("]");
	    }
	  }
	  if (foo->convert_event() != kvlcOK) {
	    printf("Conversion failed\n");
	    break;
	  }
	  foo->row_count(&cnt);
	} printf("\n");
      }

      foo->row_count(&cnt);
      if (cnt == 0) {
	printf("No more events to convert, conversion finished!!!\n");
      }
    }

  // Convert memorator -> file
  /* pseudo code ;-)
    while(more_to_read_from_memorator) {
      read_from_memorator(&kmeEvent)
      foo->feed_kme_event(&kmeEvent)
      foo->convert_event()
    }
  */

    delete foo;
  }
  return kvlcOK;
}
#endif
