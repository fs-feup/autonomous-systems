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

#ifndef KVACONVERTER_H_
#define KVACONVERTER_H_

#include "common_defs.h"

#include <vector>
#include "bque_imlogdata.h"
#include "KvaConverterMisc.h"
#include "KvaLogReader.h"
#include "KvaLogWriter.h"



class KvaConverter {
  private:
    char input_filename[KVACONV_MAX_PATH_LEN + 1];
    char output_filename[KVACONV_MAX_PATH_LEN + 1];

    int input_format;
    int output_format;

    KvaLogReader *logReader;
    KvaLogWriter *logWriter;
    bool input_file_open;
    bool output_file_open;
    bool file_existence_checked;

    bque_imlogdata *event_buffer;

    // Data counter counts the number of events with new_data == true
    // that has been received
    unsigned long data_counter;

    // Does HRT and RTC have a correlation in time
    #define TIME_FIX_UNKNOWN_HRT 0x1
    #define TIME_FIX_UNKNOWN_ABS 0x2
    uint8_t unknown_time_fix; // bitmask bit 0 means unknown hrt, bit 1 means unknown abs
    time_int64  hrt_last_known_time;
    time_uint64 abs_last_known_time; // nanos since 1970

    // RTC time of first rtc (nanos since 1970)
    time_uint64 abs_first_rtc;
    // RTC time of first trigger (nanos since 1970)
    time_uint64 abs_first_trigger;
    // High res timer of first rtc (as good as possible)
    time_int64  hrt_first_rtc;
    // High res timer from reader's value of first trigger
    time_int64  hrt_first_trigger;

    // The hi res time is unknown in first file
    bool unknown_first_hrt;
    // The absolute time is unknown in first file
    bool unknown_first_rtc;

    // RTC time of last RTC (which is the first RTC in current file)
    time_uint64 abs_last_rtc;
    // RTC time of last trigger
    time_uint64 abs_last_trigger;
    // High res timer of last rtc (as good as possible)
    time_int64  hrt_last_rtc;
    // High res timer from reader's value of last trigger
    time_int64  hrt_last_trigger;

    // The high res time is unknown in the last file
    bool unknown_last_hrt;
    // The absolute time is unknown in the last file
    bool unknown_last_rtc;

    // delta_time is the time between abs_last_rtc and abs_first_trigger,
    // should be updated after time next_file has been called when we
    // actually know the delta_time...
    // Is used when merging files
    time_int64 delta_time;


    long preTriggerTime;              // Time in ms.
    int keepEvent;                    // Should pre-trigger message be kept
    unsigned long   written_frameno;  // Used to cover up the cropped messages

  protected:
    imLogData logEvent;
    imLogData *pLogEvent;

  public:
    KvlcStatus add_database_file(const char *filename, unsigned int channel_mask);
    KvlcStatus add_database_handle(KvaDbHnd dh, unsigned int channel_mask);
    void close();

    KvaConverter();
    ~KvaConverter();

    /**
    * set_input_file(char *filename, int format)
    *   Opens the file with name 'filename' for reading as if it were a file of
    *   format 'format'.
    *   Initializes logReader
    **/
    KvlcStatus set_input_file(const char *filename, int format);

    /**
    * set_output_format(char *filename, int format)
    *
    *   Initializes logWriter with format 'format'.
    **/
    KvlcStatus set_output_format(int format);

    /**
    *   Returns member output_format
    **/
    KvlcStatus get_output_format(int *format);

    /**
    * set_output_file(const char *filename, int format)
    *   Opens the file with name 'filename' for writing as if it were a file of
    *   format 'format' set with set_output_format().
    **/
    KvlcStatus set_output_file(const char *filename);

    /**
    * feed_event(void *event)
    *   Converts an event into a intermediate logEvent.
    *   Useful for streaming events from a Memorator.
    *
    *   Returns KVACONV_NULL_POINTER if no reader
    *   Returns KVACONV_OTHER_ERROR  if file is opened for reading
    *   Returns KVACONV_NOT_IMPLEMENTED if no convert_event implemented
    *   Otherwise, it returns the result from reader->convert_event(event,...)
    **/
    KvlcStatus feed_event(void *event);

    /**
    * Tells the converter that it should updates it's clocks since there
    * is a new logfile on it's way in.
    **/
    KvlcStatus next_file();

    /**
    * Tells the converter that it should updates it's clocks since there
    * is a new logfile on it's way in.
    **/
    KvlcStatus next_input_file (const char *filename);

    /**
    * select_reader_format(int format), select_writer_format(int format)
    *   Selects the format of the reader and the writer, use one of the
    *   definitions FILE_FORMAT_* to select format.
    *   Returns
    *     KVACONV_NOT_IMPLEMENTED if format is intended to be implmented.
    *     KVACONV_OTHER_ERROR if format isn't planned to be implemented,
    *     the format is unknown or format already set, create new converter
    *     to change format.
    *     KVACONV_OK if everything works fine.
    **/
    KvlcStatus  select_reader_format(int format);
    KvlcStatus  select_writer_format(int format);

    /**
    * is_reading_file() returns true if reading input from file, false otherwise
    **/
    bool is_reading_file();

    /**
    * write_header()
    **/
    KvlcStatus write_header();

    /**
    * convert_event()
    *   In case of file->file conversion:
    *     If logEvent is set, then write a translation of it to the output_file,
    *     otherwise read one event from the input_file and write it to the
    *     output_file. The logEvent is marked as used.
    *     Will most probably decrease eventCount.
    *
    *   In case of memorator->file conversion:
    *     If there is anything to convert in logEvent,
    *     then write it to the output_file,
    *     otherwise do nothing.
    *
    *   Returns the number of rows written. This can be from 0 and upwards.
    *   Returns -1 if the logEvent was used already.
    *   Returns -2 if problem with handling the output_file (write protected,
    *   access denied etc).
    **/
    KvlcStatus convert_event();

    /**
    * flush_events()
    *   Flushes any remaining queued messages to the output_file.
    *   (Can only happen if there was no trigger in the input_file,
    *    and only Memorator can create such a file.)
    *
    *   Returns kvlcERR_INTERNAL_ERROR on queue problems ("impossible").
    *      else whatever error code logWriter->write_row() reports.
    **/
    KvlcStatus flush_events();

    /**
    * row_count()
    *   Returns the approximative number of events in the input_file.
    *   Returns a number > 0 when there still are events to convert.
    **/
    KvlcStatus row_count(uint64 *count);

    /**
    * Return overrun status (i.e. did we see overrun flags during
    * conversion?
    **/
    KvlcStatus get_status_overrun(bool *overrun);

    /**
    * Reset overrun status
    **/
    KvlcStatus reset_status_overrun();

    /**
    * Return data truncated status (i.e. trying to save more than 8 byte CANFD
    * message data to a format that only supports 8 byte).
    **/
    KvlcStatus get_status_data_truncated(bool *truncated);

    /**
    * Reset data truncated status
    **/
    KvlcStatus reset_status_data_truncated();

    /**
    * Return dlc mismatch status (i.e. CAN id is found in db but it has
	* mismatching dlc)
    **/
	KvlcStatus get_status_dlc_mismatch(bool *dlc_mismatch);

	/**
	* Reset dlc mismatch status
	**/
	KvlcStatus reset_status_dlc_mismatch();

	/**
	* Return the message ids that had mismatching dlc
	**/
	KvlcStatus get_list_dlc_mismatch(uint32_t* MsgIds, uint32_t* MsgDlc, uint32_t* MsgOccurance, uint32_t* length);

    /**
     * Writer size estimation functions, get to know whether writer supports it
     * and read it if the writer does
     */
    KvlcStatus get_writer_size_estimate(long long *size_estimate);
    KvlcStatus get_last_timestamp(time_uint64 *timestamp);

    /**
    * Enumerator for Writers, order is not guaranteed between initializations,
    * returns 0 if no more writer is to be found
    **/
    static int getFirstWriterFormat();
    static int getNextWriterFormat(int currentFormat);

    /**
    * Get Writer information,
    *   uses sprintf on the char *str, returns the length
    *   features is built from OR:ed PROPERTY_*
    **/
    static int getWriterName(int format, char *str);
    static int getWriterExtension(int format, char *str);
    static int getWriterDescription(int format, char *str);
    static int isWriterPropertySupported(int format, unsigned int property, bool *supported);
    static int getWriterPropertyDefault(int format, unsigned int property, void *buf, size_t len);

    /**
    * Enumerator for Readers, order is not guaranteed between initializations,
    * returns 0 if no more reader is to be found
    **/
    static int getFirstReaderFormat();
    static int getNextReaderFormat(int currentFormat);

    /**
    * Get Reader information,
    *   uses sprintf on the char *str, returns the length
    *   features is built from OR:ed PROPERTY_*
    **/
    static int getReaderName(int format, char *str);
    static int getReaderExtension(int format, char *str);
    static int getReaderDescription(int format, char *str);
    static int isReaderPropertySupported(int format, unsigned int property, bool *supported);
    static int getReaderPropertyDefault(int format, unsigned int property, void *buf, size_t len);

    /**
    * Get current output filename
    **/
    KvlcStatus get_output_filename (char *filename, size_t len);

    /**
    * Returns true if the output file has changed since the previous call
    * to get_output_filename_status().
    **/
    KvlcStatus get_output_filename_status (bool *updated);

    /**
    * Attach a file to the output file. Returns kvlcERR_FILE_ERROR if the
    * file can't be opened. Returns kvlcERR_NOT_IMPLEMENTED if the selected
    * converter doesn't support attachments.
    **/
    KvlcStatus attach_file (const char *filename);


    /**
    * Set and get a property value. Returns
      kvlcERR_NULL_POINTER if buf and/or len is zero.
      kvlcERR_NOT_IMPLEMENTED if the property doesn't exist.
      KvLogTypeMismatch if the len doesn't match the expected type length.
      kvlcERR_PARAM if the contents of buf can't used as property.
    **/
    KvlcStatus set_property_value (unsigned int property, void *buf, size_t len);
    KvlcStatus get_property_value (unsigned int property, void *buf, size_t len);
};


#endif /* KVACONVERTER_H_ */
