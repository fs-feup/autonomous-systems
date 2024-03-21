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

#ifndef KVALOGWRITER_H_
#define KVALOGWRITER_H_

#include "KvaConverterMisc.h"
#include <time.h>

#include "kvaDbLib.h"
#include "KvaProperties.h"
#include <vector>
#include <string>

#define MAX_ELEM_LEN 320


int MuxCallbackValueChecker(KvaDbSignalHnd mux_signal, unsigned char *event_data, unsigned int len);

class KvaLogWriter {
  private:
    char file_write_buffer[65536];

    bool fake_wallclock_time;
    time_t faked_wallclock_time;

    int mFileNo;
    std::string mFilename;
    std::string mCurrentFilename;
    bool mFirstEvent;
    time_int64 mFirstTime;
    bool mNewFileCreated;

  protected:
    std::vector <KvaDbHnd> databases;
    std::vector <unsigned int> databases_chanmask;
    std::vector <unsigned int> databases_is_file;
    bool mSplitPending;


    FILE *outfile;
    int64_t file_size;
    int64_t file_position;
    time_uint64 start_of_logging; // nanos since 1970, time of first event with rtc-info
    time_uint64 last_clock_event;
    time_uint64 first_timestamp;
    KvlcStatus write_file(void *buf, size_t length);
    bool overrun_occurred;
    bool data_truncation_occurred;
    bool dlc_mismatch_occurred;
    bool isOpened;
    KvlcStatus mapDbError(KvaDbStatus stat);

    int printCalendarDate(char *p, time_int64 time64);
    int printCalendarDate(char *p, const struct tm &calendar_date);
    int printDecimalNumber(char *p, int64_t t1, int nofDecimals);
    int printDecimalNumber(char *p, uint64_t t1, uint64_t t2, bool isNegative, int nofDecimals);
    int printDoubleNumber(char *p, double valdbl, int nofDecimals);
    int printEnumString(char *p, const char *str);

    time_uint64 setStartOfLogging(imLogData *ev);
    const char* get_next_filename();
    KvlcStatus open_new_file(void);
    const char * get_filename();
    KvlcStatus file_status();
    virtual KvlcStatus split_file(void);
    bool split_on_time(time_int64 currentTime);
    bool split_on_size(uint64_t currentSize);
    bool split_files() { return (mProperties.mSizeLimit != 0 || mProperties.mTimeLimit != 0);}

    // Internal API

  public:
    KvaLogWriter();
    virtual ~KvaLogWriter();
    KvaProperties mProperties;
    virtual KvlcStatus write_header() { return kvlcERR_NOT_IMPLEMENTED; };
    virtual KvlcStatus write_row(imLogData *logEvent) = 0;
    virtual KvlcStatus write_event(imLogData *logEvent);
    virtual bool isBinary() = 0;
    virtual KvlcStatus open_file();
    virtual KvlcStatus close_file();
    virtual void next_file();

    virtual bool get_property_start_of_measurement();
    virtual time_int64 get_property_offset();
    virtual time_int64 get_property_logfile_creation_date();

    virtual unsigned long get_property_channel_mask();
    virtual bool get_property_hlp_j1939();
    virtual bool get_property_calendar_time_stamps();
    virtual bool get_property_write_header();
    virtual bool get_property_id_in_hex();
    virtual bool get_property_data_in_hex();
    virtual bool get_property_name_mangling();
    virtual bool get_property_fill_blanks();
    virtual bool get_property_show_units();


    virtual int  get_property_time_decimals();
    virtual int  get_property_data_decimals();
    virtual int  get_property_iso_8601_decimals();
    virtual bool get_property_merge_lines();
    virtual int  get_property_resample_column();
    virtual int  get_property_version();
    virtual bool get_property_show_counter();
    virtual bool get_property_crop_pretrigger();
    virtual bool get_property_enum_values();
    virtual long long get_bytes_written() { return -1; };
    virtual unsigned long get_property_size_limit();
    virtual unsigned long get_property_time_limit();
    virtual unsigned int get_property_limit_databytes();

    virtual char get_property_separator_char();
    virtual char get_property_decimal_char();

    virtual bool get_status_overrun() { return overrun_occurred; }
    virtual void reset_status_overrun() { overrun_occurred = false; }
    virtual bool get_status_data_truncated() { return data_truncation_occurred; }
    virtual void reset_status_data_truncated() { data_truncation_occurred = false; }

    virtual bool get_status_dlc_mismatch() { return dlc_mismatch_occurred; }
    virtual void reset_status_dlc_mismatch() { dlc_mismatch_occurred = false; }
    virtual KvlcStatus get_list_dlc_mismatch(uint32_t* MsgIds, uint32_t* MsgDlc, uint32_t* MsgOccurance, uint32_t* length) { (void)MsgIds; (void)MsgDlc; (void)MsgOccurance; *length = 0;  return kvlcOK; };

    virtual KvlcStatus set_property_offset(time_int64 offset);

    virtual KvlcStatus attach_file(const char *filename);

    KvlcStatus add_database_file(const char *filename, unsigned int channel_mask);
    KvlcStatus add_database_handle(KvaDbHnd dh, unsigned int channel_mask, bool is_file = false);

    unsigned long get_J1939_id(unsigned int id);
    unsigned int get_J1939_mask(unsigned int id);
    time_t get_wallclock_time();
    void get_calendar_time(time_t input_time, struct tm *out_calendar_time);
    KvlcStatus set_filename(const char *filename);
    KvlcStatus get_filename(char *filename, size_t len);
    KvlcStatus get_filename_status(bool *updated);

    KvlcStatus set_property_value (unsigned int property, void *buf, size_t len);
    KvlcStatus get_property_value (unsigned int property, void *buf, size_t len);

    KvlcStatus file_existence_status();
};

#include "KvaWriterMaker.h"

#endif /*KVALOGWRITER_H_*/
