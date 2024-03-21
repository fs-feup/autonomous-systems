/*
**             Copyright 2021 by Kvaser AB, Molndal, Sweden
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
#include "KvaLogWriter_RpcIIISignal.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "os_util.h"
#include <ctime>
#include <iomanip>
#include <sstream> 


#define VALUES_PER_BLOCK        1024
static unsigned char dummy_header_data [128];
const static char * monthName[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

//==============================================================================
class ChannelInfo {
public:
  short int       ivalues [VALUES_PER_BLOCK];
  double          lower_limit,
                  upper_limit,
                  rescale_factor;

  long            file_offset_parameter_lower_limit,
                  file_offset_parameter_upper_limit;
  bool            first_value;
  bool            limit_values;
  double          lower_physical_limit,
                  upper_physical_limit;


public:
  ChannelInfo () {
    first_value = true;
    limit_values = false;
    lower_limit = 0;
    upper_limit = 0;
    rescale_factor = 0;
    file_offset_parameter_lower_limit = 0;
    file_offset_parameter_upper_limit = 0;
    lower_physical_limit = 0;
    upper_physical_limit = 0;
  }

  ~ChannelInfo () {
  }
};

//==============================================================================
int KvaLogWriter_RpcIIISignal::getChannelCount (void)
{
  return sr->getSize();
}

//==============================================================================
void KvaLogWriter_RpcIIISignal::write_blocks ()
{
  if (channel_info && isOpened) {
    for (int channel_index = 0; channel_index < getChannelCount (); ++channel_index) {
      int bi = block_index;
      short int fill_ivalue = 0;
      if (block_index > 0) fill_ivalue = channel_info [channel_index] .ivalues [block_index - 1];
      while (bi < VALUES_PER_BLOCK) {
        channel_info [channel_index] .ivalues [bi] = fill_ivalue;
        bi++;
      }
      write_file(channel_info [channel_index] .ivalues, VALUES_PER_BLOCK * sizeof (int short));
    }
    ++frames;
  }
} // KvaLogWriter_RpcIIISignal::write_blocks

//==============================================================================
unsigned int KvaLogWriter_RpcIIISignal::write_parameter (const char *name, const char* value)
{
  int64_t fpos = 0;
  if (isOpened) {
    fpos = os_ftell (outfile);
    int nlen = (int)(strlen (name) + 1),
        vlen = (int)(strlen (value) + 1);
    if (nlen >= 32) nlen = 31;
    if (vlen >= 96) vlen = 95;
    write_file ((void*)name, nlen * sizeof (char));
    write_file (dummy_header_data, (32 - nlen) * sizeof (char));
    write_file ((void*)value, vlen * sizeof (char));
    write_file (dummy_header_data, (96 - vlen) * sizeof (char));
  }
  return (unsigned int)fpos;
} // KvaLogWriter_RpcIIISignal::write_parameter (const char *name, const char* value)

//==============================================================================
unsigned int KvaLogWriter_RpcIIISignal::write_parameter (const char *name, int value)
{
  char tmp [100];
  sprintf (tmp, "%d", value);
  return write_parameter (name, tmp);
} // KvaLogWriter_RpcIIISignal::write_parameter

//==============================================================================
unsigned int KvaLogWriter_RpcIIISignal::write_parameter (const char *name, double value)
{
  // Microsoft Visual studio differs from QT here
  char tmp [100] = {0};
  sprintf (tmp, "%6.12le", value);
  char *e = strchr(tmp, 'e');
  if (e) {
    e++;
    int expo = atoi(e);
    sprintf(e, "%+04d", expo);
  }
  return write_parameter (name, tmp);
} // KvaLogWriter_RpcIIISignal::write_parameter


//==============================================================================
KvaLogWriter_RpcIIISignal::KvaLogWriter_RpcIIISignal()
{
  PRINTF(("KvaLogWriter_RpcIIISignal::KvaLogWriter_RpcIIISignal()\n"));
  currentSize = 0;
  time_values = 0;
  block_index = 0;
  frames = 0;
  channel_info = NULL;
}

//==============================================================================
KvaLogWriter_RpcIIISignal::~KvaLogWriter_RpcIIISignal()
{
  PRINTF(("KvaLogWriter_RpcIIISignal::~KvaLogWriter_RpcIIISignal()\n"));
}

//==============================================================================
KvlcStatus KvaLogWriter_RpcIIISignal::write_signal_header()
{
  int           num_fixed_parameters = 21,
                num_parameters = 0,
                num_header_blocks = 0,
                param_count;
  double time_interval = property_default_time_interval;
  std::string date_time = "dd-MMM-yyyy hh:mm:ss";
  KvlcStatus kvstatus = kvlcOK;
  unsigned int time_step = get_property_sample_and_hold_timestep();

  if (time_step == 0) {
    PRINTF(("KvaLogWriter_RpcIIISignal::write_header, KVLC_PROPERTY_SAMPLE_AND_HOLD_TIMESTEP is not set\n"));
    return kvlcERR_NO_TIME_REFERENCE;
  }

  // Convert from us to seconds
  time_interval = (double) time_step / 1000000.0;

  if (!isOpened) {
    PRINTF(("KvaLogWriter_RpcIIISignal::write_header, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (headerWritten) {
    // Sample & Hold: Extrapolate a final sample. Note that we most likely have
    // extrapolated a lot of samples before this, so this is only to be sure to
    // include the last signal value.
    if (samplingTimeStep && samplingAbsTime) {
      samplingHighResTime += samplingTimeStep;
      samplingAbsTime     += samplingTimeStep;
      sr->setTimeStamp(samplingHighResTime); // High resolution timer
      sr->setAbsTime(samplingAbsTime);       // ABS time
      kvstatus = write_signals();
      if (kvstatus != kvlcOK) return kvstatus;
    }
    return kvlcOK;
  } else {
    headerWritten = true;
  }

  os_rewind(outfile);

  if (channel_info) delete [] channel_info;
  channel_info = NULL;

  if (getChannelCount() > 0) {
    channel_info = new ChannelInfo [getChannelCount()];
  } else {
    PRINTF(("KvaLogWriter_RpcIIISignal::write_header, no signals selected\n"));
    return kvlcERR_FILE_ERROR;
  }

  num_parameters = num_fixed_parameters + sr->getSize() * 6;
  num_header_blocks = (num_parameters + 3) / 4;


  /*  1 */ write_parameter ("FORMAT", "BINARY");
  /*  2 */ write_parameter ("NUM_HEADER_BLOCKS", num_header_blocks);
  /*  3 */ write_parameter ("NUM_PARAMS", num_parameters);
  /*  4 */ write_parameter ("FILE_TYPE", "TIME_HISTORY");
  /*  5 */ write_parameter ("DATA_TYPE", "SHORT_INTEGER");
  /*  6 */ write_parameter ("TIME_TYPE", "RESPONSE");
  /*  7 */ write_parameter ("OPERATION", "CANCONV");
  /*  8 */ write_parameter ("DELTA_T", time_interval);
  /*  9 */ write_parameter ("DESCRIPTION", "");
  /* 10 */ write_parameter ("OPERATION", "DRIVE");
  /* 11 */ file_offset_date = write_parameter ("DATE", date_time.c_str());
  /* 12 */ write_parameter ("CHANNELS", getChannelCount ());
  /* 13 */ write_parameter ("PTS_PER_FRAME", 1024);
  /* 14 */ write_parameter ("PTS_PER_GROUP", 1024);
  /* 15 */ write_parameter ("HALF_FRAMES", 0);
  /* 16 */ write_parameter ("REPEATS", 0);
  /* 17 */ write_parameter ("BYPASS_FILTER", 0);
  /* 18 */ write_parameter ("AVERAGES", 0);
  /* 19 */ write_parameter ("INT_FULL_SCALE", 32768);
  /* 20 */ file_offset_parameter_frames = write_parameter ("FRAMES", 0); // aktualisieren
  param_count = num_fixed_parameters;

  int channel_count = getChannelCount ();
  for (int channel_index = 0; channel_index  < channel_count; ++channel_index) {

    // Make sure that signal values fit into a short int
    double ll = sr->getLowerLimit (channel_index),
           ul = sr->getUpperLimit (channel_index);
    double range = ul - ll;

    double rescale_factor = sr->getFactor(channel_index);

    if (range > 0) {
      if (ll < 0) ll = -ll;
      if (ul < 0) ul = -ul;
      double ml = ll;
      if (ul > ml) ml = ul;
      rescale_factor = ml / 32766;
      channel_info[channel_index].limit_values = true;
      channel_info[channel_index].lower_physical_limit = sr->getLowerLimit (channel_index);
      channel_info[channel_index].upper_physical_limit = sr->getUpperLimit (channel_index);
    }

    channel_info[channel_index].rescale_factor = rescale_factor;
    char key [64];
    int len ;

    len = sr->getName(outbuffer, channel_index);
    if (len > 63) outbuffer[64] = '\0';
    sprintf (key, "DESC.CHAN_%d", channel_index + 1);
    write_parameter (key, outbuffer);
    ++param_count;

    len = sr->getUnit(outbuffer, channel_index);
    if (len > 63) outbuffer[64] = '\0';
    sprintf (key, "UNITS.CHAN_%d", channel_index + 1);
    write_parameter (key, outbuffer);
    ++param_count;

    sprintf (key, "MAP.CHAN_%d", channel_index + 1);
    write_parameter (key, channel_index + 1);
    ++param_count;

    sprintf (key, "SCALE.CHAN_%d", channel_index + 1);
    write_parameter (key, rescale_factor);
    ++param_count;

    sprintf (key, "LOWER_LIMIT.CHAN_%d", channel_index + 1);
    channel_info [channel_index] .file_offset_parameter_lower_limit = write_parameter (key, 0.0);
    ++param_count;

    sprintf (key, "UPPER_LIMIT.CHAN_%d", channel_index + 1);
    channel_info [channel_index] .file_offset_parameter_upper_limit = write_parameter (key, 0.0);
    ++param_count;

  }

  /* 21 */ write_parameter ("PARENT_1", "c:\\dummy.log");

  while (param_count < (num_header_blocks * 4)) {
    write_parameter ("", "");
    ++param_count;
  }

  return kvlcOK;
} // MeasureDataFileRpcIII::writeHeader

//==============================================================================
KvlcStatus KvaLogWriter_RpcIIISignal::write_signals() {
  if (firstFrame) {
    first_message = sr->getTimeStamp();
    firstFrame = false;
  }

  if (channel_info) {
    if (property_all_channel_must_be_valid) {
      for (int channel_index = 0; channel_index < getChannelCount (); ++channel_index) {
        if (!sr->isSignalDefined(channel_index)) {
          goto leave;
        }
      }
    }

    for (int channel_index = 0; channel_index < getChannelCount (); ++channel_index) {
      double rescale_factor = channel_info [channel_index] .rescale_factor;
      double signal_value = property_default_value;

      if (sr->isSignalDefined(channel_index)) signal_value = sr->getSignalValue (channel_index);

      channel_info [channel_index] .ivalues [block_index] = (short int) (signal_value / rescale_factor);

      if (channel_info [channel_index] .first_value) {
        channel_info [channel_index] .first_value = false;
        channel_info [channel_index] .lower_limit = signal_value;
        channel_info [channel_index] .upper_limit = signal_value;
      }
      else {
        if (signal_value > channel_info [channel_index] .upper_limit) {
          channel_info [channel_index] .upper_limit = signal_value;
        }
        if (signal_value < channel_info [channel_index] .lower_limit) {
          channel_info [channel_index] .lower_limit = signal_value;
        }
      }
    }
    time_values++;
    block_index++;
    if (block_index >= VALUES_PER_BLOCK) {
      write_blocks ();
      block_index = 0;
    }
  }

leave:
  return kvlcOK;
} // MeasureDataFileRpcIII::writeRow

//==============================================================================
// we override close_file so that we can add stuff at the very end of the file.
KvlcStatus KvaLogWriter_RpcIIISignal::close_file()
{
  if (block_index > 0) {
    write_blocks ();
    block_index = 0;
  }

  if (isOpened && channel_info) {
    fflush(outfile);
    int64_t fpos = os_ftell (outfile);
    
    // The number of frames written
    os_fseek (outfile, file_offset_parameter_frames, SEEK_SET);
    write_parameter ("FRAMES", frames);
    
    // We finally know when the logging started
    time_t sec_time = (time_t) (start_of_logging + first_message)/ ONE_BILLION;
    struct tm tmp_tm;
    std::string my_time;
    switch(mProperties.mTimezone) {
      case mProperties.KVLC_TZ_UTC:
        GMTIME(&sec_time, &tmp_tm);
        break;
      default:
        LOCALTIME(&sec_time, &tmp_tm);
        break;
    }
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << tmp_tm.tm_mday << "-"
        << monthName[tmp_tm.tm_mon] << "-"
        << tmp_tm.tm_year + 1900
        << " "
        << std::setw(2) << tmp_tm.tm_hour << std::setw(2)
        << tmp_tm.tm_min << std::setw(2) << tmp_tm.tm_sec;
    os_fseek (outfile, file_offset_date, SEEK_SET);
    write_parameter ("DATE", oss.str().c_str());

    for (int channel_index = 0; channel_index < getChannelCount (); ++channel_index) {
      char key [64];

      sprintf (key, "LOWER_LIMIT.CHAN_%d", channel_index + 1);
      os_fseek (outfile, channel_info [channel_index] .file_offset_parameter_lower_limit, SEEK_SET);
      write_parameter (key, channel_info [channel_index] .lower_limit);

      sprintf (key, "UPPER_LIMIT.CHAN_%d", channel_index + 1);
      os_fseek (outfile, channel_info [channel_index] .file_offset_parameter_upper_limit, SEEK_SET);
      write_parameter (key, channel_info [channel_index] .upper_limit);
    }
    os_fseek (outfile, fpos, SEEK_SET);
  }

  headerWritten = false;
  return KvaLogWriter::close_file();
} 


#define NAME        "MTS RPC III Signal"
#define EXTENSION   "rpc"
#define DESCRIPTION "Selected signals in MTS RPC III format"

class KvaWriterMaker_RpcIIISignal : public KvaWriterMaker
{
  public:
    KvaWriterMaker_RpcIIISignal() : KvaWriterMaker(KVLC_FILE_FORMAT_RPCIII) {
      propertyList[KVLC_PROPERTY_START_OF_MEASUREMENT] = true;
      propertyList[KVLC_PROPERTY_FIRST_TRIGGER] = true;
      propertyList[KVLC_PROPERTY_CHANNEL_MASK] = true;
      propertyList[KVLC_PROPERTY_HLP_J1939] = true;
      propertyList[KVLC_PROPERTY_SIGNAL_BASED] = true;
      propertyList[KVLC_PROPERTY_SHOW_SIGNAL_SELECT] = true;
      propertyList[KVLC_PROPERTY_CROP_PRETRIGGER] = true;
      propertyList[KVLC_PROPERTY_OVERWRITE] = true;
      propertyList[KVLC_PROPERTY_TIMEZONE] = true;
      propertyList[KVLC_PROPERTY_SAMPLE_AND_HOLD_TIMESTEP] = true;
    }
    int getName(char *str) { return sprintf(str, "%s", NAME); }
    int getExtension(char *str) { return sprintf(str, "%s", EXTENSION); }
    int getDescription(char *str) { return sprintf(str, "%s", DESCRIPTION); }

  private:
    KvaLogWriter *OnCreateWriter() {
      KvaLogWriter *writer = new KvaLogWriter_RpcIIISignal();
      writer->mProperties = properties;
      return writer;
    }
}  registerKvaLogWriter_RpcIIISignal;

