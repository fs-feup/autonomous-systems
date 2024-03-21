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
#include "KvaLogWriter_DiademSignal.h"
#include "TimeConv.h"
#include "kvdebug.h"
#include "os_util.h"
#include <ctime>


//==============================================================================
class DiademInfo {
public:
  int64_t    last_timestamp;
  unsigned int ttl;

  DiademInfo () {
    last_timestamp = 0;
    ttl = 0;
  }

  ~DiademInfo () {
  }
};

//==============================================================================
int KvaLogWriter_DiademSignal::getChannelCount (void)
{
  return sr->getSize();
}

//==============================================================================
KvaLogWriter_DiademSignal::KvaLogWriter_DiademSignal()
{
  PRINTF(("KvaLogWriter_DiademSignal::KvaLogWriter_DiademSignal()\n"));
  currentSize = 0;
}

//==============================================================================
KvaLogWriter_DiademSignal::~KvaLogWriter_DiademSignal()
{
  PRINTF(("KvaLogWriter_DiademSignal::~KvaLogWriter_DiademSignal()\n"));
  if (info) delete [] info;
  info = NULL;
}

//==============================================================================
KvlcStatus KvaLogWriter_DiademSignal::write_signal_header()
{
  char *p = outbuffer;
  time_t aclock;
  struct tm newtime;

  KvlcStatus kvstatus = kvlcOK;
  outbuffer[0] = '\0';

  if (!isOpened) {
    PRINTF(("KvaLogWriter_DiademSignal::write_header, file is not opened\n"));
    return kvlcERR_FILE_ERROR;
  }

  unsigned int timestep = get_property_sample_and_hold_timestep();
  if (timestep) {
    property_time_implicit = true;
  } else {
    property_time_implicit = false;
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

    // We finally know when the logging started
    aclock = (time_t) start_of_logging / ONE_BILLION;
    switch(mProperties.mTimezone) {
      case mProperties.KVLC_TZ_UTC:
        GMTIME(&aclock, &newtime);
        break;
      default:
        LOCALTIME(&aclock, &newtime);
        break;
    }

  } else {
    headerWritten = true;
    aclock = get_wallclock_time();
    get_calendar_time(aclock, &newtime);
  }

  os_rewind(outfile);

  PRINTF(("KvaLogWriter_DiademSignal::write_signal_header()\n"));
  // write DIAdem global header
  p += sprintf (p, "DIAEXTENDED  {@:ENGLISH\n");
  p += sprintf (p, "\n");
  p += sprintf (p, "#BEGINGLOBALHEADER\n");
  p += sprintf (p, "  1,WINDOWS\n");
  p += sprintf (p, "  2,{@R:800 {@V:8.00.981 {@F:4.00\n");
  p += sprintf (p, "101,---\n");
  p += sprintf (p, "106,\n");
  p += sprintf (p, "102,\n");
  p += sprintf (p, "103,---\n");
  p += sprintf (p, "104,%02d.%02d.%04d\n", newtime.tm_mday, newtime.tm_mon+1, newtime.tm_year + 1900);
  p += sprintf (p, "105,%02d:%02d:%02d\n", newtime.tm_hour, newtime.tm_min, newtime.tm_sec);
  p += sprintf (p, "110,#dd.mm.yyyy hh:nn:ss\n");
  p += sprintf (p, "111, %lf\n", property_novalue_value);
  p += sprintf (p, "112,High -> Low\n");
  p += sprintf (p, "#ENDGLOBALHEADER\n");
  p += sprintf (p, "\n");

  kvstatus = write_file(outbuffer, p - outbuffer);

  if (info == NULL && getChannelCount() > 0) {
    info = new DiademInfo [getChannelCount()];
  } else {
    PRINTF(("KvaLogWriter_DiademSignal::write_header, no signals selected\n"));
    return kvlcERR_FILE_ERROR;
  }

  if (datafile == NULL) {
    std::string filename = get_filename();
    std::string key = ".dat";
    std::size_t found = filename.rfind(key);
    if (found!=std::string::npos) {
      filename.replace (found,key.length(),".d01");
    } else {
      PRINTF(("Could not create new filename from '%s'\n", filename.c_str()));
      return kvlcERR_FILE_ERROR;
    }
    datafile = utf_fopen(filename.c_str(), "wb");
    if (!datafile) {
      PRINTF(("Could not open file '%s'\n", filename.c_str()));
      return kvlcERR_FILE_ERROR;
    }

    std::string drive, dir, base, ext;
    if (os_splitpath(filename, drive, dir, base, ext)) {
      PRINTF(("Could not remove path from file '%s'\n", filename.c_str()));
      outfile_name_data_without_path = base + ext;
    } else {
      return kvlcERR_INTERNAL_ERROR;
    }
  }

  return kvstatus;
}

//==============================================================================
KvlcStatus KvaLogWriter_DiademSignal::write_signals() {
  unsigned int timestamp = (unsigned int) (sr->getTimeStamp() / 1000000); // getCurrentTimeMs ();

  if (!property_time_implicit) {
    double ts_float = (float) timestamp / 1000.0;
    fwrite (&ts_float, sizeof (ts_float), 1, datafile);
    filesize += sizeof (&ts_float);
  }

  int channel_count = getChannelCount ();
  for (int i = 0; i < channel_count; ++i) {
    double signal_value = property_novalue_value;
    if (sr->isSignalDefined(i)) signal_value = sr->getSignalValue (i);
    fwrite (&signal_value, sizeof (signal_value), 1, datafile);
  }

  time_values++;

  return kvlcOK;
} // MeasureDataFileDiadem::writeRow


//==============================================================================
KvlcStatus KvaLogWriter_DiademSignal::write_signal_trailer ()
{
  char *p = outbuffer;
  KvlcStatus kvstatus = kvlcOK;
  outbuffer[0] = '\0';

  PRINTF(("KvaLogWriter_DiademSignal::write_signal_trailer()\n"));
  int file_offset = 1;
  if (property_time_implicit) {
    p += sprintf (p, "#BEGINCHANNELHEADER\n");
    p += sprintf (p, "200,Zeit(Def_Takt)\n");
    p += sprintf (p, "201,Fixed timing interval\n");
    p += sprintf (p, "202,s\n");
    p += sprintf (p, "210,IMPLICIT\n");
    p += sprintf (p, "220,%d\n", time_values);
    //p += sprintf (p, "240,%lf\n", ((double) start_of_logging) / 1000.0);
    p += sprintf (p, "240,%lf\n", ((double) (start_of_logging % ONE_BILLION) / 100000.0));
    p += sprintf (p, "241,%lf\n", ((double) get_property_sample_and_hold_timestep()) / 1000000.0);
    p += sprintf (p, "253,increasing\n");
    p += sprintf (p, "260,Numeric\n");
    //fprintf (outfile_header, "261,1\n");
    //fprintf (outfile_header, "262,0\n");
    p += sprintf (p, "#ENDCHANNELHEADER\n");
    p += sprintf (p, "\n");
  }
  else {
    p += sprintf (p, "#BEGINCHANNELHEADER\n");
    p += sprintf (p, "200,Zeit(Def_Takt)\n");
    p += sprintf (p, "201,Timestamps generated from CAN Messages\n");
    p += sprintf (p, "202,s\n");
    p += sprintf (p, "210,EXPLICIT\n");
    p += sprintf (p, "211,%s\n", outfile_name_data_without_path.c_str());
    p += sprintf (p, "213,BLOCK\n");
    p += sprintf (p, "214,REAL64\n");
    p += sprintf (p, "220,%d\n", time_values);
    p += sprintf (p, "221,%d\n", file_offset++);
    p += sprintf (p, "240,0\n");
    p += sprintf (p, "241,1\n");
    p += sprintf (p, "253,increasing\n");
    //sprintf (outfile_header, "260,Time\n");
    p += sprintf (p, "260,Numeric\n");
    p += sprintf (p, "261,1\n");
    p += sprintf (p, "262,0\n");
    p += sprintf (p, "#ENDCHANNELHEADER\n");
    p += sprintf (p, "\n");
  }

  currentSize += p - outbuffer;
  kvstatus = write_file(outbuffer, p - outbuffer);
  if (kvstatus != kvlcOK) return kvstatus;

  int channel_count = getChannelCount ();
  PRINTF(("Found %d channels:", channel_count));
  for (int i = 0; i < channel_count; ++i) {
    p = outbuffer;
    outbuffer[0] = '\0';
    char tmp[512];

    sr->getName(tmp, i);
    std::string short_name = tmp;
    sr->getFullName(tmp, i);
    // Always "file.message.signal", but file is GUID so remove it
    std::string long_name = tmp;
    size_t pos = long_name.find(".");
    if (pos != std::string::npos && pos+1 < long_name.length()) {
      long_name = long_name.substr(pos+1, long_name.length());
    }

    sr->getUnit(tmp, i);
    std::string unit = tmp;

    p += sprintf (p, "#BEGINCHANNELHEADER\n");
    p += sprintf (p, "200,%s\n", long_name.c_str());
    p += sprintf (p, "201,%s\n", short_name.c_str());
    p += sprintf (p, "202,%s\n", unit.c_str());
    p += sprintf (p, "210,EXPLICIT\n");
    p += sprintf (p, "211,%s\n", (const char*) outfile_name_data_without_path.c_str());
    p += sprintf (p, "213,BLOCK\n");
    p += sprintf (p, "214,REAL64\n");
    p += sprintf (p, "220,%d\n", time_values);
    p += sprintf (p, "221,%d\n", file_offset++);
    //sprintf (outfile_header, "221,1\n");
    //sprintf (outfile_header, "240,0\n");
    //sprintf (outfile_header, "241,1\n");
    p += sprintf (p, "240,%lf\n", 0.0 );
    p += sprintf (p, "241,%lf\n", 1.0 );
    p += sprintf (p, "255,1\n");
    p += sprintf (p, "260,Numeric\n");
    p += sprintf (p, "261,1\n");
    p += sprintf (p, "262,0\n");
    p += sprintf (p, "#ENDCHANNELHEADER\n");
    p += sprintf (p, "\n");
    currentSize += p - outbuffer;
    PRINTF((" %d", i));
    kvstatus = write_file(outbuffer, p - outbuffer);
    if (kvstatus != kvlcOK) {
      PRINTF(("ERROR: Could not write to file.\n"));
      return kvstatus;
    }
  }
  PRINTF(("\n"));
  return kvlcOK;
  
} // MeasureDataFileDiadem::writeTrailer

//==============================================================================
// we override close_file so that we can add stuff at the very end of the file.
KvlcStatus KvaLogWriter_DiademSignal::close_file()
{
  PRINTF(("KvaLogWriter_DiademSignal::close_file()\n"));

  if (isOpened && info) {
    fflush(outfile);
    write_signal_trailer();
  }
  if (datafile) {
    fclose(datafile);
    datafile = NULL;
  }
  return KvaLogWriter::close_file();
} // MeasureDataFileRpcIII::writeTrailer




#define NAME        "Diadem Signal"
#define EXTENSION   "dat"
#define DESCRIPTION "Selected signals in DIAdem format"

class KvaWriterMaker_DiademSignal : public KvaWriterMaker
{
  public:
    KvaWriterMaker_DiademSignal() : KvaWriterMaker(KVLC_FILE_FORMAT_DIADEM) {
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
      KvaLogWriter *writer = new KvaLogWriter_DiademSignal();
      writer->mProperties = properties;
      return writer;
    }
}  registerKvaLogWriter_DiademSignal;

