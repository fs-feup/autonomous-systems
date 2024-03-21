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

#ifndef KVALOGWRITER_FAMOS_H_
#define KVALOGWRITER_FAMOS_H_

#include "KvaLogWriter_Signal.h"
#include <string.h>
#include "BufQueue.h"

#define FAMOS_OVERRUN_STRING "# Overrun occurred during logging, messages were lost. #"

class KvaLogWriter_Famos : public KvaLogWriter_Signal {
  unsigned long counter;

  FILE *hFile;
  char lineBuffer[512];
  char dataBuffer[256];
  int bufLength;

  struct {
    unsigned int    day;
    unsigned int    month;
    unsigned int    year;
    unsigned int    hh;
    unsigned int    mm;
    unsigned int    ss;
  } measDate;

  // The buffer description index; increased for each new sub channel
  int indCb;

  // Store data in the correct format; time is always a double
  struct SampleType {
    union {
      double       dval;
      float        fval;
      unsigned int uval;
      int          ival;
    } data;
  };

  struct signal_info_type {
    int index;        // Column index
    int dimension;    // Number of samples
    int format;       // FAMOS format number
    int signBits;     // Number of significant bits
    int noBytes;      // Number of bytes
    int noConSamples; // The number of consecutive samples (constant)
    int mask;
    double offset;
    double factor;
    KvaDbSignalHnd signal_handle;
    BQueue *buffered_samples;
  };


  /*
    A group consists of signals and their common time base.
    Sample and hold will be used if a signal is missing from a row
    where at least one other signal from the group is defined.
  */
  struct GroupType {
    int id;
    char *timeName;
    BQueue *buffered_time_samples;
    std::vector<signal_info_type> signal_info;
    std::vector<signal_info_type>::iterator signal_iterator;
  };

  std::vector<GroupType> groups;

  public:
    KvaLogWriter_Famos();
    ~KvaLogWriter_Famos();
    KvlcStatus write_signal_header() { return kvlcOK; };
    KvlcStatus write_signals();
    KvlcStatus open_file();
    KvlcStatus close_file();

    // Forward declarations
    int writeFamosHeader(void);
    int writeFamosChannelHeader(void);
    int writeFamosTimeDescription(int Cb, size_t skip);
    int writeFamosDataDescription(signal_info_type* si, int Cb, size_t skip);
    int writeFamosBufferDescription(GroupType* gp, int CP, size_t size, int offset);
    int writeFamosCalibration(signal_info_type* si, const char *unit);
    int writeFamosParameterName(signal_info_type* si, char *name, char *note);
    int writeFamosDataBuffer(GroupType* gp);

    int findGroup(int signalindex);
    size_t createTimeName(GroupType *gp, char *fullname);

    bool isBinary() { return false; }

};
#endif /*KVALOGWRITER_FAMOS_H_*/
