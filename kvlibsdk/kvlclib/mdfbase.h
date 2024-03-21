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
**  Classes for writing Bosch MDF (Vector), supports ver 3.20, 3.00 and 2.03
** -----------------------------------------------------------------------------
*/

#ifndef MDFBASE_H
#define MDFBASE_H

#include <stdlib.h>
#include <stdio.h>

#include "MdfTypes.h"
#include "BufQueue.h"
#include "MuxChecker.h"


// -----------------------------------------------------------------------------
// String constants
// -----------------------------------------------------------------------------
#define MDF_FILE_ID         "MDF     "
#define MDF_PROGRAM_ID      "Kvaser  "
#define MDF_VERSION_STRING_320  "3.20    "
#define MDF_VERSION_STRING_203  "2.03    "
#define MDF_VERSION_STRING_400  "4.00    "
#define MDF_VERSION_STRING_410  "4.10    "


// Own definitions
typedef uint64_t  KVMDF_TIMESTAMP;
#define MDF_FALSE (MDF_BOOL)0
#define MDF_TRUE  (MDF_BOOL)~MDF_FALSE
#define MDF_MAX_DLC 64

// -----------------------------------------------------------------------------
// Signal data types used by converter; corresponds to MDF 2.x and 3.x
// -----------------------------------------------------------------------------
// Default byte order from IDBLOCK
#define MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT     0
#define MDF_SIGNAL_DATA_TYPE_SIGNED_INT       1 // two complements
#define MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT       2 // 4 bytes
#define MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE      3 // 8-10 bytes
// VAX floating point (4-6) obsolete
// The following types are only supported in >= MDF 2.15
#define MDF_SIGNAL_DATA_TYPE_STRING           7 // NULL terminated
#define MDF_SIGNAL_DATA_TYPE_BYTE_ARRAY       8 // max 8191 bytes, constant record length
// The following types are only supported in >= MDF 3.10
// Big Endian
#define MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_BE  9
#define MDF_SIGNAL_DATA_TYPE_SIGNED_INT_BE   10 // two complements
#define MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_BE   11 // 4 bytes
#define MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_BE  12 // 8-10 bytes
// Little Endian
#define MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE 13
#define MDF_SIGNAL_DATA_TYPE_SIGNED_INT_LE   14 // two complements
#define MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_LE   15 // 4 bytes
#define MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_LE  16 // 8-10 bytes

// -----------------------------------------------------------------------------
// Conversion types used by converter; corresponds to MDF 2.x and 3.x
// -----------------------------------------------------------------------------
// The Channel Conversion Block CCBLOCK
#define MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR 0
// No support for formulas 1-65534
#define MDF_CONVERSION_TYPE_ONE_TO_ONE        65535


// -----------------------------------------------------------------------------
// Base class for all block nodes
// -----------------------------------------------------------------------------
class NodeBase {
private:
    size_t        current_file_pos;
    int           version;
    unsigned long last_size;
    bool          size_dirty;
public:
    NodeBase() { version = -1; size_dirty = true; current_file_pos= 0;};
    ~NodeBase() {};
    void setVersion(int ver) { if (version == -1) { version = ver; } }
    int getVersion() { return version; }
    virtual int write(FILE *mdfFile) = 0;
    size_t  getCurrentFilePos() { return current_file_pos; }
    void setCurrentFilePos(size_t new_pos) { current_file_pos = new_pos; }
    virtual unsigned long size() = 0;
    bool isSizeDirty() { return size_dirty; }
    void setSizeDirty(bool dirty = true) { size_dirty = dirty; }
    void setSize(unsigned long s) { last_size = s; setSizeDirty(false);}
    unsigned long getLastSize() { return last_size; }
};

// -----------------------------------------------------------------------------
// Interface class used by converter
// -----------------------------------------------------------------------------
class MdfBase {
public:
  virtual int create(const char *filename) = 0;
  virtual void create(FILE *fh) = 0;
  virtual int write_header() = 0;
  virtual int write_data() = 0;


  virtual unsigned long maxSize() = 0;

  virtual MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT8 dlc = 8) = 0;
  virtual MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc = 8) = 0;
  virtual MdfStatus new_sig(MDF_UINT32 canId,
              char *longname,
              char *shortname,
              char *unit,
              MDF_UINT16 StartOffset,
              MDF_UINT16 NumBits,
              MDF_UINT16 SigDataType,
              MDF_UINT16 ChannelConversionType = MDF_CONVERSION_TYPE_ONE_TO_ONE,
              MDF_REAL Factor = 1.0,
              MDF_REAL Offset = 0.0) = 0;

  virtual int setStartOfRecording(
    unsigned int year,
    unsigned char month,
    unsigned char day,
    unsigned char hour,
    unsigned char minute,
    unsigned char second
  ) = 0;
  virtual int addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data) = 0;

  virtual unsigned long size() = 0;

  virtual int close() = 0;

  virtual int initAndSetVersion(int version) = 0;

  virtual ~MdfBase() {};
};

// -----------------------------------------------------------------------------
// MDF v2.x and 3.x conversion
// -----------------------------------------------------------------------------
namespace Mdf3Nodes {
class idNode;
class hdNode;
}

// -----------------------------------------------------------------------------
// MDF v2.x and 3.x conversion
// -----------------------------------------------------------------------------
class Mdf3 : public MdfBase {

  FILE *mdfFile;

  Mdf3Nodes::idNode *id;
  Mdf3Nodes::hdNode *hd;

  int version;

  MDF_UINT16 curByteOrder;

public:
  int create(const char *filename);
  void create(FILE *fh);
  int write_header();
  int write_data();

  unsigned long maxSize();
  static unsigned long maxSize(int ver);

  MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT8 dlc = 8);
  MdfStatus new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc = 8);
  MdfStatus new_sig(MDF_UINT32 canId,
              char *longname,
              char *shortname,
              char *unit,
              MDF_UINT16 StartOffset,
              MDF_UINT16 NumBits,
              MDF_UINT16 SigDataType,
              MDF_UINT16 ChannelConversionType = MDF_CONVERSION_TYPE_ONE_TO_ONE,
              MDF_REAL Factor = 1.0,
              MDF_REAL Offset = 0.0);

  int setStartOfRecording(
    unsigned int year,
    unsigned char month,
    unsigned char day,
    unsigned char hour,
    unsigned char minute,
    unsigned char second
  );
  int addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data);

  unsigned long size();

  int close();

  int initAndSetVersion(int version);
  Mdf3();
  ~Mdf3();
};

#endif
