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
**  Classes for writing ASAM MDF v4.1 for Vector CANalyser
** ---------------------------------------------------------------------------
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stack>
#include <ctime>
#include <map>
#include "os_util.h"
#include "TimeConv.h"
#include "md5sum.h"
#include "KvaConverterMisc.h"
#include "common_defs.h"
#include "mdf4.h"
#include "zlib.h"

#if defined(DEBUG) && defined(CRTMEMCHECK)
  #include <windows.h>
  #define CRTDBG_MAP_ALLOC
  #include <crtdbg.h>
#endif


#if defined(DEBUG)
  #include "kvdebug.h"
  #define FWRITE(x) fwrite x
  #define FTELL(x) os_ftell(x)
  #define FSEEK(x) os_fseek x
#else
  #define PRINTF(x)
  #define FTELL(x) os_ftell(x)
  #define FWRITE(x) fwrite x
  #define FSEEK(x) os_fseek x
#endif

#define MDF_VERSION_4x 4

static int compression_level = Z_DEFAULT_COMPRESSION; // <--> KVLC_PROPERTY_COMPRESSION_LEVEL

Mdf4Options NodeBase::options;

static int CAN_DATA_FRAME[MAX_CHANNELS];
static int CAN_REMOTE_FRAME[MAX_CHANNELS];
static int CAN_ERROR_FRAME[MAX_CHANNELS];
static int CAN_OVERLOAD_FRAME[MAX_CHANNELS];

static int channelTypeExists(int channel, int type)
{
  int ch = channel % MAX_CHANNELS;
  switch(type) {
    case MDF_ERROR_FRAME_TYPE: return CAN_ERROR_FRAME[ch];
    case MDF_CAN_FRAME_TYPE: return CAN_DATA_FRAME[ch];
    case MDF_REMOTE_FRAME_TYPE: return CAN_REMOTE_FRAME[ch];

  }
  PRINTF(("Error: channelTypeExists failed for type %d", type));
  return 1;
}

static void channelTypeCreated(int channel, int type)
{
  int ch = channel % MAX_CHANNELS;
  switch(type) {
    case MDF_ERROR_FRAME_TYPE:
      CAN_ERROR_FRAME[ch] = 1;
      break;

    case MDF_CAN_FRAME_TYPE:
      CAN_DATA_FRAME[ch] = 1;
      break;

      case MDF_REMOTE_FRAME_TYPE:
      CAN_REMOTE_FRAME[ch] = 1;
      break;

    default:
    PRINTF(("Error: channelTypeCreated failed for type %d", type));
  }
}

static void clearAllChannelTypes(void) {
  memset(CAN_DATA_FRAME, 0,sizeof(CAN_DATA_FRAME));
  memset(CAN_REMOTE_FRAME, 0, sizeof(CAN_REMOTE_FRAME));
  memset(CAN_ERROR_FRAME, 0, sizeof(CAN_ERROR_FRAME));
  memset(CAN_OVERLOAD_FRAME, 0, sizeof(CAN_OVERLOAD_FRAME));
}


static MDF_UINT64 align8(MDF_UINT64 length)
{
  if (length % 8) {
    return (length / 8 + 1) * 8;
  }
  return length;
}
/*
static size_t align8(MDF_UINT64 length)
{
  return align8((size_t) length);
}
*/
using namespace Mdf4Nodes;

// String defines for channel nodes
static ch ChStrings;

/*******************************************************************************
* Mdf4
*******************************************************************************/
Mdf4::Mdf4()
{
  PRINTF(("Mdf4::Mdf4()"));

  mdfFile = NULL;

  id = NULL;
  hd = NULL;

  version = 0;
  clearAllChannelTypes();
}

Mdf4::~Mdf4()
{
  PRINTF(("Mdf4::~Mdf4()"));
  close();

  PRINTF(("Mdf4::size = %lu\n", size()));

  if (id) {
    delete id;
  }

  if (hd) {
    delete hd;
  }
}

int Mdf4::create(const char *filename)
{
  //mdfFile = fopen(filename, "wb");
  mdfFile = utf_fopen(filename, "wb");
  if (!mdfFile) {
    PRINTF(("ERROR: Unable to open '%s'\n", filename));
    return MDF_ERROR_FILE;
  }

  return MDF_OK;
}

void Mdf4::create(FILE *fh)
{
  mdfFile = fh;
}

int Mdf4::write_header()
{
  PRINTF(("Mdf4::write_header"));
  if (mdfFile && id && hd) {
    FSEEK((mdfFile, 0, SEEK_SET));
    id->write(mdfFile);
    hd->write(mdfFile);
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;
}

int Mdf4::write_data() {
  if (hd) {
    return hd->write_data(mdfFile);
  }
  return MDF_ERROR_MEMORY;
}

int Mdf4::close()
{
  PRINTF(("Mdf4::close()"));
  if (mdfFile) {
    fflush(mdfFile);
    fclose(mdfFile);
    mdfFile = NULL;
    return MDF_OK;
  }
  return MDF_ERROR_FILE;
}

uint64_t Mdf4::size()
{
  PRINTF(("Mdf4::size()"));
  if (hd && id) {
    return hd->size() + id->size();
  }
  return 0;
}

int Mdf4::initAndSetVersion(int version)
{
  PRINTF(("Mdf4::initAndSetVersion(%d)", version));
  this->version = version;
  id = new Mdf4Nodes::idNode(version);
  hd = new Mdf4Nodes::hdNode(version);
  if (id && hd) {
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;
}


int Mdf4::setStartOfRecording(
  unsigned int year,
  unsigned char month,
  unsigned char day,
  unsigned char hour,
  unsigned char minute,
  unsigned char second
)
{
  if (hd) {
    return hd->setStartOfRecording(year, month, day, hour, minute, second);
  }
  return MDF_ERROR_MEMORY;
}

int Mdf4::addFrame(int channel, int type, KVMDF_TIMESTAMP ts, void *data)
{

  if (!channelTypeExists(channel, type) && hd) {
    PRINTF(("Create channel for ch: %d and type: %d", channel, type));
    hd->createMdf4Channel(channel, type);
  }

  PRINTF(("Mdf4::addFrame(%d, %d)", channel, type));
  if (hd) {
    return hd->addFrame(channel, type, ts, (MDF_UINT8*) data);
  }
  return MDF_ERROR_MEMORY;
}

int Mdf4::attach(const char *filename)
{
  FILE *h;
  //h = fopen(filename , "rb" );
  h = utf_fopen(filename, "rb");
  if (h == NULL) {
    PRINTF(("Could not open attachment '%s'", filename));
    return MDF_ERROR_FILE;
  }
  fclose(h);

  if (hd) {
    hd->attach(filename);
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;

}

// Use variable length signal data (VLSD) channel for data bytes
MdfStatus Mdf4::useVariableRecordSize(bool on)
{
  if (hd) {
    hd->options.useVariableRecordSize = on;
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;
}

// Use fixed length (64bytes) signal data channel for data bytes
MdfStatus Mdf4::useFixedRecordSize64(bool on)
{
  if (hd) {
    hd->options.useFixedRecordSize64 = on;
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;
}

MdfStatus Mdf4::useDataListNodes(bool on)
{
  if (hd) {
    hd->options.useDataListNodes = on;
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;
}

MdfStatus Mdf4::useZippedData(bool on)
{
  if (hd) {
    hd->options.useZippedData = on;
    if (on) {
      // Ensure that data lists are turned on
      hd->options.useDataListNodes = true;
    }
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;
}


MdfStatus Mdf4::new_dg(MDF_UINT32 canId, MDF_UINT8 dlc)
{
  return Mdf4::new_dg(canId, CAN_ID_MASK_ALL_BITS, MuxChecker(0,-1), dlc);
}

MdfStatus Mdf4::new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc, const std::string& msgname, int channel)
{
  if (hd) {
    return hd->new_dg(canId, canMask, mux, dlc, msgname, channel);
  }
  return MDF_ERROR_MEMORY;
}

MdfStatus Mdf4::new_sig(MDF_UINT32 canId,
            char *longname,
            char *shortname,
            char *unit,
            MDF_UINT16 StartOffset,
            MDF_UINT16 NumBits,
            MDF_UINT16 SigDataType,
            MDF_UINT16 ChannelConversionType,
            MDF_REAL Factor,
            MDF_REAL Offset)
{
  if (hd) {
    return hd->new_sig( canId, longname, shortname, unit, StartOffset, NumBits,
                        SigDataType, ChannelConversionType, Factor, Offset);
  }
  return MDF_ERROR_MEMORY;
}

void Mdf4::setCompressionLevel(int level){
  compression_level = level;
  if (compression_level==0){
    useZippedData(false);
  } else {
    useZippedData(true);
  }
}


// CAN frame channels
cnNode * newChannel(cnNode **next, const int type, ChannelType ch_type,
                    MDF_UINT8 cn_bit_offset, MDF_UINT32  cn_byte_offset,
                    MDF_UINT32 cn_bit_count)
{

  std::string long_name;
  switch (type) {
  case MDF_CAN_FRAME_TYPE:
    long_name = CAN_FRAME_NAME;
    break;

  case MDF_ERROR_FRAME_TYPE:
    long_name = CAN_ERROR_NAME;
    break;

  case MDF_REMOTE_FRAME_TYPE:
    long_name = CAN_REMOTE_NAME;
    break;
  }
  long_name = long_name + MDF_PATH_SEP + ChStrings.getName(ch_type);;

  cnNode * cn = new cnNode(MDF_VERSION_4x);
  *next = cn;

  cn->tx_long_signal_name = new txNode(MDF_VERSION_4x, long_name.c_str());
  cn->md_comment = new mdNode(MDF_VERSION_4x, mdNode::CN_MD_COMMENT, ChStrings.getCommentC(ch_type), ChStrings.getNameC(ch_type));
  cn->cn.cn_bit_offset = cn_bit_offset;
  cn->cn.cn_byte_offset = cn_byte_offset;
  cn->cn.cn_bit_count = cn_bit_count;
  cn->cn.cn_flags = MDF_CN_FLAGS_DVAL | MDF_CN_FLAGS_BUS_EVENT;
  return cn;
}

int hdNode::createMdf4Channel(int channel, int type, MDF_UINT32 id, const std::string& msgname)
{
  char tmp[128] = {0};
  cnNode *cn_root = NULL;
  cnNode *cn_data = NULL;
  //  sdNode *cn_data_sd = NULL;
  cnNode *cn = NULL;
  //cnNode *cn_tmp = NULL;
  std::string long_name;
  int version = getVersion();

  new_vector_dg(channel, type);
  // Create the composite node
  new_vector_can_frame(channel, type); // Sets cur_dg

  cur_dg->cg->si_acq_source = new siNode(version, channel);

  switch (type) {
  case MDF_CAN_FRAME_TYPE:
    if (id ==  CAN_ID_NOT_USED) {
      cur_dg->cg->si_acq_source->tx_path = new txNode(version, CAN_FRAME_NAME.c_str());
      cur_dg->cg->cg.cg_flags = MDF_CG_FLAGS_BUS_EVENT | MDF_CG_FLAGS_PLAIN_BUS_EVENT;
    } else {
      char tmp[128] = {0};
      sprintf(tmp, "ID=%d", id);
      std::string str = CAN_FRAME_NAME + MDF_PATH_SEP + std::string(tmp);
      cur_dg->cg->si_acq_source->tx_path = new txNode(version, str.c_str());
      cur_dg->cg->cg.cg_flags = MDF_CG_FLAGS_BUS_EVENT;

      // channel groups are sorted on CAN id so groupname := msgname
      int ver = cur_dg->cg->tx_acq_name->getVersion();
      delete cur_dg->cg->tx_acq_name;
      cur_dg->cg->tx_acq_name = new txNode(ver, msgname.c_str());
    }
    break;

  case MDF_ERROR_FRAME_TYPE:
    cur_dg->cg->si_acq_source->tx_path = new txNode(version, CAN_ERROR_NAME.c_str());
    cur_dg->cg->cg.cg_flags = MDF_CG_FLAGS_BUS_EVENT | MDF_CG_FLAGS_PLAIN_BUS_EVENT;
    break;

  case MDF_REMOTE_FRAME_TYPE:
    cur_dg->cg->si_acq_source->tx_path = new txNode(version, CAN_REMOTE_NAME.c_str());
    cur_dg->cg->cg.cg_flags = MDF_CG_FLAGS_BUS_EVENT | MDF_CG_FLAGS_PLAIN_BUS_EVENT;
    break;
  }
  sprintf(tmp, "%d", channel);
  cur_dg->cg->si_acq_source->md_comment = new mdNode(version, mdNode::SI_MD_COMMENT, tmp);

  cur_dg->cg->cg.cg_path_separator = MDF_CG_PATH_SEPARATOR_DOT;

  cn_root = cur_dg->cg->cn;
  switch (type) {
    case MDF_CAN_FRAME_TYPE:
      cn_root->tx_long_signal_name = new txNode(version, CAN_FRAME_NAME.c_str());
      break;

    case MDF_ERROR_FRAME_TYPE:
      cn_root->tx_long_signal_name = new txNode(version, CAN_ERROR_NAME.c_str());
      break;

    case MDF_REMOTE_FRAME_TYPE:
      cn_root->tx_long_signal_name = new txNode(version, CAN_REMOTE_NAME.c_str());
      break;
  }

  cn_root->cn_si_source = new siNode(version, channel);
  sprintf(tmp, "CAN%d", channel);
  cn_root->cn_si_source->tx_path = new txNode(version, tmp);

  cn_root->cn.cn_data_type = cnNode::CN_DATA_TYPE_BYTE_ARRAY;
  cn_root->cn.cn_byte_offset = sizeof(KVMDF_TIMESTAMP);

  switch(type) {
    case MDF_CAN_FRAME_TYPE:
      if (options.useFixedRecordSize64) {
        cn_root->cn.cn_bit_count = MDF_DATA_FRAME_SIZE_FD * 8;
      } else {
        cn_root->cn.cn_bit_count = MDF_DATA_FRAME_SIZE * 8;
      }
      break;

    case MDF_ERROR_FRAME_TYPE:
      cn_root->cn.cn_bit_count = MDF_DATA_FRAME_SIZE * 8;
      break;

    case MDF_REMOTE_FRAME_TYPE:

      cn_root->cn.cn_bit_count = MDF_DATA_FRAME_SIZE * 8;
      break;
  }

  cn_root->cn.cn_flags = MDF_CN_FLAGS_BUS_EVENT;
  cn_root->cn_composition = new cnNode(version);

  cn = cn_root->cn_composition;
  switch (type) {
  case MDF_CAN_FRAME_TYPE:
    long_name = CAN_FRAME_NAME;
    break;

  case MDF_ERROR_FRAME_TYPE:
    long_name = CAN_ERROR_NAME;
    break;

  case MDF_REMOTE_FRAME_TYPE:
    long_name = CAN_REMOTE_NAME;
    break;
  }
  long_name = long_name + MDF_PATH_SEP + std::string("BusChannel");
  cn->tx_long_signal_name = new txNode(version, long_name.c_str());

  cn->md_comment = new mdNode(version, mdNode::CN_MD_COMMENT, "Logical bus channel number the frame was sent or received.", "BusChannel");
  cn->cc = new ccNode(version, NULL, MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR, channel, 0);

  cn->cn.cn_type = MDF_CN_TYPE_VIRTUAL;
  cn->cn.cn_bit_offset = 0;
  cn->cn.cn_byte_offset = 8;
  cn->cn.cn_bit_count = 0;
  cn->cn.cn_flags = MDF_CN_FLAGS_BUS_EVENT | MDF_CN_FLAGS_DVAL;

  // Channels for CAN Data Frame
  if (MDF_CAN_FRAME_TYPE == type) {
    /* Mandatory members: "ID", "DLC", "DataLength", and "DataBytes". */

    cn = newChannel(&cn->next, type, ID, 0, 8, 32);
    cn = newChannel(&cn->next, type, KVASER_FLAGS, 0, 12, 32);
    cn = newChannel(&cn->next, type, DIR, 7, 12, 1);
    cn = newChannel(&cn->next, type, IDE, 7, 11, 1);
    cn = newChannel(&cn->next, type, EDL, 0, 14, 1);
    cn = newChannel(&cn->next, type, BRS, 1, 14, 1);
    cn = newChannel(&cn->next, type, ESI, 2, 14, 1);
    cn = newChannel(&cn->next, type, DLC, 0, 16, 8);
    cn = newChannel(&cn->next, type, DATA_LENGTH, 0, 17, 8);
    cn_data = cn;

    // DataBytes
    if (options.useFixedRecordSize64) {
      cn = newChannel(&cn->next, type, DATA_BYTES, 0, MDF_FIRST_DATA_BYTE, 64*8);
    } else {
      cn = newChannel(&cn->next, type, DATA_BYTES, 0, MDF_FIRST_DATA_BYTE, 8*8);
    }
    if (options.useVariableRecordSize) {
      cn->cn.cn_type = MDF_CN_TYPE_VARIABLE_LENGTH;
      if (options.useDataListNodes) {
        if (options.useZippedData) {
          cn->cn_data_sd = cur_dg->hl_sd;
        } else {
          cn->cn_data_sd = cur_dg->dl_sd;
        }
      } else {
        cn->cn_data_sd = cur_dg->sd;
      }
    }
    else {
      cn->cn.cn_type = MDF_CN_TYPE_MAXIMUM_LENGTH;
      cn->cn_data = cn_data;
    }
    cn->cn.cn_data_type = cnNode::CN_DATA_TYPE_BYTE_ARRAY;
  }

  // Channels for CAN Remote Frame
  if (MDF_REMOTE_FRAME_TYPE == type) {
    /* Mandatory: ID, DLC */
    cn = newChannel(&cn->next, type, ID, 0, 8, 32);
    cn = newChannel(&cn->next, type, KVASER_FLAGS, 0, 12, 32);
    cn = newChannel(&cn->next, type, DLC, 0, 16, 8);
    cn = newChannel(&cn->next, type, DATA_LENGTH, 0, 17, 8);
    cn_data = cn;
    cn = newChannel(&cn->next, type, DATA_BYTES, 0, MDF_FIRST_DATA_BYTE, 64);
    cn->cn.cn_type = MDF_CN_TYPE_MAXIMUM_LENGTH;
    cn->cn_data = cn_data;
  }

  // Channels for CAN Error Frame
  if (MDF_ERROR_FRAME_TYPE == type) {
    /* Mandatory: None */
    cn = newChannel(&cn->next, type, ID, 0, 8, 32);
    cn = newChannel(&cn->next, type, KVASER_FLAGS, 0, 12, 32);
    cn = newChannel(&cn->next, type, DLC, 0, 16, 8);
    cn = newChannel(&cn->next, type, DATA_LENGTH, 0, 17, 8);
    cn_data = cn;
    cn = newChannel(&cn->next, type, DATA_BYTES, 0, MDF_FIRST_DATA_BYTE, 64);
    cn->cn.cn_type = MDF_CN_TYPE_MAXIMUM_LENGTH;
    cn->cn_data = cn_data;
  }

  // Create the time node
  cn_root->next = new cnNode(version);
  cn = cn_root->next;
  cn->tx_long_signal_name = new txNode(version, "t");
  cn->cn.cn_type = MDF_CN_TYPE_MASTER_CHANNEL;
  cn->cn.cn_data_type = MDF_CN_TYPE_FIXED_LENGTH;
  cn->cn.cn_sync_type = MDF_CN_SYNC_TYPE_TIME;
  cn->cn.cn_bit_count = sizeof(KVMDF_TIMESTAMP) * 8;
  cn->cn.cn_flags = MDF_CN_FLAGS_VALUE_RANGE_OK;
  cn->cc = new ccNode(version, "s", MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR, 0, TIMESTAMP_TO_SECONDS);
  cn->cc->cc.cc_flags = MDF_CC_FLAGS_PHYSICAL_RANGE_OK;
  cur_dg->cn_time = cn;

  channelTypeCreated(channel, type);
  return 0;
}



namespace Mdf4Nodes {
/*******************************************************************************
* fhNode - Node for FHBLOCK
*******************************************************************************/
fhNode::fhNode(int version)
{
  memset(&fh, 0, sizeof(FHBLOCK));
  memcpy(fh.Head.id,MDF_ID_FHBLOCK, 4);
  fh.Head.length = sizeof(FHBLOCK);
  fh.Head.link_count = 2; // Number of links in link section
  fh.fh_fh_next = 0;
  fh.fh_md_comment = 0; // Link to MDBLOCK containing comment about the creation or modification of the MDF file.

  // Set creation time and time zone
  time_t aclock = time(NULL);

  struct tm newtime;
  localtime_r(&aclock, &newtime);
  fh.fh_time_ns = mktime(&newtime) * 1000000000LL;
  fh.fh_tz_offset_min = (MDF_INT16) (newtime.tm_gmtoff / 60);
  if (newtime.tm_isdst > 0) {
    // Turn off day light saving to get fh_tz_offset_min
    newtime.tm_isdst = 0;
    time_t daylight = mktime(&newtime);
    double diff = difftime(daylight, aclock);
    // Daylight saving time (DST) offset in minutes
    fh.fh_dst_offset_min = (MDF_INT16) (diff/60.0);
    fh.fh_tz_offset_min -= fh.fh_dst_offset_min;
  }
  fh.fh_time_flags = 0x2; // Time zone data is valid
  md = new mdNode(version, mdNode::FH_MD_COMMENT, NULL);

}
fhNode::~fhNode()
{
  PRINTF(("~fhNode (%p)\n", this));
  if (md) delete(md);
}

int fhNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));

  if (md) {
    fh.fh_md_comment = md->getCurrentFilePos();
  }

  FWRITE((&fh, 1, sizeof(FHBLOCK), mdfFile));

  if (md) {
    md->write(mdfFile);
  }
  return 0;
}

uint64_t fhNode::size()
{
  uint64_t s = sizeof(FHBLOCK);
  if (md) s += md->size();
  return s;
}


/*******************************************************************************
* txNode - Node for TXBLOCK
*******************************************************************************/
txNode::txNode(int version, const char *str)
{
  setVersion(version);
  size_t len = strlen(str) + 1;

  memset(&tx, 0, sizeof(TXBLOCK));

  memcpy(tx.Head.id,MDF_ID_TXBLOCK, 4);
  len = (size_t) align8(len);
  tx.Head.length = sizeof(BL_HEAD) + len;
  tx.tx_data = new MDF_BYTE[len];
  memset(tx.tx_data, 0, len);
  memcpy(tx.tx_data, str, strlen(str));
}

txNode::~txNode()
{
  PRINTF(("~txNode (%p)\n", this));
  if (tx.tx_data) {
    delete [] tx.tx_data;
  }
}

int txNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));
  if (tx.Head.length) {
    FWRITE((&tx, 1, sizeof(BL_HEAD), mdfFile));
    FWRITE((tx.tx_data, 1, (size_t)tx.Head.length - sizeof(BL_HEAD), mdfFile));
  }
  return 0;
}

uint64_t txNode::size()
{
  return tx.Head.length;
}


/*******************************************************************************
* siNode - Node for SIBLOCK
*******************************************************************************/
siNode::siNode(int version, int /* channel */)
{
  setVersion(version);
  memset(&si, 0, sizeof(SIBLOCK));
  memcpy(si.Head.id, MDF_ID_SIBLOCK, 4);
  si.Head.length = sizeof(SIBLOCK);
  si.Head.link_count = 3;

  si.si_type     = SI_TYPE_BUS;
  si.si_bus_type = SI_BUS_TYPE_CAN;
  tx_name = NULL;
  tx_path = NULL;
  md_comment = NULL;

}

siNode::~siNode()
{
  PRINTF(("~siNode (%p)\n", this));
  if (tx_name) {
    delete tx_name;
  }
  if (tx_path) {
    delete tx_path;
  }
  if (md_comment) {
    delete md_comment;
  }
}

int siNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));
  if (tx_name) {
    si.si_tx_name = tx_name->getCurrentFilePos();
  }
  if (tx_path) {
    si.si_tx_path = tx_path->getCurrentFilePos();
  }
  if (md_comment) {
    si.si_md_comment = md_comment->getCurrentFilePos();
  }

  FWRITE((&si, 1, (size_t)si.Head.length, mdfFile));

  if (tx_path) {
    tx_path->write(mdfFile);
  }

  if (md_comment) {
    md_comment->write(mdfFile);
  }

  return 0;
}

uint64_t siNode::size()
{
  uint64_t s = si.Head.length;
  if (tx_name) {
    s += tx_name->size();
  }
  if (tx_path) {
    s += tx_path->size();
  }
  if (md_comment) {
    s += md_comment->size();
  }
  return s;
}


/*******************************************************************************
* mdNode - Node for MDBLOCK
*******************************************************************************/
mdNode::mdNode(int version, int type, const char *str, const char *display)
{
  std::string metaData;

  setVersion(version);
  memset(&md, 0, sizeof(MDBLOCK));
  memcpy(md.Head.id,MDF_ID_MDBLOCK, 4);

  switch (type) {

  case FH_MD_COMMENT:
    metaData = "<FHcomment>"
               "<TX>Kvaser Memorator Config Tool</TX>"
               "<tool_id>Memorator Config Tool</tool_id>"
               "<tool_vendor>Kvaser</tool_vendor>"
               "<tool_version>5.04</tool_version>"
               "<user_name>User</user_name>"
               "</FHcomment>";
    break;

  case CC_MD_UNIT:
    metaData = "<CCunit><TX>";
    metaData.append(str);
    metaData.append("</TX></CCunit>");
    break;

  case CN_MD_COMMENT:
    metaData = "<CNcomment><TX>";
    metaData.append(str);
    metaData.append("</TX>");
    metaData.append("<names><display>");
    if (display) {
      metaData.append(display);
    } else {
      metaData.append(str);
    }
    metaData.append("</display></names></CNcomment>");
    break;

  case SI_MD_COMMENT:
    metaData = "<SIcomment><TX/><common_properties><e name=\"ChannelNo\" ro=\"true\">";
    metaData.append(str);
    metaData.append("</e></common_properties></SIcomment>");
    break;

  default:
    metaData = "Error: Unknown mdNode type.";
  }

  size_t len = (size_t) align8(metaData.size() + 1);
  md.Head.length = sizeof(BL_HEAD) + len;
  md.md_data = new MDF_BYTE[len];
  memset(md.md_data, 0, len);
  memcpy(md.md_data, metaData.c_str(), metaData.size());
}

mdNode::~mdNode()
{
    PRINTF(("~mdNode (%p)\n", this));
  if (md.md_data) {
    delete [] md.md_data;
  }
}

int mdNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));
  if (md.Head.length) {
    FWRITE((&md, 1, sizeof(BL_HEAD), mdfFile));
    FWRITE((md.md_data, 1, (size_t)md.Head.length - sizeof(BL_HEAD), mdfFile));
  }
  return 0;
}

uint64_t mdNode::size()
{
  return md.Head.length;;
}


/*******************************************************************************
* cnNode - Node for CNBLOCK
*******************************************************************************/

cnNode::~cnNode()
{
  if (md_comment) {
    delete md_comment;
  }
  if (tx_long_signal_name) {
    delete tx_long_signal_name;
  }
  if (cn_si_source) {
    delete cn_si_source;
  }

  if (cc) {
    delete cc;
  }

  if (cn_composition) {
    delete cn_composition;
  }

  if (next) {
    delete next;
  }
  PRINTF(("~cnNode (%p)\n", this));
}

int cnNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));
  if (next) {
    cn.cn_cn_next = next->getCurrentFilePos();
  }
  if (md_comment) {
    cn.cn_md_comment = md_comment->getCurrentFilePos();
  }
  if (tx_long_signal_name) {
    cn.cn_tx_name = tx_long_signal_name->getCurrentFilePos();
  }

  if (cn_composition) {
    cn.cn_composition = cn_composition->getCurrentFilePos();
  }
  if (cn_data) {
    cn.cn_data = cn_data->getCurrentFilePos();
  }

  if (cn_data_sd) {
    cn.cn_data = cn_data_sd->getCurrentFilePos();
  }

  if (cn_si_source) {
    cn.cn_si_source = cn_si_source->getCurrentFilePos();
  }


  if (cc) {
    cn.cn_cc_conversion = cc->getCurrentFilePos();
    if (cc->md_unit){
      cn.cn_md_unit = cc->md_unit->getCurrentFilePos();
    }
  }
  FWRITE((&cn, 1, (size_t)cn.Head.length, mdfFile));
  if (next) {
    next->write(mdfFile);
  }
  if (md_comment) {
    md_comment->write(mdfFile);
  }
  if (tx_long_signal_name) {
    tx_long_signal_name->write(mdfFile);
  }
  if (cc) {
    cc->write(mdfFile);
  }
  if (cn_composition) {
    cn_composition->write(mdfFile);
  }

  if (cn_si_source) {
    cn_si_source->write(mdfFile);
  }

  return -1;
}

uint64_t cnNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = cn.Head.length;
    if (next) {
      s += next->size();
    }
    if (md_comment) {
      s += md_comment->size();
    }
    if (tx_long_signal_name) {
      s += tx_long_signal_name->size();
    }
    if (cc) {
      s += cc->size();
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

MDF_UINT8 cnNode::toChannelDataType(MDF_UINT16 SigDataType)
{
  switch (SigDataType) {
  // Old v2.x format; never used in KvaLogWriter_MdfSignal
  case MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT: return CN_DATA_TYPE_UNSIGNED_INT_LE;
  case MDF_SIGNAL_DATA_TYPE_SIGNED_INT:   return CN_DATA_TYPE_SIGNED_INT_LE;
  case MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT:   return CN_DATA_TYPE_IEEE_FLOAT_LE;
  case MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE:  return CN_DATA_TYPE_IEEE_FLOAT_LE;

  // Never used in KvaLogWriter_MdfSignal
  case MDF_SIGNAL_DATA_TYPE_STRING:       return CN_DATA_TYPE_STRING_ASCII;
  case MDF_SIGNAL_DATA_TYPE_BYTE_ARRAY:   return CN_DATA_TYPE_BYTE_ARRAY;

  // The only types used in conversion
  case MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_BE: return CN_DATA_TYPE_UNSIGNED_INT_BE;
  case MDF_SIGNAL_DATA_TYPE_SIGNED_INT_BE:   return CN_DATA_TYPE_SIGNED_INT_BE;
  case MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_BE:   return CN_DATA_TYPE_IEEE_FLOAT_BE;
  case MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_BE:  return CN_DATA_TYPE_IEEE_FLOAT_BE;

  case MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE: return CN_DATA_TYPE_UNSIGNED_INT_LE;
  case MDF_SIGNAL_DATA_TYPE_SIGNED_INT_LE:   return CN_DATA_TYPE_SIGNED_INT_LE;
  case MDF_SIGNAL_DATA_TYPE_IEEE_FLOAT_LE:   return CN_DATA_TYPE_IEEE_FLOAT_LE;
  case MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_LE:  return CN_DATA_TYPE_IEEE_FLOAT_LE;

  default:
    PRINTF(("Error: toChannelDataType: Unknown SigDataType %d.", SigDataType));
    return CN_DATA_TYPE_UNSIGNED_INT_LE;
  }
}

cnNode::cnNode(int version)
{
  PRINTF(("[%d]", __LINE__));
  setVersion(version);
  memset(&cn, 0, sizeof(CNBLOCK));
  PRINTF(("[%d]", __LINE__));
  memcpy(cn.Head.id, MDF_ID_CNBLOCK, 4);
  cn.Head.length = sizeof(CNBLOCK);
  cn.Head.link_count = 8;
  tx_long_signal_name = NULL;
  md_comment = NULL;
  cc = NULL;
  next = NULL;
  cn_composition = NULL;
  cn_si_source = NULL;
  cn_data = NULL;
  cn_data_sd = NULL;

  PRINTF(("cnNode (%p)\n", this));
}

cnNode::cnNode(
          int version,
          char *long_name,
          char *name,
          char *unit,
          MDF_UINT16 StartOffset,
          MDF_UINT16 NumBits,
          MDF_UINT16 SigDataType,
          MDF_UINT16 ChannelConversionType,
          MDF_REAL Factor,
          MDF_REAL Offset)
{
  setVersion(version);

  memset(&cn, 0, sizeof(CNBLOCK));
  memcpy(cn.Head.id, MDF_ID_CNBLOCK, 4);
  cn.Head.length = sizeof(CNBLOCK);
  cn.Head.link_count = 8;


  tx_long_signal_name = new txNode(version, long_name);
  md_comment = new mdNode(version, mdNode::CN_MD_COMMENT, name);
  cc = new ccNode(version, unit, ChannelConversionType, Offset, Factor);
  next = NULL;

  cn.cn_type = MDF_CN_TYPE_FIXED_LENGTH;
  cn.cn_sync_type = MDF_CN_SYNC_TYPE_NONE;
  cn.cn_data_type = toChannelDataType(SigDataType);

  cn.cn_byte_offset = StartOffset / 8;
  cn.cn_bit_offset = StartOffset % 8;
  cn.cn_bit_count = NumBits;

  cn_composition = NULL;
  cn_si_source = NULL;
  cn_data = NULL;
  cn_data_sd = NULL;

}

/*******************************************************************************
* cgNode - Node for CGBLOCK
*******************************************************************************/

cgNode::cgNode(int version, char *cg_comment)
{
  setVersion(version);
  memset(&cg, 0, sizeof(CGBLOCK));
  memcpy(cg.Head.id, MDF_ID_CGBLOCK, 4);
  cg.Head.length = sizeof(CGBLOCK);
  cg.Head.link_count = 6; // Number of links in link section
  cn = NULL;
  next = NULL;
  tx_acq_name = NULL;
  si_acq_source = NULL;
  if (cg_comment) {
    tx_acq_name = new txNode(version, cg_comment);
  }
}

// Contructor for Vector format
cgNode::cgNode(int version, int channel)
{
  char tmp[128] = {0};
  setVersion(version);
  memset(&cg, 0, sizeof(CGBLOCK));
  memcpy(cg.Head.id, MDF_ID_CGBLOCK, 4);
  cg.Head.length = sizeof(CGBLOCK);
  cg.Head.link_count = 6; // Number of links in link section
  cn = NULL;
  next = NULL;

  sprintf(tmp, "CAN%d", channel);
  tx_acq_name = new txNode(version, tmp);
  si_acq_source = NULL;
}

cgNode::~cgNode()
{
  PRINTF(("~cgNode (%p)\n", this));
  if (tx_acq_name) {
    delete tx_acq_name;
  }
  if (si_acq_source) {
    delete si_acq_source;
  }

  if (cn) {
    std::stack<cnNode*> st;
    cnNode *tmp = cn;
    st.push(tmp);
    while ((tmp = tmp->next) != NULL) {
      st.push(tmp);
    }
    while (!st.empty()) {
      st.top()->next = NULL;
      delete st.top();
      st.pop();
    }
  }

  if (next) {
    delete next;
  }
}

int cgNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));

  if (next) {
    cg.cg_cg_next = next->getCurrentFilePos();
  }
  if (cn) {
    cg.cg_cn_first = cn->getCurrentFilePos();
  }
  if (tx_acq_name) {
    cg.cg_tx_acq_name = tx_acq_name->getCurrentFilePos();
  }
  if (si_acq_source) {
    cg.cg_si_acq_source = si_acq_source->getCurrentFilePos();
  }

  FWRITE((&cg, 1, (size_t)cg.Head.length, mdfFile));
  if (next) {
    next->write(mdfFile);
  }
  if (cn) {
    cn->write(mdfFile);
  }
  if (tx_acq_name) {
    cg.cg_tx_acq_name = tx_acq_name->write(mdfFile);
  }
  if (si_acq_source) {
    cg.cg_si_acq_source = si_acq_source->write(mdfFile);
  }
  return -1;
}

uint64_t cgNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = cg.Head.length;
    if (next) {
      s += next->size();
    }
    if (cn) {
      s += cn->size();
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

MdfStatus cgNode::new_cn(
                  char *long_name,
                  char *name,
                  char *unit,
                  MDF_UINT16 StartOffset,
                  MDF_UINT16 NumBits,
                  MDF_UINT16 SigDataType,
                  MDF_UINT16 ChannelConversionType,
                  MDF_REAL Factor,
                  MDF_REAL Offset)
{
  cnNode *local_cn;

  local_cn = cn;
  // Insert new cg at first place and add bit offset into MDF record
  cn = new cnNode(getVersion(),
                  long_name,
                  name,
                  unit,
                  StartOffset + MDF_FIRST_DATA_BYTE*8,
                  NumBits,
                  SigDataType,
                  ChannelConversionType,
                  Factor,
                  Offset
                 );
  PRINTF(("new_cn (%p)\n", cn));

  if(!cn) {
    PRINTF(("new_cn failed, unable to create new cnNode\n"));
    return MDF_ERROR_MEMORY;
  }

  // Reinsert previously first cn at second place
  cn->next = local_cn;
  setSizeDirty();
  return MDF_OK;
}

MdfStatus cgNode::new_vector_cn()
{
  cnNode *local_cn;

  local_cn = cn;
  // Insert new cg at first place
  cn = new cnNode(getVersion());
  if(!cn) {
    PRINTF(("new_cn failed, unable to create new cnNode\n"));
    return MDF_ERROR_MEMORY;
  }

  // Reinsert previously first cn at second place
  cn->next = local_cn;
  setSizeDirty();
  return MDF_OK;
}


void cgNode::increaseNumberOfMessages(int num)
{
  cg.cg_cycle_count+=num;
}

/*******************************************************************************
* dtNode - Node for DTBLOCK
*******************************************************************************/
dtNode::dtNode(int version)
{
  setVersion(version);
  memset(&dt, 0, sizeof(DTBLOCK));
  memcpy(dt.Head.id, MDF_ID_DTBLOCK, 4);
  dt.Head.length = sizeof(DTBLOCK); // Empty size
  dt.Head.link_count = 0; // No link section
  bq = NULL;
  RecordSize = 0;
  record = NULL;
  dataRecords = 0;
  dl = NULL;

  // Zipped version of DTBLOCK
  memset(&dz, 0, sizeof(DZBLOCK));
  memcpy(dz.Head.id, MDF_ID_DZBLOCK, 4);
  dz.Head.length = sizeof(DZBLOCK); // Empty size
  dz.Head.link_count = 0; // No link section
  memcpy(dz.dz_org_block_type, &MDF_ID_DTBLOCK[2], 2); // Will replace DT block
  dz.dz_zip_type = MDF_DZ_ZIP_TYPE_DEFLATE;
}

dtNode::~dtNode()
{
  PRINTF(("~dtNode (%p)\n", this));
  if (bq) {
    delete bq;
  }
  if (record) {
    delete [] record;
  }
}

int dtNode::write(FILE *mdfFile)
{
  size_t alignmentBytes = 0;
  MDF_UINT64 bytesWritten = 0;
  MDF_BYTE tmp[8] = {0};
  MDF_LINK64 filePos;
  setCurrentFilePos(FTELL(mdfFile));

  if (options.useZippedData) {
    FWRITE((&dz, 1, sizeof(DZBLOCK), mdfFile));
    bytesWritten = sizeof(DZBLOCK);
  } else {
    FWRITE((&dt, 1, sizeof(BL_HEAD), mdfFile));
    bytesWritten = sizeof(BL_HEAD);
  }

  while (bq->size()) {
    if (options.useDataListNodes) {
      if (bytesWritten + RecordSize > MDF_DT_MAX_SIZE) {
        break;
      }
    }
    bq->pop(record);
    bytesWritten += RecordSize;
    if (options.useZippedData) {
      dl->add_data_bytes(record, RecordSize);
    } else {
      FWRITE((record, RecordSize, 1, mdfFile));
    }
  }

  if (options.useZippedData) {
    char *buf;
    size_t len;
    dl->get_zipped_data(&buf, &len);
    dz.dz_org_data_length = bytesWritten - sizeof(DZBLOCK);
    data_size = dz.dz_org_data_length;
    dz.dz_data_length = len;
    dz.Head.length = sizeof(DZBLOCK) + len;
    bytesWritten = dz.Head.length;
    FWRITE((buf, len, 1, mdfFile));
  } else {
    data_size = bytesWritten - sizeof(BL_HEAD);
    dt.Head.length = bytesWritten;
  }

  if (bytesWritten % 8) {
    alignmentBytes = (size_t) (align8(bytesWritten) - bytesWritten);
    FWRITE((tmp, 1, alignmentBytes, mdfFile));
  }

  // Rewind and update size
  filePos = FTELL(mdfFile);
  FSEEK((mdfFile, getCurrentFilePos(), SEEK_SET));
  if (options.useZippedData) {
    FWRITE((&dz, 1, sizeof(DZBLOCK), mdfFile));
  } else {
    FWRITE((&dt, 1, sizeof(BL_HEAD), mdfFile));
  }
  FSEEK((mdfFile, filePos, SEEK_SET));

  // Add a new dtNode to list for any remaining data
  if (options.useDataListNodes && bq->size()) {
    dtNode *next = new dtNode(getVersion());
    // Move queue and record buffer to next dtNode
    *next = *this;
    bq = NULL;
    record = NULL;
    dl->add_data_node(next);
    next->write(mdfFile);
  }
  return 0;
}

uint64_t dtNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = dt.Head.length;
    // Add data records only if write() has not been called
    if ( s == sizeof(DTBLOCK) && dataRecords != 0 ) {
      s += dataRecords * RecordSize;
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

/*******************************************************************************
* sdNode - Node for SDBLOCK
*******************************************************************************/
sdNode::sdNode(int version)
{
  setVersion(version);
  memset(&sd, 0, sizeof(SDBLOCK));
  memcpy(sd.Head.id, MDF_ID_SDBLOCK, 4);
  sd.Head.length = sizeof(SDBLOCK); // Empty size
  sd.Head.link_count = 0; // No link section

  // Zipped version of SDBLOCK
  memset(&dz, 0, sizeof(DZBLOCK));
  memcpy(dz.Head.id, MDF_ID_DZBLOCK, 4);
  dz.Head.length = sizeof(DZBLOCK); // Empty size
  dz.Head.link_count = 0; // No link section
  memcpy(dz.dz_org_block_type, &MDF_ID_SDBLOCK[2], 2); // Will replace DT block
  dz.dz_zip_type = MDF_DZ_ZIP_TYPE_DEFLATE;


  bqv = NULL;
  RecordSize = 0;
  record = NULL;
  dataRecords = 0;
  position = 0;
  dl = NULL;
}

sdNode::~sdNode()
{
  PRINTF(("~sdNode (%p)\n", this));

  if (bqv) {
    delete bqv;
  }
  if (record) {
    delete [] record;
  }
}

int sdNode::write(FILE *mdfFile)
{
  size_t alignmentBytes = 0;
  MDF_UINT64 bytesWritten = 0;
  MDF_BYTE tmp[8] = {0};
  MDF_LINK64 filePos;
  setCurrentFilePos(FTELL(mdfFile));

  if (options.useZippedData) {
    FWRITE((&dz, 1, sizeof(DZBLOCK), mdfFile));
    bytesWritten = sizeof(DZBLOCK);
  } else {
    FWRITE((&sd, 1, sizeof(BL_HEAD), mdfFile));
    bytesWritten = sizeof(BL_HEAD);
  }

  while (bqv->size()) {
    unsigned int len = 0;

    if (options.useDataListNodes) {
      if (bytesWritten + RecordSize > MDF_DT_MAX_SIZE) {
        break;
      }
    }
    bqv->pop(record);
    len = *((unsigned int *) record);
    sd.Head.length += len + sizeof(len);
    bytesWritten += len + sizeof(len);

    if (options.useZippedData) {
      dl->add_data_bytes(record, len + sizeof(len));
    } else {
      FWRITE((&len, sizeof(len), 1, mdfFile));
      FWRITE((&record[sizeof(len)], len, 1, mdfFile));
    }
  }

  if (options.useZippedData) {
    char *buf;
    size_t len;
    dl->get_zipped_data(&buf, &len);
    dz.dz_org_data_length = bytesWritten - sizeof(DZBLOCK);
    data_size = dz.dz_org_data_length;
    dz.dz_data_length = len;
    dz.Head.length = sizeof(DZBLOCK) + len;
    bytesWritten = dz.Head.length;
    FWRITE((buf, len, 1, mdfFile));
  } else {
    data_size = bytesWritten - sizeof(BL_HEAD);
    sd.Head.length = bytesWritten;
  }

  if (bytesWritten % 8) {
    alignmentBytes = (size_t) (align8(bytesWritten) - bytesWritten);
    FWRITE((tmp, 1, alignmentBytes, mdfFile));
  }

  // Rewind and update size
  filePos = FTELL(mdfFile);
  FSEEK((mdfFile, getCurrentFilePos(), SEEK_SET));
  if (options.useZippedData) {
    FWRITE((&dz, 1, sizeof(DZBLOCK), mdfFile));
  } else {
    FWRITE((&sd, 1, sizeof(BL_HEAD), mdfFile));
  }
  FSEEK((mdfFile, filePos, SEEK_SET));

  // Add a new sdNode to list for any remaining data
  if (options.useDataListNodes && bqv->size()) {
    sdNode *next = new sdNode(getVersion());
    // Move queue and record buffer to next dtNode
    *next = *this;
    bqv = NULL;
    record = NULL;
    dl->add_data_node(next);
    next->write(mdfFile);
  }
  return 0;
}

uint64_t sdNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = sd.Head.length;
    // Add data records only if write() has not been called
    if ( s == sizeof(SDBLOCK) && dataRecords != 0 ) {
      s += position;
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

/*******************************************************************************
* atNode - Node for ATBLOCK
*******************************************************************************/
atNode::atNode(int version, const char *fname)
{
  setVersion(version);
  memset(&at, 0, sizeof(ATBLOCK));
  memcpy(at.Head.id, MDF_ID_ATBLOCK, 4);
  at.Head.length = sizeof(ATBLOCK) - sizeof(MDF_BYTE*); // Empty size
  at.Head.link_count = 4;
  tx = new txNode(version, fname);
  filename = std::string(fname);
  next = NULL;
}

atNode::~atNode()
{
  PRINTF(("~atNode (%p)\n", this));

  if (tx) {
    delete tx;
  }
}

int atNode::write(FILE *mdfFile)
{
  if (tx) {
    at.at_tx_filename = tx->getCurrentFilePos();
  }

  if (next) {
    at.at_at_next = next->getCurrentFilePos();
  }

  // Update size
  MDF_LINK64 filePos = FTELL(mdfFile);
  FSEEK((mdfFile, getCurrentFilePos(), SEEK_SET));
  FWRITE((&at, 1, sizeof(ATBLOCK) - sizeof(MDF_BYTE*), mdfFile));
  FSEEK((mdfFile, filePos, SEEK_SET));

  if (next) {
    next->write(mdfFile);
  }

  return 0;
}

int atNode::write_data(FILE *mdfFile)
{
  FILE *h;
  MDF_UINT64 fileSize;
  MDF_BYTE *buf;
  size_t bytesRead = 0;
  size_t bytesWritten = 0;
  size_t alignmentBytes = 0;
  MDF_BYTE tmp[8] = {0};

  PRINTF(("Open file '%s'", filename.c_str()));
  //h = fopen(filename.c_str() , "rb" );
  h = utf_fopen(filename.c_str(), "rb");
  if (h == NULL) {
    PRINTF(("Could not open file '%s'", filename.c_str()));
    return -1;
  }

  FSEEK((h, 0, SEEK_END));
  fileSize = FTELL(h);
  os_rewind(h);
  PRINTF(("Filesize: %lu", fileSize));

  buf = (MDF_BYTE*) malloc(AT_BLOCK_BUFFER_SIZE);
  if (!buf) {
    PRINTF(("Error: Could not allocate memory for atNode"));
    fclose(h);
    return -1;
  }
  setCurrentFilePos(FTELL(mdfFile));
  at.Head.length += fileSize;
  at.at_flags = MDF_AT_FLAG_EMBEDDED | MDF_AT_FLAG_MD5_VALID;
  at.at_creator_index = 0;         // First FH block is creator
  at.at_original_size = fileSize;  // Original data size in Bytes
  at.at_embedded_size = fileSize;  // Embedded data size N, i.e. number of Bytes

  md5sum16(filename.c_str(), at.at_md5_checksum, sizeof(at.at_md5_checksum));

  FWRITE((&at, 1, sizeof(ATBLOCK) - sizeof(MDF_BYTE*), mdfFile));

  while (!feof(h)) {
    bytesRead = fread(buf, 1, AT_BLOCK_BUFFER_SIZE, h);
    PRINTF(("Read %lu bytes.", bytesRead));
    if (!bytesRead) {
      break;
    }
    bytesWritten = FWRITE((buf, 1, bytesRead, mdfFile));
    PRINTF(("Wrote %lu bytes.", bytesWritten));
    if (bytesRead != bytesWritten) {
      PRINTF(("Error: Read/write mismatch!"));
    }
  }
  fclose (h);
  if (buf) free(buf);

  // Align on 8 byte if needed
  if (at.Head.length % 8) {
    alignmentBytes = (size_t) (align8(at.Head.length) - at.Head.length);
    FWRITE((tmp, 1, alignmentBytes, mdfFile));
  }

  if (tx) {
    tx->write(mdfFile);
  }

  if (next) {
    next->write_data(mdfFile);
  }
  return 0;
}


uint64_t atNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = at.Head.length;
    if (tx) {
      s += tx->size();
    }
    if (next) {
      s += next->size();
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}
/*******************************************************************************
* dlNode - Node for DLBLOCK
*******************************************************************************/
dlNode::dlNode(int version)
{
  setVersion(version);
  memset(&dl, 0, sizeof(DLBLOCK));

  memcpy(dl.Head.id, MDF_ID_DLBLOCK, 4);
  dl.Head.length = sizeof(DLBLOCK);
  dl.Head.link_count = 1; // Number of links in link section; matches dtArray.
  next = NULL;
  input = NULL;
  output = NULL;
  input_len = 0;

}

dlNode::~dlNode()
{
  if (next) {
    delete next;
  }

  for (size_t k = 0; k < nodeList.size(); k++) {
    delete nodeList[k];
  }
  nodeList.clear();
  PRINTF(("~dlNode (%p)\n", this));
}

int dlNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));

  // Update and header
  if (next) {
    dl.dl_dl_next = next->getCurrentFilePos();
  }
  dl.Head.link_count = nodeList.size() + 1;
  dl.Head.length = sizeof(DLBLOCK) + nodeList.size() * (sizeof(MDF_LINK64) + sizeof(MDF_UINT64));

  // write header
  FWRITE((&dl, 1, sizeof(BL_HEAD), mdfFile));

  // Update and write links
  FWRITE((&dl.dl_dl_next, 1, sizeof(MDF_LINK64), mdfFile));
  for (size_t k = 0; k < nodeList.size(); k++) {
    MDF_LINK64 dl_data = nodeList[k]->getCurrentFilePos();
    FWRITE((&dl_data, 1, sizeof(MDF_LINK64), mdfFile));
  }

  // Update and write data
  dl.dl_count = (MDF_UINT32) nodeList.size();
  FWRITE((&dl.dl_flags, 1, sizeof(dl.dl_flags), mdfFile));
  FWRITE((dl.dl_reserved, 1, sizeof(dl.dl_reserved), mdfFile));
  FWRITE((&dl.dl_count, 1, sizeof(dl.dl_count), mdfFile));

  MDF_UINT64 dl_offset = 0;
  for (size_t k = 0; k < nodeList.size(); k++) {
    FWRITE((&dl_offset, 1, sizeof(MDF_UINT64), mdfFile));
    dl_offset += nodeList[k]->data_size;
  }

  if (next) {
    next->write(mdfFile);
  }

  return 0;
}

uint64_t dlNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = dl.Head.length;
    if (next) {
      s += next->size();
    }

    for (size_t k = 0; k < nodeList.size(); k++) {
      s += nodeList[k]->size();
    }
    setSize(s);
    return s;
    }
  else {
    return getLastSize();
  }
}

int dlNode::write_data(FILE *mdfFile)
{
  // Allocate memory for zip
  if (options.useZippedData) {
    input = new char[MDF_DT_MAX_SIZE];
    output = new char[MDF_DT_MAX_SIZE];
  }

  if (nodeList.size()) {
    // Additional nodes added to nodeList by write() as needed
    nodeList[0]->write(mdfFile);
  }

  if (input) {
    delete [] input;
    input = NULL;
  }
  if (output) {
    delete [] output;
    output = NULL;
  }

  if (next) {
    return next->write_data(mdfFile);
    next->write(mdfFile);
  }
  write(mdfFile);

  return 0;
}

void dlNode::add_data_node(NodeBase *node)
{
  nodeList.push_back(node);
}

void dlNode::add_data_bytes(char *data, size_t len)
{
  if (input_len + len < MDF_DT_MAX_SIZE) {
    memcpy(&input[input_len], data, len);
    input_len += len;
  }
}

void dlNode::get_zipped_data(char **data, size_t *len)
{
  z_stream defstream;
  defstream.zalloc = Z_NULL;
  defstream.zfree = Z_NULL;
  defstream.opaque = Z_NULL;
  defstream.data_type = Z_BINARY;

  defstream.avail_in = (uInt) input_len;
  defstream.next_in = (Bytef *) input;
  defstream.avail_out = MDF_DT_MAX_SIZE;
  defstream.next_out = (Bytef *) output;

  deflateInit(&defstream, compression_level);
  deflate(&defstream, Z_FINISH);
  deflateEnd(&defstream);
  if (defstream.msg) {
    PRINTF(("dlNode::get_zipped_data: '%s'\n", defstream.msg));
    *len = 0;
  }
  *data = output;
  *len = defstream.total_out;
  input_len = 0;
}



/*******************************************************************************
* hlNode - Node for HLBLOCK
*******************************************************************************/
hlNode::hlNode(int version)
{
  setVersion(version);
  memset(&hl, 0, sizeof(HLBLOCK));
  memcpy(hl.Head.id, MDF_ID_HLBLOCK, 4);
  hl.Head.length = sizeof(HLBLOCK);
  hl.Head.link_count = 1;
  hl.hl_zip_type = MDF_DZ_ZIP_TYPE_DEFLATE;
  dl = NULL;
}

hlNode::~hlNode()
{
  if (dl) {
    delete dl;
  }
  PRINTF(("~hlNode (%p)\n", this));
}

uint64_t hlNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = hl.Head.length;
    if (dl) {
      s += dl->size();
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

int hlNode::write_data(FILE *mdfFile)
{
  // Write data blocks and data list to file
  if (dl) {
    dl->write_data(mdfFile);
  }

  setCurrentFilePos(FTELL(mdfFile));

  if (dl) {
    hl.hl_dl_first = dl->getCurrentFilePos();
  }

  FWRITE((&hl, 1, sizeof(HLBLOCK), mdfFile));

  return 0;
}

void hlNode::add_data_node(NodeBase *node)
{
  if (!dl) {
    dl = new dlNode(getVersion());
  }
  dl->add_data_node(node);
}

/*******************************************************************************
* dgNode - Node for DGBLOCK
*******************************************************************************/

dgNode::dgNode(int version)
{
  setVersion(version);
  memset(&dg, 0, sizeof(DGBLOCK));
  next = NULL;
  cg = NULL;
  dt = NULL;
  sd = NULL;
  cn_time = NULL;
  dl = NULL;
  hl = NULL;
  dl_sd = NULL;
  hl_sd = NULL;

  memcpy(dg.Head.id, MDF_ID_DGBLOCK, 4);
  dg.Head.length = sizeof(DGBLOCK);
  dg.Head.link_count = 4; // Number of links in link section

  bq = NULL;
  mux_checker = MuxChecker(0,-2);
  can_channel = -1;
  frame_type = -1;
  canId = CAN_ID_NOT_USED;
  if (options.useDataListNodes) {
    if (options.useZippedData) {
      hl = new hlNode(getVersion());
      hl_sd = new hlNode(getVersion());
    } else {
      dl = new dlNode(getVersion());
      dl_sd = new dlNode(getVersion());
    }
  }
}

dgNode::~dgNode()
{
  if (next) {
    delete next;
  }

  if (cg) {
    std::stack<cgNode*> st;
    cgNode *tmp = cg;
    st.push(tmp);
    while ((tmp = tmp->next) != NULL) {
      st.push(tmp);
    }
    while (!st.empty()) {
      st.top()->next = NULL;
      delete st.top();
      st.pop();
    }
  }

  if (options.useDataListNodes) {
    if (hl) delete hl;
    // Deleting dl will also delete dt
    if (dl) delete dl;
    // Deleting dl will also delete sd
    if (hl_sd) delete hl_sd;
    if (dl_sd) delete dl_sd;
  } else {
    if (dt) delete dt;
    if (sd) delete sd;
  }

  PRINTF(("~dgNode (%p)\n", this));
}

int dgNode::write(FILE *mdfFile)
{

  setCurrentFilePos(FTELL(mdfFile));
  if (next) {
    dg.dg_dg_next = next->getCurrentFilePos();
  }
  if (cg) {
    dg.dg_cg_first = cg->getCurrentFilePos();
  }

  if (options.useDataListNodes) {
    if (hl) dg.dg_data = hl->getCurrentFilePos();
    if (dl) dg.dg_data = dl->getCurrentFilePos();
  } else {
    if (dt) dg.dg_data = dt->getCurrentFilePos();
  }

  FWRITE((&dg, 1, (size_t)dg.Head.length, mdfFile));
  if (next) {
    next->write(mdfFile);
  }

  if (cg) {
    cg->write(mdfFile);
  }

  return 0;
}

uint64_t dgNode::size()
{
  if (isSizeDirty()) {
    uint64_t s = dg.Head.length;
    if (next) {
      s += next->size();
    }
    if (cg) {
      s += cg->size();
    }
    if (options.useDataListNodes) {
      if (dl) s += dl->size();
      if (hl) s += hl->size();
      if (hl_sd) s += hl_sd->size();
    } else {
      if (dt) s += dt->size();
    }
    setSize(s);
    return s;
    }
  else {
    return getLastSize();
  }
}

MdfStatus dgNode::new_mdf4_cg(int channel, int type)
{
  cgNode *local_cg;
  MdfStatus stat = MDF_OK;

  local_cg = cg;
  // Insert new cg at first place
  cg = new cgNode(getVersion(), channel);

  if(!cg) {
    PRINTF(("new_mdf4_cg failed, unable to create new cgNode\n"));
    return MDF_ERROR_MEMORY;
  }
  // Reinsert previously first cg at second place
  cg->next = local_cg;

  // Number of data Bytes (after record ID)
  switch (type) {
    case MDF_CAN_FRAME_TYPE:
      if (options.useVariableRecordSize) {
        cg->cg.cg_data_bytes = MDF_VLSD_RECORD_SIZE;
      } else {
        if (options.useFixedRecordSize64) {
          cg->cg.cg_data_bytes = MDF_MLSD_RECORD_SIZE_FD;
        } else {
          cg->cg.cg_data_bytes = MDF_MLSD_RECORD_SIZE;
        }
      }
      break;

    case MDF_ERROR_FRAME_TYPE:
      cg->cg.cg_data_bytes = MDF_MLSD_RECORD_SIZE;
      break;

    case MDF_REMOTE_FRAME_TYPE:
      cg->cg.cg_data_bytes = MDF_MLSD_RECORD_SIZE;
      break;
  }
  setSizeDirty();
  return stat;
}

int dgNode::addFrame(int channel, int type, KVMDF_TIMESTAMP ts, MDF_UINT8 *data)
{
  int ret_code = -1;
  PRINTF(("dgNode::addFrame channel = %d and can_channel = %d, type = %d\n", channel, can_channel, type));
  unsigned int dataLength = data[DATA_LENGTH_BYTE];
  MDF_UINT32 id = *((MDF_UINT32*) &data[0]);
  MDF_UINT8 *frame_data = &data[FIRST_DATA_BYTE];

  if (can_channel == channel && frame_type == type) {
    // Sort on message ID
    if (canId != CAN_ID_NOT_USED) {
      if (((canId & canMask) == (id & canMask)) && mux_checker.inEvent(frame_data, dataLength) ) {
        // Correct channel and signal
      } else if (next) {
        setSizeDirty();
        return next->addFrame(channel, type, ts, data);
      } else {
        // This message did not match any signal; discard
        return 0;
      }
    }

    if (cn_time) {
      if (dt->dataRecords == 0) {
        cn_time->cc->cc.cc_phy_range_min = TIMESTAMP_TO_SECONDS * ts;
        cn_time->cn.cn_val_range_min = (MDF_REAL) ts;
      }
      cn_time->cn.cn_val_range_max = (MDF_REAL) ts;
      cn_time->cc->cc.cc_phy_range_max = TIMESTAMP_TO_SECONDS * ts;
    }

    if (options.useVariableRecordSize && sd != NULL) {
      memset(sd->record, 0 , sd->RecordSize);
      memcpy(sd->record, &dataLength, sizeof(dataLength));
      memcpy(&sd->record[sizeof(dataLength)], &data[FIRST_DATA_BYTE], dataLength);
    }
    memcpy(dt->record, &ts, sizeof(KVMDF_TIMESTAMP));
    // Always copy the first 8 CAN data bytes
    memcpy(&dt->record[sizeof(KVMDF_TIMESTAMP)], data, dt->RecordSize - sizeof(KVMDF_TIMESTAMP));

    if (cg) {
      cg->increaseNumberOfMessages();
    }

    if (dt && dt->bq) {
      if (options.useVariableRecordSize && frame_type == MDF_CAN_FRAME_TYPE) {
        // Overwrite the 8 CAN data bytes with position in SDBLOCK
        memcpy(&dt->record[MDF_FIRST_DATA_BYTE], &sd->position, sizeof(sd->position));
      }
      dt->bq->push(dt->record);
      dt->dataRecords++;
    }
    else {
      PRINTF(("BQueue is not allocated!!!\n"));
      return -1;
    }

    if (sd && sd->bqv) {
      sd->bqv->push(sd->record);
      sd->dataRecords++;
      sd->position += dataLength + sizeof(dataLength);
    }
    setSizeDirty();
    ret_code = 0;
  }

  if (next) {
    setSizeDirty();
    return next->addFrame(channel, type, ts, data);
  }
  return ret_code;
}


MdfStatus dgNode::new_vector_can_frame(int channel, int type)
{
  if (can_channel == channel && frame_type == type) {
    setSizeDirty();
    return cg->new_vector_cn();
  }
  if (next) {
    setSizeDirty();
    return next->new_vector_can_frame(channel, type);
  }
  return MDF_ERROR_MEMORY;
}


MdfStatus dgNode::setRecordSize(MDF_UINT32 recordSize)
{
  if (dt) {
    dt->RecordSize = (MDF_UINT8)recordSize;
    PRINTF(("setRecordSize(%u)", recordSize));
    dt->record = new char[recordSize];
    if (!dt->record) {
      return MDF_ERROR_MEMORY;
    }
  }
  return MDF_ERROR_MEMORY;
}

MdfStatus dgNode::setVariableRecordSize(MDF_UINT32 recordSize)
{
  if (sd) {
    PRINTF(("setVariableRecordSize(%d) @ %p", recordSize, sd));
    sd->RecordSize = (MDF_UINT8)recordSize;
    sd->record = new char[sd->RecordSize];
    if (!sd->record) {
      return MDF_ERROR_MEMORY;
    }
  }
  return MDF_ERROR_MEMORY;

}


int dgNode::write_data(FILE *mdfFile)
{
  if (options.useDataListNodes) {
    if (hl) hl->write_data(mdfFile);
    if (dl) dl->write_data(mdfFile);
  } else {
    if (dt) dt->write(mdfFile);
  }

  if (options.useVariableRecordSize && sd) {
    if (options.useDataListNodes) {
      if (hl_sd) hl_sd->write_data(mdfFile);
      if (dl_sd) dl_sd->write_data(mdfFile);
    } else {
      sd->write(mdfFile);
    }
  }
  if (next) {
    return next->write_data(mdfFile);
  }
  return 0;
}

MdfStatus dgNode::new_sig(MDF_UINT32 id, char *longname, char *shortname, char *unit, MDF_UINT16 StartOffset, MDF_UINT16 NumBits, MDF_UINT16 SigDataType, MDF_UINT16 ChannelConversionType, MDF_REAL Factor, MDF_REAL Offset)
{
  if (canId == id) {
    setSizeDirty();
    MdfStatus stat = cg->new_cn(longname, shortname, unit, StartOffset, NumBits, SigDataType, ChannelConversionType, Factor, Offset);
    return stat;
  }
  if (next) {
    setSizeDirty();
    return next->new_sig(canId, longname, shortname, unit, StartOffset, NumBits, SigDataType, ChannelConversionType, Factor, Offset);
  }
  return MDF_ERROR_MEMORY;
}

/*******************************************************************************
* hdNode - Node for HDBLOCK
*******************************************************************************/
hdNode::hdNode(int version)
{
  setVersion(version);
  memset(&hd, 0, sizeof(HDBLOCK));
  tx = NULL;
  dg = NULL;
  cur_dg = NULL;
  at = NULL;

  memcpy(hd.head.id, MDF_ID_HDBLOCK, 4);
  hd.head.length =   sizeof(HDBLOCK);
  hd.head.link_count = 6;

  hd.hd_dg_first = 0;
  hd.hd_fh_first = 0;
  hd.hd_ch_first = 0;
  hd.hd_at_first = 0;
  hd.hd_ev_first = 0;
  hd.hd_md_comment = 0;

  hd.hd_time_class = 10; // external time source

  fh = new fhNode(version);

  hd.hd_fh_first = sizeof(IDBLOCK) + sizeof(HDBLOCK);

}

void hdNode::attach(const char *fname)
{
  atNode *tmp;
  if (!at) {
    at = new atNode(getVersion(), fname);
  } else {
    tmp = at;
    while (tmp->next) {
      tmp = tmp->next;
    }
    tmp->next = new atNode(getVersion(), fname);
  }
}

hdNode::~hdNode()
{
  if (dg) {
    std::stack<dgNode*> st;
    dgNode *tmp = dg;
    st.push(tmp);
    while ((tmp = tmp->next) != NULL) {
      st.push(tmp);
    }
    while (!st.empty()) {
      st.top()->next = NULL;
      delete st.top();
      st.pop();
    }
  }

  if (fh) {
    delete fh;
  }

  if (at) {
    std::stack<atNode*> st;
    atNode *tmp = at;
    st.push(tmp);
    while ((tmp = tmp->next) != NULL) {
      st.push(tmp);
    }
    while (!st.empty()) {
      st.top()->next = NULL;
      delete st.top();
      st.pop();
    }
  }
}

int hdNode::write(FILE *mdfFile)
{
  if (dg) {
    hd.hd_dg_first = dg->getCurrentFilePos();
  }

  if (at) {
    hd.hd_at_first = at->getCurrentFilePos();
  }

  FWRITE((&hd, (size_t)hd.head.length, 1, mdfFile));
  fh->write(mdfFile);

  if (dg) {
    dg->write(mdfFile);
  }

  if (at) {
    at->write(mdfFile);
  }

  return 0;
}

uint64_t hdNode::size()
{
  if (isSizeDirty()) {
   uint64_t s = hd.head.length;
    if (dg) {
      s += dg->size();
    }
    if (fh) {
      s += fh->size();
    }
    setSize(s);
    return s;
 }
  else {
    return getLastSize();
  }
}

MdfStatus hdNode::new_vector_dg(int channel, int type)
{
  dgNode *local_dg;
  char   prefix[128] = {0};
  MdfStatus stat;
  local_dg = dg;
  while (dg) {
    if (dg->can_channel == channel && dg->frame_type == type) {
      cur_dg = dg;
      dg = local_dg;
      return MDF_OK;
    }
    dg = dg->next;
  }

  // Insert new dg at first place
  dg = new dgNode(getVersion());
  if (!dg) {
    PRINTF(("new_vector_dg failed, unable to create new dgNode\n"));
    return MDF_ERROR_MEMORY;
  }
  // Reinsert previously first dg at second place
  dg->next = local_dg;
  dg->can_channel = channel;
  dg->frame_type = type;

  stat = dg->new_mdf4_cg(channel, type);
  if (stat != MDF_OK) {
    return stat;
  }

  sprintf(prefix, "MDF_%08x_%08x", channel, type);

  dg->dt = new dtNode(getVersion());
  if (options.useDataListNodes) {
    if (options.useZippedData) {
      dg->hl->add_data_node(dg->dt);
      dg->dt->dl = dg->hl->dl;
    } else {
      dg->dl->add_data_node(dg->dt);
      dg->dt->dl = dg->dl;
    }
  }

    // Number of data Bytes (after record ID)
  switch (type) {
    case MDF_CAN_FRAME_TYPE:
      if (options.useVariableRecordSize) {
        dg->sd = new sdNode(getVersion());
        if (!dg->sd) return MDF_ERROR_MEMORY;
        dg->setVariableRecordSize(MDF_MAX_SD_BLOCK_SIZE);
        sprintf(prefix, "MF4_%08x_%08x", channel, type);
        dg->sd->bqv = new BQueue(prefix, 10000, dg->sd->RecordSize);
        if (!dg->sd->bqv) return MDF_ERROR_MEMORY;

        if (options.useDataListNodes) {
          if (options.useZippedData) {
            if (dg->hl_sd)  dg->hl_sd->add_data_node(dg->sd);
            dg->sd->dl = dg->hl_sd->dl;
          } else {
            if (dg->dl_sd)  dg->dl_sd->add_data_node(dg->sd);
            dg->sd->dl = dg->dl_sd;
          }
        }
      }
      if (options.useFixedRecordSize64) {
          dg->setRecordSize(MDF_MLSD_RECORD_SIZE_FD);
      } else {
          dg->setRecordSize(MDF_MLSD_RECORD_SIZE);
      }
      break;

    case MDF_ERROR_FRAME_TYPE:
      dg->setRecordSize(MDF_MLSD_RECORD_SIZE);
      break;

    case MDF_REMOTE_FRAME_TYPE:
      dg->setRecordSize(MDF_MLSD_RECORD_SIZE);
      break;
  }
  dg->dt->bq = new BQueue(prefix, 10000, dg->dt->RecordSize);
  if (!dg->dt->bq) return MDF_ERROR_MEMORY;

  cur_dg = dg;
  setSizeDirty();
  return MDF_OK;
}

MdfStatus hdNode::new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 /* dlc */, const std::string& msgname, int channel)
{
  dgNode *local_dg;

  local_dg = dg;
  while (dg) {
    if (((canMask & dg->canId) == (canMask & canId))&&(dg->mux_checker.getValue() == mux.getValue()) && dg->can_channel==channel) {
      cur_dg = dg;
      dg = local_dg;
      return MDF_OK; //dgblock for these kind of signals is already created
    }
    dg = dg->next;
  }

  // Will set cur_dg.
  createMdf4Channel(channel, MDF_CAN_FRAME_TYPE, canId, msgname);

  // Reinsert previously first dg at second place
  cur_dg->next = local_dg;
  cur_dg->canId = canId;
  cur_dg->canMask = canMask;
  cur_dg->mux_checker=MuxChecker(mux);
  setSizeDirty();
  return MDF_OK;
}

MdfStatus hdNode::new_sig(MDF_UINT32 canId, char *longname, char *shortname, char *unit, MDF_UINT16 StartOffset, MDF_UINT16 NumBits, MDF_UINT16 SigDataType, MDF_UINT16 ChannelConversionType, MDF_REAL Factor, MDF_REAL Offset)
{
  if (!cur_dg) {
    cur_dg = dg; //if not set, take the first node
  }
  if (cur_dg) {
    setSizeDirty();
    return cur_dg->new_sig(canId, longname, shortname, unit, StartOffset, NumBits, SigDataType, ChannelConversionType, Factor, Offset);
  }
  return MDF_ERROR_MEMORY;
}

int hdNode::setStartOfRecording (
                                  unsigned int year,
                                  unsigned char month,
                                  unsigned char day,
                                  unsigned char hour,
                                  unsigned char minute,
                                  unsigned char second
                                )
{

  hd.hd_start_time_ns = (MDF_UINT64) convert_time(year, month, day, hour, minute, second);
  hd.hd_start_time_ns =  hd.hd_start_time_ns * ONE_BILLION;
  return 0;
}

int hdNode::addFrame(int channel, int type, KVMDF_TIMESTAMP ts, MDF_UINT8 *data)
{
  PRINTF(("hdNode::addFrame(%d, %d)", channel, type));
  if (dg) {
    setSizeDirty();
    return dg->addFrame(channel, type, ts, data);
  }
  return -1;
}

int hdNode::write_data(FILE *mdfFile)
{
  PRINTF(("hdNode::write_data"));
  if (dg) {
    dg->write_data(mdfFile);
  }
  if (at) {
    at->write_data(mdfFile);
  }
  return 0;
}

MdfStatus hdNode::new_vector_can_frame(int channel, int type)
{
  if (!cur_dg) {
    cur_dg = dg; //if not set, take the first node
  }
  if (cur_dg) {
    setSizeDirty();
    return cur_dg->new_vector_can_frame(channel, type);
  }
  return MDF_ERROR_MEMORY;
}

/*******************************************************************************
* ccNode - Node for CCBLOCK
*******************************************************************************/

ccNode::ccNode(int version, const char *unit, MDF_UINT16 type, MDF_REAL p1, MDF_REAL p2)
{
  setVersion(version);
  memset(&cc, 0, sizeof(CCBLOCK));

  memcpy(cc.Head.id, MDF_ID_CCBLOCK, 4);
  cc.Head.length = sizeof(CCBLOCK) - sizeof(MDF_REAL*);
  cc.cc_ref_count = 0;

  // No cc_ref blocks needed
  cc.Head.link_count = 4;

  md_unit = NULL;
  if (unit) {
    md_unit = new txNode(version, unit);
  }

  switch (type) {
    case MDF_CONVERSION_TYPE_ONE_TO_ONE:
      cc.cc_type = CC_TYPE_CONVERSION_TYPE_ONE_TO_ONE;
      cc.cc_val_count = 0;
      break;

    case MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR:
      cc.cc_type = CC_TYPE_CONVERSION_TYPE_PARAMETRIC_LINEAR;
      cc.cc_val_count = 2;
      cc.cc_val = new MDF_REAL[cc.cc_val_count];
      if (cc.cc_val) {
        cc.cc_val[0] = p1;
        cc.cc_val[1] = p2;
      }
      break;

    default:
      PRINTF(("ccNode:: unsupported type %d\n", type));
  }
  cc.Head.length += cc.cc_val_count * sizeof(MDF_REAL);
}

ccNode::~ccNode()
{
  if (cc.cc_val) {
    delete [] cc.cc_val;
  }
  if (md_unit) {
    delete md_unit;
  }
}

int ccNode::write(FILE *mdfFile)
{
  setCurrentFilePos(FTELL(mdfFile));

  if (md_unit) {
    cc.cc_md_unit = md_unit->getCurrentFilePos();
  }

  FWRITE((&cc, (size_t)cc.Head.length - cc.cc_val_count * sizeof(MDF_REAL), 1, mdfFile));
  if (cc.cc_val) {
    FWRITE((cc.cc_val, cc.cc_val_count * sizeof(MDF_REAL), 1, mdfFile));
  }

  if (md_unit) {
    md_unit->write(mdfFile);
  }

  return 0;
}

uint64_t ccNode::size()
{
  uint64_t s = cc.Head.length;
  if (md_unit) {
    s += md_unit->size();
  }
  return s;
}


/*******************************************************************************
* idNode - Node for IDBLOCK
*******************************************************************************/
idNode::idNode(int version)
{
  setVersion(version);
  memset(&id, 0, sizeof(IDBLOCK));
  memcpy(id.id_file, MDF_FILE_ID, sizeof(id.id_file));
  if (version == 410) {
    memcpy(id.id_vers, MDF_VERSION_STRING_410, sizeof(id.id_vers));
  }
  memcpy(id.id_prog,MDF_PROGRAM_ID_STRING, sizeof(MDF_PROGRAM_ID_STRING));
  id.id_ver = (MDF_UINT16) version;
}

idNode::~idNode()
{
  // Do nothing
}

int idNode::write(FILE *mdfFile)
{
  FWRITE((&id, sizeof(IDBLOCK), 1, mdfFile));
  return 0;
}

uint64_t idNode::size()
{
  return sizeof(IDBLOCK);;
}


}; //end-of-namespace
