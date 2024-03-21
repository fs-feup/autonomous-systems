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
**  Reader for MDF 4.x
** ---------------------------------------------------------------------------
*/

#include "KvaLogReader_Mdf4.h"
#include <stdlib.h>
#include <stdio.h>
#include "KvaConverterMisc.h"
#include "common_defs.h"
#include "mdf4base.h"
#include "mdf4.h"
#include "zlib.h"
#include "md5sum.h"
#include "os_util.h"
#include "kvdebug.h"

using namespace Mdf4Nodes;
using namespace std;

string KvaLogReader_Mdf4::mDatabaseName;
KvaDbHnd KvaLogReader_Mdf4::mDatabaseHnd = 0;
bool KvaLogReader_Mdf4::mCheckSignals = false;

static const char SEPARATOR = '.';
static char API_STRING[2048] = {0};

static void split_string_to_vector(vector<string>& items, char* str)
{
  // Using separator "."
  char *p = strtok(str,&SEPARATOR);
  items.clear();
  while(p != NULL)
  {
    items.push_back(p);
    p = strtok(NULL,&SEPARATOR);
  }
}

// The KvLogCnv API doesn't always use const char*
static char * toApi(string& str) {
  sprintf(API_STRING, "%s", str.c_str());
  return API_STRING;
}

static void toDbType(unsigned int cn_data_type, KvaDbSignalEncoding *se,
                     KvaDbSignalType *st)
{
  switch(cn_data_type) {
    case cnNode::CN_DATA_TYPE_UNSIGNED_INT_BE:
    *se = kvaDb_Motorola;
    *st = kvaDb_Unsigned;
    break;

    case cnNode::CN_DATA_TYPE_SIGNED_INT_BE:
    *se = kvaDb_Motorola;
    *st = kvaDb_Signed;
    break;

    case cnNode::CN_DATA_TYPE_IEEE_FLOAT_BE:
    *se = kvaDb_Motorola;
    *st = kvaDb_Float;
    break;

    case cnNode::CN_DATA_TYPE_UNSIGNED_INT_LE:
    *se = kvaDb_Intel;
    *st = kvaDb_Unsigned;
    break;

    case cnNode::CN_DATA_TYPE_SIGNED_INT_LE:
    *se = kvaDb_Intel;
    *st = kvaDb_Signed;
    break;

    case cnNode::CN_DATA_TYPE_IEEE_FLOAT_LE:
    *se = kvaDb_Intel;
    *st = kvaDb_Float;
    break;
  }
}

// =============================================================================
//  DEBUG
// =============================================================================
void PRINTF_BLOCK(BL_HEAD *h)
{
  if (h) {
  PRINTF(("%c%c%c%c (%lu)\n",h->id[0], h->id[1], h->id[2] ,h->id[3],
    h->length));
}
}


// =============================================================================
//  MdfBlockBase - corresponds to a block in MDF 4.x
// =============================================================================
class MdfChannelGroup;
class MdfBlockBase {
  public:
    KvaLogReader_Mdf4 *mFile;
    bool isVariableRecordSize;
    MDF_LINK64 mFilePos;
    vector<MDF_LINK64> mFilePosList;
    vector<MDF_UINT64> mOffsetList;
    unsigned int mPosIdx;
    bool mUseDataListBlock;
    bool mUseZippedData;
    char *mZipIn;
    char *mZipOut;
    size_t mZipLen;
    size_t mZipPos;
    size_t mZipOutLen;
    MdfBlockBase(KvaLogReader_Mdf4 *fh);
    virtual ~MdfBlockBase();
    void allocateBuffers();
    void freeBuffers();
    KvlcStatus getData(char *buf, size_t len);
    size_t unzipBuffer(void* buf, size_t len, void* out, size_t outLen);
    KvlcStatus readDataListBlock();
};

MdfBlockBase::MdfBlockBase(KvaLogReader_Mdf4 *fh)
{
  mFile = fh;
  mFilePos = 0;
  mPosIdx = 0;
  mUseDataListBlock = false;
  mUseZippedData = false;
  mZipIn = NULL;
  mZipOut = NULL;
  mZipLen = 0;
  mZipPos = 0;
  mZipOutLen = MDF_DT_MAX_SIZE;
  isVariableRecordSize = false;
}

MdfBlockBase::~MdfBlockBase()
{
  freeBuffers();
}

void MdfBlockBase::allocateBuffers()
{
  if (!mZipIn) mZipIn = new char[MDF_DT_MAX_SIZE];
  if (!mZipOut) mZipOut = new char[MDF_DT_MAX_SIZE];
}

void MdfBlockBase::freeBuffers()
{
  if (mZipIn) {
    delete [] mZipIn;
    mZipIn = NULL;
  }
  if (mZipOut) {
    delete [] mZipOut;
    mZipOut = NULL;
  }
}

size_t MdfBlockBase::unzipBuffer(void* buf, size_t len, void* out, size_t outLen)
{
    z_stream infstream;
    infstream.zalloc = Z_NULL;
    infstream.zfree = Z_NULL;
    infstream.opaque = Z_NULL;
    infstream.avail_in =  (uInt)len;
    infstream.next_in = (Bytef *) buf;
    infstream.avail_out = (uInt) outLen;
    infstream.next_out = (Bytef *) out;
    inflateInit(&infstream);
    inflate(&infstream, Z_NO_FLUSH);
    inflateEnd(&infstream);
    if (infstream.msg) {
      PRINTF(("ERROR: '%s'\n", infstream.msg));
    }
    return infstream.total_out;
}

// Note: Not tested with only mUseDataListBlock set!
KvlcStatus MdfBlockBase::getData(char *buf, size_t len)
{
  KvlcStatus status = kvlcOK;
  size_t dataLen = len;
  unsigned int dataCount = 0;

  if (isVariableRecordSize) {
    // Data consists of dataLength and possibly up to 64 data bytes.
    dataLen = sizeof(dataCount);
  }

  if (mUseZippedData) {
    if (!mZipIn || !mZipOut) {
      allocateBuffers();
    }

    // Get next data block
    if (mZipPos + dataLen > mZipOutLen) {
      if ( mPosIdx+1 < mFilePosList.size()) {
        mFilePos = mFilePosList[++mPosIdx];
        mZipPos = 0;
      }
    }
    // Load data block
    if (mZipPos == 0) {
      DZBLOCK dzBlock;
      // Read and unzip DZBLOCK
      mFile->goto_block(mFilePos);
      mFile->read_block(&dzBlock, sizeof(dzBlock));
      PRINTF_BLOCK(&dzBlock.Head);
      if (dzBlock.dz_data_length > MDF_DT_MAX_SIZE ||
          dzBlock.dz_org_data_length > MDF_DT_MAX_SIZE) {
        PRINTF(("ERROR: Data length: %lu Original length: %lu too large.\n",
                dzBlock.dz_data_length, dzBlock.dz_org_data_length));
        return kvlcERR_FILE_ERROR;
      }
      status = mFile->read_file(mZipIn, (size_t)dzBlock.dz_data_length);
      mZipOutLen = unzipBuffer(mZipIn, (size_t) dzBlock.dz_data_length,
                               mZipOut, MDF_DT_MAX_SIZE);
      if (dzBlock.dz_org_data_length != mZipOutLen) {
        PRINTF(("ERROR: Original length: %lu. Unzipped length %lu.\n",
                dzBlock.dz_org_data_length, mZipOutLen));
        return kvlcERR_FILE_ERROR;
      }
    }

    if (isVariableRecordSize) {

      memcpy(&dataCount, &mZipOut[mZipPos], sizeof(dataCount));
      mZipPos += sizeof(dataCount);
      // Copy data
      memcpy(buf, &mZipOut[mZipPos], dataCount);
      mZipPos += dataCount;
    }
    else
    {
        // Copy data
        memcpy(buf, &mZipOut[mZipPos], len);
        mZipPos += len;
    }
  } else
  {
    mFile->goto_block(mFilePos);
    if (isVariableRecordSize) {
      status = mFile->read_file((char*) &dataCount, sizeof(dataCount));
    }
    status = mFile->read_file(buf, len);
    mFilePos = mFile->get_link();
  }
  return status;
}

KvlcStatus MdfBlockBase::readDataListBlock()
{
  KvlcStatus status = kvlcOK;
  MDF_LINK64 tmp;
  MDF_UINT8   dl_flags;
  MDF_BYTE    dl_reserved[3];
  MDF_UINT32  dl_count;
  BL_HEAD blHead;

  mFile->read_block(&blHead, sizeof(blHead));
  PRINTF_BLOCK(&blHead);
  status = mFile->read_file((char*) &tmp, sizeof(MDF_LINK64));
  mUseDataListBlock = true;
  PRINTF(("Found %lu links.\n", blHead.link_count - 1));

  for (unsigned int k = 0; k < blHead.link_count - 1; k++) {
    status = mFile->read_file((char*) &tmp, sizeof(MDF_LINK64));
    mFilePosList.push_back(tmp);
    PRINTF(("DATA BLOCK[%d] @ 0x%lx\n", k, mFilePosList.at(k)));
  }

  if (mFilePosList.size()) {
    mFilePos = mFilePosList.at(0);
    if (!mUseZippedData) {
      PRINTF(("WARNING: Both mUseDataListBlock and mUseZippedData must be defined.\n"));
      // Skip header for DTBLOCK and SDBLOCK
      mFilePos += sizeof(BL_HEAD);
    }
  }

  // Handle data section
  status = mFile->read_file((char*) &dl_flags, sizeof(dl_flags));
  status = mFile->read_file((char*) dl_reserved, sizeof(dl_reserved));
  status = mFile->read_file((char*) &dl_count, sizeof(dl_count));
  PRINTF(("Found %u offsets.\n", dl_count));
  for (unsigned int k = 0; k < blHead.link_count - 1; k++) {
    status = mFile->read_file((char*) &tmp, sizeof(MDF_LINK64));
    // The offset is counted from first DT/SD block and doesn't include headers
    tmp += mFilePos + k*sizeof(BL_HEAD);
    mOffsetList.push_back(tmp);
    PRINTF(("Offset[%d] @ 0x%lx\n", k, mOffsetList.at(k)));
  }
  return status;
}


// =============================================================================
//  MdfBlockDT - corresponds to a DTBLOCK in MDF 4.x
// =============================================================================
class MdfBlockDT : public MdfBlockBase {
  public:
  MdfBlockDT(KvaLogReader_Mdf4 *fh): MdfBlockBase(fh){
    isVariableRecordSize = false;
  };
  ~MdfBlockDT() {freeBuffers();}
};

// =============================================================================
//  MdfBlockDT - corresponds to a SDBLOCK in MDF 4.x
// =============================================================================
class MdfBlockSD : public MdfBlockBase {
  public:
  MdfBlockSD(KvaLogReader_Mdf4 *fh): MdfBlockBase(fh){
      isVariableRecordSize = true;
  };
  ~MdfBlockSD() {freeBuffers();}
};

// =============================================================================
//  MdfDataChannel - corresponds to CNBLOCK in MDF 4.x
// =============================================================================
class MdfDataChannel {
  public:
    string mName;
    unsigned int mBitOffset;
    unsigned int mByteOffset;
    unsigned int mBitCount;
    unsigned int mDataType;
    MdfDataChannel *mNextChannel;
    KvaLogReader_Mdf4 *mFile;
    void getData(void *data, void *buf, size_t len);
    void setup(const CNBLOCK* cn);
    MdfDataChannel(KvaLogReader_Mdf4 *fh);
    ~MdfDataChannel();
    void checkSignalProperties(KvaDbSignalHnd sh);
    MdfChannelGroup *mChannelGroup;
};

// =============================================================================
//  MdfChannelGroup - corresponds to CGBLOCK in MDF 4.x
// =============================================================================
class MdfChannelGroup {
  public:
    KvlcStatus getEvent(void);
    MdfBlockDT* mDTBlock;
    MdfBlockSD* mSDBlock;
    bool mValid;
    int mChannel;
    MDF_UINT64 mEventCount;
    MDF_UINT32 mDataBytes;
    imLogData mEvent;
    KvaLogReader_Mdf4 *mFile;
    MdfChannelGroup();
    ~MdfChannelGroup();
    enum FrameType {DATA_FRAME, ERROR_FRAME, REMOTE_FRAME};
    FrameType mType;
    bool isErrorFrame() {return (mType == ERROR_FRAME);};
    bool isRemoteFrame() {return (mType == REMOTE_FRAME);};
    bool isDataFrame() {return (mType == DATA_FRAME);};
    MdfDataChannel *mDataChannels;
    MdfDataChannel *mCurrentDataChannel;
    void setFrameType(const char *str);
    void addDataChannel(const CNBLOCK* cn);
    bool getChannelData(const char *name, void *data, void *buf, size_t len);
    void PRINTF_CG();
    bool getFirstSignalChannelData(void *data, string& name, int *value);
    bool getNextSignalChannelData(void *data, string& name, int *value);
    string mFileName;
    unsigned int mMessageId;
    FILE *mFileHandle;                // only used by below functions?
    void writeSignalHeader();         // debug function, write signals to csv file
    void writeSignalData(void *data); // debug function, write signals to csv file
    void checkMessageProperties(KvaDbMessageHnd mh);

    MdfChannelGroup& operator=(const MdfChannelGroup& other) //copy assignment
    {
      if (this != &other) { //check against self-assignment
        delete mDataChannels;
        delete mSDBlock;
        delete mDTBlock;

        mDataChannels = nullptr;
        mSDBlock = nullptr;
        mDTBlock = nullptr;
        mDataChannels = other.mDataChannels;
        mSDBlock = other.mSDBlock;
        mDTBlock = other.mDTBlock;
      }
      return *this;
    }
};

MdfDataChannel::MdfDataChannel(KvaLogReader_Mdf4 *fh)
{
  mFile = fh;
  mNextChannel = NULL;
  mBitOffset = 0;
  mByteOffset = 0;
  mBitCount = 0;
  mDataType = 0;
  mName = string("Unknown");
  mChannelGroup = NULL;
}

void MdfDataChannel::setup(const CNBLOCK* cn)
{
  TXBLOCK txBlock;
  char buffer[256] = {0};
  KvlcStatus status;

  mBitOffset  = cn->cn_bit_offset;
  mByteOffset = cn->cn_byte_offset;
  mBitCount   = cn->cn_bit_count;
  mDataType   = cn->cn_data_type;

  mFile->goto_block(cn->cn_tx_name);
  mFile->read_block(&txBlock, sizeof(txBlock) - sizeof(MDF_BYTE*));
  PRINTF_BLOCK(&txBlock.Head);
  status = mFile->read_file(buffer, (int) txBlock.Head.length);
  if (kvlcOK == status) {
    mName = string(buffer);
    PRINTF(("Added channel '%s'", mName.c_str()));
  }
  mNextChannel = NULL;

  if (KvaLogReader_Mdf4::mCheckSignals) {
    vector<string> names;
    split_string_to_vector(names, buffer);
    if (names.size() == 3 &&
        KvaLogReader_Mdf4::mDatabaseName.find(names[0]) != string::npos) {
      KvaDbMessageHnd mh;
      KvaDbSignalHnd sh;
      KvaDbStatus dbstat = kvaDbGetMsgByName(KvaLogReader_Mdf4::mDatabaseHnd, names[1].c_str(), &mh);
      if (dbstat != kvaDbOK) {
        printf("\nERROR: Could not find message '%s'\n", names[1].c_str());
        throw kvlcERR_INTERNAL_ERROR;
      }

      mChannelGroup->checkMessageProperties(mh);
      dbstat = kvaDbGetSignalByName(mh, toApi(names[2]), &sh);
      if (dbstat != kvaDbOK) {
        printf("\nERROR: Could not find signal '%s'\n", names[2].c_str());
        throw kvlcERR_INTERNAL_ERROR;
      }
      checkSignalProperties(sh);
    }
  }
}

MdfDataChannel::~MdfDataChannel ()
{
  PRINTF(("MdfDataChannel::~MdfDataChannel()"));
  if (mNextChannel) {
    delete mNextChannel;
  }
}

// See get_value_uint() in kvadblib for the proper way to do this
void MdfDataChannel::getData(void *data, void *buf, size_t len)
{
  char *p = (char*) data;
  if (mBitOffset != 0 || mBitCount % 8 != 0 || len < mBitCount / 8) {
    PRINTF(("\nERROR: getData() can only handle whole data bytes.\n"
            "       mBitOffset: %d mBitCount: %d mByteOffset:%d\n",
                    mBitOffset, mBitCount, mByteOffset));
    return;
  }
  memset(buf, 0, len);
  memcpy(buf, p + mByteOffset, mBitCount / 8);
}


static const char *DATA_FRAME_NAME = "CAN_DataFrame";
static const char *ERROR_FRAME_NAME = "CAN_ErrorFrame";
static const char *REMOTE_FRAME_NAME = "CAN_RemoteFrame";

void MdfChannelGroup::PRINTF_CG()
{
  switch(mType) {
    case DATA_FRAME:
      PRINTF(("%s ch:%d bytes:%u frames:%lu valid:%s",
        DATA_FRAME_NAME, mChannel, mDataBytes, mEventCount,
        mValid?"YES":"NO"));
      break;

    case ERROR_FRAME:
      PRINTF(("%s ch:%d bytes:%u frames:%lu valid:%s",
        ERROR_FRAME_NAME, mChannel, mDataBytes, mEventCount,
        mValid?"YES":"NO"));
      break;

    case REMOTE_FRAME:
      PRINTF(("%s ch:%d bytes:%u frames:%lu valid:%s",
        REMOTE_FRAME_NAME, mChannel, mDataBytes, mEventCount,
        mValid?"YES":"NO"));
      break;

    default:
      PRINTF(("\nERROR: Unknown frame type %d.\n", mType));
      break;
  }
}



MdfChannelGroup::MdfChannelGroup()
{
  mChannel = -1;
  mEventCount = 0LL;
  mValid = false;
  mMessageId = 0;
  memset(&mEvent, 0, sizeof(mEvent));
  mFile = NULL;
  mDataBytes = 0;
  mDataChannels = NULL;
  mCurrentDataChannel = NULL;
  mFileHandle = NULL;
  mSDBlock = NULL;
  mDTBlock = NULL;;
}

MdfChannelGroup::~MdfChannelGroup()
{
  if (mDataChannels) {
    delete mDataChannels;
  }
  if (mFileHandle) fclose(mFileHandle);
  if (mSDBlock) delete mSDBlock;
  if (mDTBlock) delete mDTBlock;

}

void MdfChannelGroup::setFrameType(const char *str)
{
  mType = MdfChannelGroup::DATA_FRAME;

  if (strncmp(ERROR_FRAME_NAME, str, 14) == 0) {
    mType = MdfChannelGroup::ERROR_FRAME;
  }
  if (strncmp(REMOTE_FRAME_NAME, str, 15) == 0) {
    mType = MdfChannelGroup::REMOTE_FRAME;
  }
  mFileName = string(str);

  if (mType == MdfChannelGroup::DATA_FRAME) {
    sscanf(mFileName.c_str(), "CAN_DataFrame.ID=%d", &mMessageId);
  }
}

void MdfChannelGroup::checkMessageProperties(KvaDbMessageHnd mh)
{
  unsigned int id = 0;
  unsigned int flags = 0;
  KvaDbStatus stat = kvaDbGetMsgId(mh, &id, &flags);
  if (stat != kvaDbOK) {
    printf("\nERROR: kvaDbGetMsgId failed: %d\n", stat);
    throw kvlcERR_INTERNAL_ERROR;
  }
  if (id != mMessageId) {
    printf("\nERROR: MDF4 has message id %u, but kvaDbGetMsgId returns %u.\n",
      mMessageId, id);
    throw kvlcERR_INTERNAL_ERROR;
  }
}


void MdfChannelGroup::addDataChannel(const CNBLOCK* cn)
{
  if (mDataChannels == NULL) {
    mDataChannels = new MdfDataChannel(mFile);
    mDataChannels->mChannelGroup = this;
    mDataChannels->setup(cn);
  }
  else {
    MdfDataChannel *ch = mDataChannels;
    while (ch->mNextChannel != NULL) {
      ch = ch->mNextChannel;
    }
    ch->mNextChannel = new MdfDataChannel(mFile);
    ch->mNextChannel->mChannelGroup = this;
    ch->mNextChannel->setup(cn);
  }
}

bool MdfChannelGroup::getChannelData(const char *name, void *data, void *buf, size_t len)
{
  MdfDataChannel *ch = mDataChannels;
  string str;
  bool found = false;
  // Time is the same for all frame types
  if (strcmp("t", name) != 0) {
    switch (mType) {
      case DATA_FRAME:
        str = string(DATA_FRAME_NAME);
        break;

      case ERROR_FRAME:
        str = string(ERROR_FRAME_NAME);
        break;

      case REMOTE_FRAME:
        str = string(REMOTE_FRAME_NAME);
        break;
    }
    str.append(".");
  }
  str.append(name);
  while (ch != NULL) {
    if (ch->mName == str) {
      ch->getData(data, buf, len);
      found = true;
      break;
    }
    ch = ch->mNextChannel;
  }
  return found;
}

bool MdfChannelGroup::getFirstSignalChannelData(void *data, string& name, int *value)
{
  mCurrentDataChannel = mDataChannels;

  while (mCurrentDataChannel != NULL) {
    if (mCurrentDataChannel->mName.find(KvaLogReader_Mdf4::mDatabaseName) != string::npos) {
      name = mCurrentDataChannel->mName;
      if (data && value) {
        mCurrentDataChannel->getData(data, value, sizeof(int));
      }
      return true;
    }
    mCurrentDataChannel = mCurrentDataChannel->mNextChannel;
  }
  return false;
}

bool MdfChannelGroup::getNextSignalChannelData(void *data, string& name, int *value)
{
  if (!mCurrentDataChannel) return false;
  mCurrentDataChannel = mCurrentDataChannel->mNextChannel;
  while (mCurrentDataChannel != NULL) {
    if (mCurrentDataChannel->mName.find(KvaLogReader_Mdf4::mDatabaseName) != string::npos) {
      name = mCurrentDataChannel->mName;
      if (data && value) {
        mCurrentDataChannel->getData(data, value, sizeof(int));
      }
      return true;
    }
    mCurrentDataChannel = mCurrentDataChannel->mNextChannel;
  }
  return false;
}

void MdfChannelGroup::writeSignalHeader()
{
  string name;
  if ( getFirstSignalChannelData(NULL, name, NULL) ) {
    //mFileHandle = fopen(mFileName.c_str(), "w");
	mFileHandle = utf_fopen(mFileName.c_str(), "w");
    if (!mFileHandle) {
      PRINTF(("ERROR: Could not open file '%s' for writing.", mFileName.c_str()));
      return;
    }
    fprintf(mFileHandle, "%s ", name.c_str());

    while ( getNextSignalChannelData(NULL, name, NULL) ) {
      fprintf(mFileHandle, "%s ", name.c_str());
    }
    fprintf(mFileHandle, "\n");
  }
}

void MdfChannelGroup::writeSignalData(void *data)
{
  string name;
  int value;

  if (!mFileHandle || !data) return;

  if ( getFirstSignalChannelData(data, name, &value) ) {
    fprintf(mFileHandle, "%d ", value);

    while ( getNextSignalChannelData(data, name, &value) ) {
      fprintf(mFileHandle, "%d ", value);
    }
    fprintf(mFileHandle, "\n");
  }
}

void MdfDataChannel::checkSignalProperties(KvaDbSignalHnd sh)
{
  int startbit;
  int length;
  KvaDbSignalEncoding encDb = kvaDb_Intel;
  KvaDbSignalEncoding encMdf = kvaDb_Intel;
  KvaDbSignalType typeDb = kvaDb_Invalid;
  KvaDbSignalType typeMdf = kvaDb_Invalid;
  KvaDbStatus stat = kvaDbGetSignalValueSize(sh, &startbit, &length);
  if (stat != kvaDbOK) {
    printf("\nERROR: kvaDbGetSignalValueSize failed: %d\n", stat);
    throw kvlcERR_INTERNAL_ERROR;
  }
  stat = kvaDbGetSignalEncoding(sh, &encDb);
  if (stat != kvaDbOK) {
    printf("\nERROR: kvaDbGetSignalEncoding failed: %d\n", stat);
    throw kvlcERR_INTERNAL_ERROR;
  }
  stat = kvaDbGetSignalRepresentationType(sh, &typeDb);
  if (stat != kvaDbOK) {
    printf("\nERROR: kvaDbGetSignalRepresentationType failed: %d\n", stat);
    throw kvlcERR_INTERNAL_ERROR;
  }
  toDbType(mDataType, &encMdf, &typeMdf);

  if (startbit % 8 != (int)mBitOffset) {
    printf("\nERROR: MDF4 has start bit %u, but kvaDbGetSignalValueSize returns %u.\n",
      mBitOffset, startbit % 8);
    throw kvlcERR_INTERNAL_ERROR;
  }
  if (startbit / 8 != (int)mByteOffset - 18) { // 18 bytes before data in MDF4
    printf("\nERROR: MDF4 has start byte %u, but kvaDbGetSignalValueSize returns %u.\n",
      mBitOffset - 18, startbit / 8);
    throw kvlcERR_INTERNAL_ERROR;
  }
  if (length != (int)mBitCount) {
    printf("\nERROR: MDF4 has start byte %u, but kvaDbGetSignalValueSize returns %u.\n",
      mBitOffset - 18, startbit / 8);
    throw kvlcERR_INTERNAL_ERROR;
  }
  if (encDb != encMdf) {
    printf("\nERROR: MDF4 has encoding %u, but kvaDbGetSignalEncoding returns %u.\n",
      encMdf, encDb);
    throw kvlcERR_INTERNAL_ERROR;
  }
  if (typeDb != typeMdf) {
    printf("\nERROR: MDF4 has type %u, but kvaDbGetSignalRepresentationType returns %u.\n",
      typeMdf, typeDb);
    throw kvlcERR_INTERNAL_ERROR;
  }
}


// Separate channels for data frame, error frame and remote frame
static MdfChannelGroup channelArray[MAX_CHANNELS * 3];

// =============================================================================
//  KvaLogReader_Mdf4
// =============================================================================
KvlcStatus KvaLogReader_Mdf4::interpret_event(imLogData *logEvent)
{
  switch (state) {
    case SEND_CLOCK:
    {
      memcpy(&mLogEvent, logEvent, sizeof(imLogData));
      PRINTF(("ILOG_TYPE_RTC T: %lu CAL: %lu\n", mLogEvent.common.time64, mStartDate));
      mUseSavedFrame = true;
      logEvent->common.new_data = true;
      logEvent->common.type = ILOG_TYPE_RTC;
      logEvent->common.time64 = mLogEvent.common.time64;
      logEvent->common.nanos_since_1970 = mStartDate;
      if (start_of_measurement64 == 0) {
        start_of_measurement64 = logEvent->common.nanos_since_1970;
        PRINTF(("Start of measurement read  = %ld", start_of_measurement64));
      }
      state = SEND_TRIGGER;
    }
    break;

    case SEND_TRIGGER:
    {
      PRINTF(("ILOG_TYPE_TRIGGER T: %lu\n", mLogEvent.common.time64));
      logEvent->common.type = ILOG_TYPE_TRIGGER;
      logEvent->common.new_data = true;
      logEvent->trig.type = 1;
      logEvent->trig.trigNo = 0;
      logEvent->trig.preTrigger = 0;
      logEvent->trig.postTrigger = -1;
      logEvent->common.time64 = mLogEvent.common.time64;
      logEvent->common.new_data = true;
      logEvent->trig.active = true;
      state = SEND_FRAMES;
    }
    break;

    case SEND_FRAMES:
    {
      if (mUseSavedFrame) {
        memcpy(logEvent, &mLogEvent, sizeof(imLogData));
        mUseSavedFrame = false;
      }
      PRINTF(("kvmLOG_TYPE_MSG T: %lu ID: %u CH: %d\n",
        logEvent->common.time64, logEvent->msg.id, logEvent->msg.channel));
    }
    break;
  }

  return kvlcOK;
}

/* Will return kvlcERR_UNSUPPORTED_VERSION if the MDF file is not from kvlclib. */
KvlcStatus KvaLogReader_Mdf4::verifyIdBlock(IDBLOCK *idBlock)
{
  MDF_CHAR    id_file[9] = {0};         // "MDF     "
  MDF_CHAR    id_vers[9] = {0};         // "4.10    "
  MDF_CHAR    id_prog[9] = {0};         // "Kvaser"

  strncpy(id_file, idBlock->id_file, 8);
  strncpy(id_vers, idBlock->id_vers, 8);
  strncpy(id_prog, idBlock->id_prog, 8);
  PRINTF(("'%s' '%s' '%s'\n", id_prog, id_file, id_vers));
  if (strncmp(id_file, MDF_FILE_ID, 8) ||
      strncmp(id_vers, MDF_VERSION_STRING_410, 8) ||
      strncmp(id_prog, MDF_PROGRAM_ID, 6)) {
    PRINTF(("Version strings don't match.\n"));
    return kvlcERR_UNSUPPORTED_VERSION;
  }

  // Get Kvaser's file version, e.g. Kvaser10 is major 1 and minor 0
  unsigned int major = 0;
  unsigned int minor = 0;


  if (strnlen(id_prog, 8) == 6) {
    // Old program id without version
    PRINTF(("Old version detected.\n"));
    major = 1;
    minor = 0;
  } else {
    PRINTF(("New version detected.\n"));
    // New program id [major][minor]
    if (isdigit (id_prog[6])) major =  id_prog[6] - '0';
    if (isdigit (id_prog[7])) minor =  id_prog[7] - '0';
  }

  PRINTF(("Kvaser version is %d.%d\n", major, minor));
  PRINTF(("Supported version is %d.%d\n", MDF_PROGRAM_ID_MAJOR, MDF_PROGRAM_ID_MINOR));

  // Unsupported major version
  if (major == 0 || major > MDF_PROGRAM_ID_MAJOR) {
    return kvlcERR_UNSUPPORTED_VERSION;
  }

  // Unsupported minor version
  if (major == MDF_PROGRAM_ID_MAJOR && minor > MDF_PROGRAM_ID_MINOR) {
    return kvlcERR_UNSUPPORTED_VERSION;
  }

  return kvlcOK;
}


void KvaLogReader_Mdf4::goto_block(MDF_LINK64 link)
{
  PRINTF(("goto_block(0x%lx)", link));
  if (os_fseek(infile, link, SEEK_SET) != 0) {
    throw(kvlcERR_FILE_ERROR);
  }
}

MDF_LINK64 KvaLogReader_Mdf4::get_link(void)
{
  int64_t pos = os_ftell(infile);
  if (pos == -1L) {
    throw(kvlcERR_FILE_ERROR);
  }
  return (MDF_LINK64) pos;
}

void KvaLogReader_Mdf4::read_block(void *block, size_t len)
{
  KvlcStatus status = read_file((char*) block, (int)len);
  if (kvlcOK != status) {
    throw status;
  }
}

void KvaLogReader_Mdf4::peek_block_head(void *block, size_t len)
{
  if (len != sizeof(BL_HEAD)) {
    throw(kvlcERR_INTERNAL_ERROR);
  }
  int64_t pos = os_ftell(infile);
  if (pos == -1L) {
    throw(kvlcERR_FILE_ERROR);
  }
  read_block(block, sizeof(BL_HEAD));
  if (os_fseek(infile, pos, SEEK_SET) != 0) {
    throw(kvlcERR_FILE_ERROR);
  }
}

static int getChannelNumber(const char *str)
{
  int ch = 0;
  char c;
  if (sscanf(str,"CAN%d%c", &ch, &c) != 1) {
    PRINTF(("getChannelNumber failed on '%s'. Setting channel = 1.", str));
    // throw(kvlcERR_INTERNAL_ERROR);

    // is used to store message names in MDF4 with signals,
    // at least when sorting on signals.
    ch = 1;
  }
  // Kvaser uses channel zero as first channel vs one in MDF
  ch = ch - 1;
  if ( ch < 0 || ch >= MAX_CHANNELS) {
    PRINTF(("getChannelNumber failed on '%s' => channel %d.", str, ch));
    throw(kvlcERR_FILE_ERROR);
  }
  return ch;
}

KvlcStatus KvaLogReader_Mdf4::get_attachment(void *atBlock) {
  FILE *h;
  MDF_UINT64 fileSize;
  size_t bytesLeft = 0;
  size_t bytesWritten = 0;
  TXBLOCK txBlock;
  char buffer[2048] = {0};
  MDF_BYTE    checksum[16];
  KvlcStatus status = kvlcOK;
  MDF_LINK64 data = 0;
  ATBLOCK* at = (ATBLOCK*) atBlock;
  std::string filename;
  char *p;

  data = get_link();
  goto_block(at->at_tx_filename);
  read_block(&txBlock, sizeof(txBlock) - sizeof(MDF_BYTE*));
  PRINTF_BLOCK(&txBlock.Head);
  status = read_file(buffer, (int) txBlock.Head.length);
  if (kvlcOK != status) {
    // Might just be EOF
    if (!feof(infile)) return status;
  }

  // Keep any database names
  p = strstr(buffer, ".dbc");
  if ( p ) {
    mDatabaseName = std::string(buffer, p - buffer + 1);
    PRINTF(("Database '%s'", mDatabaseName.c_str()));
  }

  // The filename in the MDF file may have paths, so remove those
  std::string basename(buffer);
  basename = basename.substr(basename.find_last_of("/\\") + 1);
  // Rename the file so we don't overwrite any original file
  filename = "kv_" + basename;

  PRINTF(("Open attachment file '%s'", filename.c_str()));
  //h = fopen(buffer, "wb" );
  h = utf_fopen(filename.c_str(), "wb");
  if (h == NULL) {
    PRINTF(("Could not open file '%s'", filename.c_str()));
    return kvlcERR_FILE_ERROR;
  }

  bytesLeft = (size_t) at->at_embedded_size;
  PRINTF(("Attachment size %lu", bytesLeft));
  goto_block(data);
  while (bytesLeft >= sizeof(buffer)) {
    status = read_file(buffer, sizeof(buffer));
    if (kvlcOK != status) return status;
    bytesLeft -= sizeof(buffer);
    bytesWritten = fwrite(buffer, 1, sizeof(buffer), h);
    PRINTF(("Wrote %lu bytes.", bytesWritten));
  }
  if (bytesLeft) {
    status = read_file(buffer, bytesLeft);
    if (kvlcOK != status) return status;
    bytesWritten = fwrite(buffer, 1, bytesLeft, h);
    if (bytesWritten) {
    PRINTF(("Wrote last %lu bytes.", bytesWritten));
  }
  }
  os_fseek(h, 0, SEEK_END);
  fileSize = os_ftell(h);
  fclose(h);

  PRINTF(("Filesize: %lu", fileSize));
  if (fileSize != at->at_embedded_size) {
    PRINTF(("\nERROR: Expected %lu bytes, but read %lu\n",
      at->at_embedded_size, fileSize));
      return kvlcERR_FILE_ERROR;
  }

  md5sum16(filename.c_str(), checksum, sizeof(checksum));
  for (unsigned int k = 0; k < sizeof(checksum); k++) {
    if (checksum[k] != at->at_md5_checksum[k]) {
      PRINTF(("\nERROR: MD5 checksum differs!"));
      return kvlcERR_FILE_ERROR;
    }
  }

  KvaDbStatus dbStatus = kvaDbOpen(&mDatabaseHnd);
  if (dbStatus != kvaDbOK) return kvlcERR_INTERNAL_ERROR;
  dbStatus = kvaDbCreate(mDatabaseHnd, mDatabaseName.substr(0, mDatabaseName.size()-1).c_str(), filename.c_str());
  if (dbStatus != kvaDbOK) return kvlcERR_INTERNAL_ERROR;
  return status;

}

KvlcStatus KvaLogReader_Mdf4::create_channels()
{
  KvlcStatus status = kvlcOK;
  MDF_LINK64 dg_dg_next = 0;
  MDF_LINK64 cn_cn_next = 0;
  MDF_LINK64 at_at_next = 0;
  MDF_LINK64 cn_time = 0;
  IDBLOCK idBlock;
  HDBLOCK hdBlock;
  DGBLOCK dgBlock;
  CGBLOCK cgBlock;
  CNBLOCK cnBlock;
  TXBLOCK txBlock;
  DTBLOCK dtBlock;
  HLBLOCK hlBlock;
  SDBLOCK sdBlock;
  SIBLOCK siBlock;
  ATBLOCK atBlock;
  BL_HEAD blHead;
  char buffer[256] = {0};

  MdfChannelGroup *pCh = NULL;

  try {
    // Read the IDBLOCK and verify MDF 4.x format
    read_block(&idBlock, sizeof(idBlock));
    status = verifyIdBlock(&idBlock);
    if (kvlcOK != status) {
      return status;
    }

    // Read the HDBLOCK
    read_block(&hdBlock, sizeof(hdBlock));
    PRINTF_BLOCK(&hdBlock.head);
    mStartDate = hdBlock.hd_start_time_ns;
    PRINTF(("hd_start_time_ns: %lu\n", mStartDate));

    // Extract any attachments
    at_at_next = hdBlock.hd_at_first;
    while (at_at_next) {
      goto_block(at_at_next);
      read_block(&atBlock, sizeof(atBlock) - sizeof(MDF_BYTE*));
      status = get_attachment(&atBlock);
      if (kvlcOK != status) {
        return status;
      }
      at_at_next = atBlock.at_at_next;
    }
    // Goto first data group
    dg_dg_next = hdBlock.hd_dg_first;
    while (dg_dg_next) {
      PRINTF(("dg_dg_next: 0x%lx", dg_dg_next));
      goto_block(dg_dg_next);
      read_block(&dgBlock, sizeof(dgBlock));
      PRINTF_BLOCK(&dgBlock.Head);
      dg_dg_next = dgBlock.dg_dg_next;
      PRINTF(("dg_dg_next: 0x%lx", dg_dg_next));

      // The channel groups
      pCh = &channelArray[mNextChannelIndex++];
      goto_block(dgBlock.dg_cg_first);
      read_block(&cgBlock, sizeof(cgBlock));
      PRINTF_BLOCK(&cgBlock.Head);

      goto_block(cgBlock.cg_tx_acq_name);
      read_block(&txBlock, sizeof(txBlock) - sizeof(MDF_BYTE*));
      PRINTF_BLOCK(&txBlock.Head);
      status = read_file(buffer, (int) txBlock.Head.length);
      if (kvlcOK != status) return status;
      pCh->mChannel = getChannelNumber(buffer);


      if (cgBlock.cg_si_acq_source) {
        goto_block(cgBlock.cg_si_acq_source);
        read_block(&siBlock, sizeof(siBlock));
        PRINTF_BLOCK(&siBlock.Head);
        if (siBlock.si_tx_path) {
          goto_block(siBlock.si_tx_path);
          read_block(&txBlock, sizeof(txBlock) - sizeof(MDF_BYTE*));
          PRINTF_BLOCK(&txBlock.Head);
          status = read_file(buffer, (int) txBlock.Head.length);
          if (kvlcOK != status) return status;
          pCh->setFrameType(buffer);
        }
      }

      pCh->mValid = true;
      pCh->mEventCount = cgBlock.cg_cycle_count;
      pCh->mDataBytes = cgBlock.cg_data_bytes;

      // Load data group
      goto_block(dgBlock.dg_data);

      // Check if it is a data block or block list
      peek_block_head(&blHead, sizeof(blHead));
      pCh->mDTBlock = new MdfBlockDT(pCh->mFile);
      if (memcmp(&blHead.id, MDF_ID_DTBLOCK, 4) == 0) {
        // DTBLOCK
        read_block(&dtBlock, sizeof(dtBlock));
        PRINTF_BLOCK(&dtBlock.Head);
        pCh->mDTBlock->mFilePos = get_link();
      } else {
        if (memcmp(&blHead.id, MDF_ID_HLBLOCK, 4) == 0) {
          // HLBLOCK
          read_block(&hlBlock, sizeof(hlBlock));
          PRINTF_BLOCK(&hlBlock.Head);
          pCh->mDTBlock->mUseZippedData = true;
          goto_block(hlBlock.hl_dl_first);
        }
        // DLBLOCK
        pCh->mDTBlock->readDataListBlock();
      }

      cn_cn_next = cgBlock.cg_cn_first;
      while (cn_cn_next) {
        goto_block(cn_cn_next);
        read_block(&cnBlock, sizeof(cnBlock));
        PRINTF_BLOCK(&cnBlock.Head);
        pCh->addDataChannel(&cnBlock);

        if (cnBlock.cn_data) {
          // For variable length data channel
          if ( pCh->isDataFrame() && cnBlock.cn_type == 1) {
            goto_block(cnBlock.cn_data);
            // Check if it is a data block or block list
            pCh->mSDBlock = new MdfBlockSD(pCh->mFile);
            peek_block_head(&blHead, sizeof(blHead));
            if (memcmp(&blHead.id, MDF_ID_SDBLOCK, 4) == 0) {
              // SDBLOCK
              read_block(&sdBlock, sizeof(sdBlock));
              PRINTF_BLOCK(&sdBlock.Head);
              pCh->mSDBlock->mFilePos = get_link();
            } else {
              if (memcmp(&blHead.id, MDF_ID_HLBLOCK, 4) == 0) {
              // HLBLOCK
              read_block(&hlBlock, sizeof(hlBlock));
              PRINTF_BLOCK(&hlBlock.Head);
              pCh->mSDBlock->mUseZippedData = true;
              goto_block(hlBlock.hl_dl_first);
              }
              // DLBLOCK
              pCh->mSDBlock->readDataListBlock();
            }
          }
          break;
        }

        if (cnBlock.cn_composition) {
          cn_cn_next = cnBlock.cn_composition;
          cn_time = cnBlock.cn_cn_next;
        } else {
          cn_cn_next = cnBlock.cn_cn_next;
        }
      }

      if (cn_time) {
        goto_block(cn_time);
        read_block(&cnBlock, sizeof(cnBlock));
        PRINTF_BLOCK(&cnBlock.Head);
        pCh->addDataChannel(&cnBlock);
      }

      // debug function that writes the header for any signals to CSV files
      // pCh->writeSignalHeader();

      // Get the first message for this channel group
      status = pCh->getEvent();
    }
  }
  catch (KvlcStatus err) {
    PRINTF(("KvaLogReader_Mdf4::create_channels() failed with status=%d\n", err));
    status = err;
  }

  return status;
}


KvlcStatus KvaLogReader_Mdf4::read_row(imLogData *logEvent)
{
  MdfChannelGroup *pCh = NULL;
  KvlcStatus status = kvlcOK;
  int idx = -1;
  KVMDF_TIMESTAMP firstTime = (KVMDF_TIMESTAMP) -1LL;

  if (!isOpened) {
    PRINTF(("KvaLogReader_Mdf4::read_row, file not open"));
    return kvlcERR_FILE_ERROR;
  }

  memset(logEvent, 0, sizeof(imLogData));
  logEvent->common.new_data = false;

  if (mFindChannels) {
    status = create_channels();
    mFindChannels = false;
    if (status != kvlcOK) return status;
  }

  if (!mUseSavedFrame) {
    for (unsigned int ch=0; ch < mNextChannelIndex; ch++) {
      pCh = &channelArray[ch];
      if (pCh->mValid) {
        if ( pCh->mEvent.common.time64 <= firstTime ) {
          firstTime = pCh->mEvent.common.time64;
          idx = ch;
        }
      }
    }

    if (idx < 0)  {
      PRINTF(("\nERROR: No CAN event found!\n"));

      // The previous call to getEvent() detected EOF.
      if (pCh != NULL && pCh->mEventCount == 0 && !pCh->mValid)
        return kvlcEOF;

      return kvlcERR_FILE_ERROR;
    }

    pCh = &channelArray[idx];
    memcpy(logEvent, &pCh->mEvent, sizeof(imLogData));
    logEvent->common.event_counter =  ++current_eventno;

    if (pCh->isDataFrame()) {
      logEvent->msg.frame_counter = ++current_frameno;
    }

    // Load next event from this channel
    // This could result in an EOF condition, but we can't return that
    // status here, so getEvent returns kvlcOK.  The next call to
    // read_row will detect this contition and return klvcEOF.
    status = pCh->getEvent();
    if (kvlcOK != status) {
      return status;
    }
  }

  return interpret_event(logEvent);
}

uint64 KvaLogReader_Mdf4::event_count()
{
  uint64 count = 0;
  for (unsigned int ch=0; ch < mNextChannelIndex; ch++) {
    MdfChannelGroup *pCh = &channelArray[ch];
    if (pCh->mValid) {
      count += pCh->mEventCount;
    }
  }
  return count;
}


KvlcStatus KvaLogReader_Mdf4::next_file()
{
  PRINTF(("KvaLogReader_Mdf4::next_file()"));
  // Reset states and search the new file for channels
  mFindChannels = true;
  mNextChannelIndex = 0;
  for (unsigned int ch=0; ch < MAX_CHANNELS*3; ch++) {
    channelArray[ch] = MdfChannelGroup(); //Using overloaded assign operator in MdfChannelGroup class
    channelArray[ch].mFile = this;
  }

  return kvlcOK;
}

KvlcStatus MdfChannelGroup::getEvent()
{
  char data[256] = {0};
  unsigned int dataLength = 0;
  KvlcStatus status = kvlcOK;
  bool channelFound;

  PRINTF(("KvlcStatus MdfChannelGroup::getEvent()"));
  PRINTF_CG();
  if (mEventCount == 0) {
    // Nothing more to read; channel not valid any more
    mValid = false;
    return status;
  }

  memset(&mEvent, 0, sizeof(mEvent));
  mEventCount--;

  status = mDTBlock->getData(data, mDataBytes);
  mEvent.common.type = ILOG_TYPE_MESSAGE;
  mEvent.common.new_data = true;
  mEvent.msg.channel = mChannel;
  getChannelData("t", &data, &mEvent.common.time64, sizeof(mEvent.common.time64));
  getChannelData("DLC", &data, &mEvent.msg.dlc, sizeof(mEvent.msg.dlc));
  getChannelData("ID", &data, &mEvent.msg.id, sizeof(mEvent.msg.id));
  getChannelData("DataLength", &data, &dataLength, sizeof(dataLength));


  channelFound = getChannelData("KvaserFlags", &data, &mEvent.msg.flags, sizeof(mEvent.msg.flags));
  if (!channelFound) {
    // Fallback to old version
    channelFound = getChannelData("Flags", &data, &mEvent.msg.flags, sizeof(mEvent.msg.flags));
  }

  if (!channelFound) {
    PRINTF(("Could not find 'Flags' or 'KvaserFlags'. This format is not supported.\n"));
    return kvlcERR_UNSUPPORTED_VERSION;
  }


  if (mSDBlock) {
    status = mSDBlock->getData(mEvent.msg.data, dataLength);
  } else {
    getChannelData("DataBytes", &data, mEvent.msg.data, sizeof(mEvent.msg.data));
  }

  // debug function that writes signal data to CSV files
  // writeSignalData(data);

  return status;
}

KvlcStatus KvaLogReader_Mdf4::verify_signals()
{
  KvlcStatus status = kvlcOK;

  try {
    if (!isOpened) {
      PRINTF(("KvaLogReader_Mdf4::read_row, file not open"));
      return kvlcERR_FILE_ERROR;
    }
    mCheckSignals = true;
    if (mFindChannels) {
      status = create_channels();
      mFindChannels = false;
      mCheckSignals = false;
      if (status != kvlcOK) return status;
    }
  } catch (KvlcStatus err) {
    PRINTF(("KvaLogReader_Mdf4::verify_signals() failed with status=%d\n", err));
    status = err;
  }
  return status;
}



KvaLogReader_Mdf4::KvaLogReader_Mdf4()
{
  PRINTF(("KvaLogReader_Mdf4::KvaLogReader_Mdf4()"));
  current_eventno = 0;
  current_frameno = 0;
  state = SEND_CLOCK;
  mUseSavedFrame = false;
  mFindChannels = true;
  mNextChannelIndex = 0;
  for (unsigned int ch=0; ch < MAX_CHANNELS*3; ch++) {
    channelArray[ch] = MdfChannelGroup();
    channelArray[ch].mFile = this;
  }
}

KvaLogReader_Mdf4::~KvaLogReader_Mdf4()
{
  PRINTF(("KvaLogReader_Mdf4::~KvaLogReader_Mdf4()"));
}

class KvaReaderMaker_Mdf4 : public KvaReaderMaker
{
  public:
    KvaReaderMaker_Mdf4() : KvaReaderMaker(KVLC_FILE_FORMAT_MDF_4X) {}
    int getName(char *str) { return sprintf(str, "%s", "MDF v4.1"); }
    int getExtension(char *str) { return sprintf(str, "%s", "mf4"); }
    int getDescription(char *str) { return sprintf(str, "%s", "CAN frames in MDF v4.1 for Vector CANalyzer"); }

  private:
    KvaLogReader *OnCreateReader() {
      return new KvaLogReader_Mdf4();
    }
}  registerKvaLogReader_Mdf4;


