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
**  Classes for writing Bosch MDF (Vector)
** ---------------------------------------------------------------------------
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stack>


#if defined(DEBUG) && defined(CRTMEMCHECK)
  #include <windows.h>
  #define CRTDBG_MAP_ALLOC
  #include <crtdbg.h>
#endif

#include "BufQueue.h"
#include "mdf3.h"
#include "os_util.h"
//#include "kvaconvertermisc.h"


#if defined(DEBUG)
  #include "kvdebug.h"
  // #define PRINTF(x) printf x
  #define FWRITE(x) fwrite x
  static size_t my_ftell(FILE *f, int line) {
   size_t pos = (int)os_ftell(f);
    PRINTF(("ftell@%d\t%#lx\n", line, pos));
    return pos;

  }
  #define FTELL(x) my_ftell(x,__LINE__)
#else
  #define PRINTF(x)
  #define FTELL(x) os_ftell(x)

  // When SIMULATION is defined, FWRITE is disabled, so is
  // one push to BQueue (inlined below).
  // #define SIMULATION
  #if defined(SIMULATION)
    #define FWRITE(x)
  #else
    #define FWRITE(x) fwrite x
  #endif

#endif

// "safe" addition
// Adds a to s as long as the addition doesn't wrap maxsize,
// calls "return (unsigned long)-1" otherwise, so do only use this in a function
// of the type "unsigned long"
#define ADD_UL(s,a,maxsize) do {                                                      \
  unsigned long tmp = a;                                                      \
  if (a == (unsigned long)-1 || s == (unsigned long)-1) {                     \
    PRINTF(("ADD_UL returns -1\n"));                                          \
    return (unsigned long)-1;                                                 \
  }                                                                           \
  if (maxsize - s < tmp) {                                          \
    PRINTF(("wrapped addition %lu + %lu @ %d in " __FILE__ "\n",                \
            s, tmp, __LINE__));                                               \
    return (unsigned long)-1;                                                 \
  } else {                                                                    \
    s+=tmp;                                                                   \
  }                                                                           \
} while(0)

// "safe" multiplication
// Multiplicates a and b and then add the result to s as long as neither the
// multiplication nor the addition wraps maxsize,
// calls "return (unsigned long)-1" otherwise, so do only use this in a function
// of the type "unsigned long"
#define MUL_ADD_UL(s,a,b,maxsize) do {                                        \
  if (a > maxsize/b) {                                                        \
    PRINTF(("a > maxsize / b, %lu, %d @ %d in " __FILE__ "\n",                 \
            a, b, __LINE__));                                                 \
    return -1;                                                                \
  }                                                                           \
  ADD_UL(s, b * a,maxsize);                                                   \
} while(0)

using namespace Mdf3Nodes;

/*******************************************************************************
* Mdf3
*******************************************************************************/
Mdf3::Mdf3()
{
  PRINTF(("Mdf3::Mdf3()"));

  mdfFile = NULL;

  id = NULL;
  hd = NULL;

  version = 0;

  curByteOrder = 0xffff; // undefined
}

Mdf3::~Mdf3()
{
  close();

  PRINTF(("Mdf3::size = %lu\n", size()));

  if (id) {
    delete id;
  }

  if (hd) {
    delete hd;
  }
}

unsigned long Mdf3::maxSize(int ver) {
  if (ver >= 320) {
    return (unsigned long)0xffffffff; // 4GB-1
  }
  else {
    return (unsigned long)0x7fffffff; // 2GB-1
  }
}

unsigned long Mdf3::maxSize() {
  return maxSize(version);
}


int Mdf3::create(const char *filename)
{
  mdfFile = fopen(filename, "wb");
  //mdfFile = utf_fopen(filename, "wb");
  if (!mdfFile) {
    PRINTF(("ERROR: Unable to open '%s'\n", filename));
    return -1;
  }

  return 0;
}

void Mdf3::create(FILE *fh)
{
  mdfFile = fh;
}

int Mdf3::write_header()
{
  if (mdfFile && id && hd) {
    os_fseek(mdfFile, 0, SEEK_SET);
    id->write(mdfFile);
    hd->write(mdfFile);
    return 0;
  }
  return -1;
}

int Mdf3::write_data() {
  if (hd) {
    return hd->write_data(mdfFile);
  }
  return -1;
}

int Mdf3::close()
{
  if (mdfFile) {
    fflush(mdfFile);
    fclose(mdfFile);
    mdfFile = NULL;
    return 0;
  }
  return -1;
}

unsigned long Mdf3::size()
{
  if (hd && id) {
    unsigned long s = 0;
    unsigned long hdsize;
    unsigned long idsize;
    hdsize = hd->size();
    idsize = id->size();
    if (hdsize == (unsigned long)-1 || idsize == (unsigned long)-1) {
      return (unsigned long)-1;
    }
    ADD_UL(s, idsize, maxSize());
    ADD_UL(s, hdsize, maxSize());
    return s;
  }
  else {
    return 0;
  }
}

int Mdf3::initAndSetVersion(int version)
{
  if (0xffff == curByteOrder) {
    PRINTF(("Mdf3::initAndSetVersion(%d)", version));
    this->version = version;
    id = new idNode(version);
    hd = new hdNode(version);
    if (id && hd) {
      return MDF_OK;
    }
    return MDF_ERROR_MEMORY;
  }
  return MDF_ERROR_INTERNAL;
}

MdfStatus Mdf3::new_dg(MDF_UINT32 canId, MDF_UINT8 dlc)
{
  return Mdf3::new_dg(canId, CAN_ID_MASK_ALL_BITS, MuxChecker(0,-1), dlc);
}

MdfStatus Mdf3::new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc)
{
  if (hd) {
    return hd->new_dg(canId, canMask, mux, dlc);
  }
  return MDF_ERROR_MEMORY;
}

MdfStatus Mdf3::new_sig(MDF_UINT32 canId,
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
  if (id && version < 310) {

    // New signal is BE
    if (SigDataType >= MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_BE
       && SigDataType <= MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_BE)
    {
      // Fail when curByteOrder is LE
      if (curByteOrder == MDF_LITTLE_ENDIAN) {
        // printf("sigdatatype = %d,curByteOrder = %d, line = %d\n", SigDataType, curByteOrder, __LINE__);
        return MDF_ERROR_DATATYPE;
      }
      curByteOrder = MDF_BIG_ENDIAN;
      id->setEndianess(MDF_BIG_ENDIAN);
      // Convert from new datatype to old datatype
      SigDataType = SigDataType - MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_BE;
    }
    else
    // New signal is LE
      if ((SigDataType >= MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE
          && SigDataType <= MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE_LE) ||
          (SigDataType <= MDF_SIGNAL_DATA_TYPE_IEEE_DOUBLE))
    {
      if (curByteOrder == MDF_BIG_ENDIAN) {
        // printf("sigdatatype = %d,curByteOrder = %d, line = %d\n", SigDataType, curByteOrder, __LINE__);
        return MDF_ERROR_DATATYPE;
      }
      curByteOrder = MDF_LITTLE_ENDIAN;

      // If need be, convert from new datatype to old datatype
      if (SigDataType >= MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE) {
        SigDataType = SigDataType - MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE;
      }
    }
    else {
      // Don't care about byte order (array && string)
      // printf("sigdatatype = %d,curByteOrder = %d, line = %d\n", SigDataType, curByteOrder, __LINE__);
    }
  }
  if (hd) {
    return hd->new_sig( canId, longname, shortname, unit, StartOffset, NumBits,
                        SigDataType, ChannelConversionType, Factor, Offset);
  }
  return MDF_ERROR_MEMORY;
}

int Mdf3::setStartOfRecording(
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
  return -1;
}

int Mdf3::addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data)
{
  if (hd) {
    return hd->addMsg(id, ts, data);
  }
  return -1;
}
namespace Mdf3Nodes {
/*******************************************************************************
* txNode - Node for TXBLOCK
*******************************************************************************/
txNode::txNode(int version, char *str)
{
  setVersion(version);
  size_t len = strlen(str);

  memset(&tx, 0, sizeof(TXBLOCK));

  tx.Head.BlockTypeId = MDF_ID_TXBLOCK;
  tx.Head.BlockSize = (MDF_UINT16)(sizeof(BL_HEAD) + len + 1);
  tx.Text = new MDF_CHAR[len+1];
  memcpy(tx.Text, str, len+1);
}

txNode::~txNode()
{
  PRINTF(("~txNode\n"));
  if (tx.Text) {
    delete tx.Text;
  }
}

int txNode::write(FILE *mdfFile)
{
  setCurrentFilePos((int)FTELL(mdfFile));
  if (tx.Head.BlockSize) {
    FWRITE((&tx, 1, sizeof(BL_HEAD), mdfFile));
    FWRITE((tx.Text, 1, tx.Head.BlockSize - sizeof(BL_HEAD), mdfFile));
  }
  return 0;
}

unsigned long txNode::size()
{
  unsigned long s = tx.Head.BlockSize;
  // PRINTF(("txNode.size = %lu\n", s));
  return s;
}

/*******************************************************************************
* prNode - Node for PRBLOCK
*******************************************************************************/
prNode::prNode(int version, char *str)
{
  setVersion(version);
  size_t len = strlen(str);

  memset(&pr, 0, sizeof(PRBLOCK));

  pr.Head.BlockTypeId = MDF_ID_PRBLOCK;
  pr.Head.BlockSize = (MDF_UINT16)(sizeof(BL_HEAD) + len + 1);
  pr.Text = new MDF_CHAR[len+1];
  memcpy(pr.Text, str, len+1);
}

prNode::~prNode()
{
  PRINTF(("~prNode\n"));
  if (pr.Text) {
    delete pr.Text;
  }
}

int prNode::write(FILE *mdfFile)
{
  setCurrentFilePos((int)FTELL(mdfFile));
  if (pr.Head.BlockSize) {
    FWRITE((&pr, 1, pr.Head.BlockSize, mdfFile));
  }
  return 0;
}

unsigned long prNode::size()
{
  unsigned long s = pr.Head.BlockSize;
  PRINTF(("prNode.size = %lu\n", s));
  return s;
}

/*******************************************************************************
* cnNode - Node for CNBLOCK
*******************************************************************************/
cnNode::~cnNode()
{
  if (tx_comment) {
    delete tx_comment;
  }
  if (tx_long_signal_name) {
    delete tx_long_signal_name;
  }
  if (cc) {
    delete cc;
  }
  if (next) {
    delete next;
  }
}

int cnNode::write(FILE *mdfFile)
{
  setCurrentFilePos((int)FTELL(mdfFile));
  if (next) {
    cn.NextChannelBlock = (MDF_LINK)next->getCurrentFilePos();
  }
  if (tx_comment) {
    cn.Comment = (MDF_LINK)tx_comment->getCurrentFilePos();
  }
  if (tx_long_signal_name && getVersion() >= 300) {
    cn.TX_LongSignalName = (MDF_LINK)tx_long_signal_name->getCurrentFilePos();
  }
  if (cc) {
    cn.ConversionFormula = (MDF_LINK)cc->getCurrentFilePos();
  }
  FWRITE((&cn, 1, cn.Head.BlockSize, mdfFile));
  if (next) {
    next->write(mdfFile);
  }
  if (tx_comment) {
    tx_comment->write(mdfFile);
  }
  if (tx_long_signal_name) {
    tx_long_signal_name->write(mdfFile);
  }
  if (cc) {
    cc->write(mdfFile);
  }

  return -1;
}

unsigned long cnNode::size()
{
  if (isSizeDirty()) {
    unsigned long s = cn.Head.BlockSize;
    if (next) {
      ADD_UL(s,next->size(), Mdf3::maxSize(getVersion()));
    }
    if (tx_comment) {
      ADD_UL(s, tx_comment->size(), Mdf3::maxSize(getVersion()));
    }
    if (tx_long_signal_name) {
      ADD_UL(s, tx_long_signal_name->size(), Mdf3::maxSize(getVersion()));
    }
    if (cc) {
      ADD_UL(s, cc->size(), Mdf3::maxSize(getVersion()));
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }

}

cnNode::cnNode(
          int version,
          const char *long_name,
          const char *name,
          const char *unit,
          MDF_UINT16 StartOffset,
          MDF_UINT16 NumBits,
          MDF_UINT16 SigDataType,
          MDF_UINT16 ChannelConversionType,
          MDF_REAL Factor,
          MDF_REAL Offset)
{
  setVersion(version);

  memset(&cn, 0, sizeof(CNBLOCK));
  cn.Head.BlockTypeId = MDF_ID_CNBLOCK;

  if (getVersion() >= 300) {
    cn.Head.BlockSize = sizeof(CNBLOCK);
  }
  else {
    cn.Head.BlockSize = sizeof(CNBLOCK)
                      - sizeof(MDF_LINK)    // TX_LongSignalName
                      - sizeof(MDF_LINK)    // TX_DisplayName
                      - sizeof(MDF_UINT16); // AdditionalByteOffset
  }

  tx_comment = NULL;
  // tx_long_signal_name = new txNode(long_name);
  tx_long_signal_name = NULL;
  cc = new ccNode(version, unit, ChannelConversionType, Offset, Factor);
  next = NULL;
  strncpy(cn.ShortSignalName, name, sizeof(cn.ShortSignalName) - 1);
  strncpy(cn.SignalDescription, long_name, sizeof(cn.SignalDescription) - 1);
  cn.ChannelType = MDF_CHANNEL_TYPE_DATA; // Defaults to data
  cn.ValidRangeFlag = MDF_FALSE;
  cn.StartOffsetInBits = StartOffset;
  cn.NumberOfBits = NumBits;
  cn.SignalDataType = SigDataType;
}

/*******************************************************************************
* cgNode - Node for CGBLOCK
*******************************************************************************/
cgNode::cgNode(int version, char *cg_comment)
{
  setVersion(version);
  memset(&cg, 0, sizeof(CGBLOCK));
  cg.Head.BlockTypeId = MDF_ID_CGBLOCK;
  cg.Head.BlockSize = sizeof(CGBLOCK);
  cn = NULL;
  next = NULL;
  if (cg_comment) {
    comment = new txNode(version, cg_comment);
  }
  else {
    comment = NULL;
  }
}

cgNode::~cgNode()
{
  if (comment) {
    delete comment;
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
  setCurrentFilePos((int)FTELL(mdfFile));

  if (next) {
    cg.NextChannelGroup = (MDF_LINK)next->getCurrentFilePos();
  }
  if (cn) {
    PRINTF(("cgNode::write, fcb = %lu", cn->getCurrentFilePos()));
    cg.FirstChannelBlock = (MDF_LINK)cn->getCurrentFilePos();
  }
  if (comment) {
    cg.Comment = (MDF_LINK)comment->getCurrentFilePos();
  }
  FWRITE((&cg, 1, cg.Head.BlockSize, mdfFile));
  if (next) {
    next->write(mdfFile);
  }
  if (cn) {
    cn->write(mdfFile);
  }
  if (comment) {
    comment->write(mdfFile);
  }


  return -1;
}

unsigned long cgNode::size()
{
  if (isSizeDirty()) {
    unsigned long s = cg.Head.BlockSize;
    if (next) {
      ADD_UL(s, next->size(), Mdf3::maxSize(getVersion()));
    }
    if (cn) {
      ADD_UL(s, cn->size(), Mdf3::maxSize(getVersion()));
    }
    if (comment) {
      ADD_UL(s, comment->size(), Mdf3::maxSize(getVersion()));
    }
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

MdfStatus cgNode::new_cn(
                  const char *long_name,
                  const char *name,
                  const char *unit,
                  MDF_UINT16 StartOffset,
                  MDF_UINT16 NumBits,
                  MDF_UINT16 SigDataType,
                  MDF_UINT16 ChannelConversionType,
                  MDF_REAL Factor,
                  MDF_REAL Offset)
{
  cnNode *local_cn;

  if (getVersion() < 215 && SigDataType >= 4)
  {
    // 4-6 are obsolete VAX datatypes
    // 7-8 are String and Byte Array
    return MDF_ERROR_DATATYPE;
  }
  if (getVersion() < 310 && SigDataType >= 9)
  {
    // 9-16 are int&floats with different endianess
    return MDF_ERROR_DATATYPE;
  }

  local_cn = cn;
  // Insert new cg at first place
  cn = new cnNode(getVersion(),
                  long_name,
                  name,
                  unit,
                  StartOffset,
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
  cg.NumberOfChannels++;
  setSizeDirty();
  return MDF_OK;
}

void cgNode::increaseNumberOfMessages(int num)
{
  cg.NumberOfRecords+=num;
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
  tr = NULL;
  dataRecords = 0;
  dg.Head.BlockTypeId = MDF_ID_DGBLOCK;
  dg.Head.BlockSize = sizeof(DGBLOCK);
  bq = NULL;
  record = NULL;
  RecordSize = 0;
  mux_checker = MuxChecker(0,-2);
}

dgNode::~dgNode()
{
  PRINTF(("~dgNode: next = %p, cg = %p, tr = %p\n", next, cg, tr));
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
  if (tr) {
    delete tr;
  }
  if (bq) {
    delete bq;
  }
  if (record) {
    delete record;
  }
  PRINTF(("~dgNode (%p)\n", this));
}

int dgNode::write(FILE *mdfFile)
{
  setCurrentFilePos((int)FTELL(mdfFile));
  if (next) {
    dg.NextDataGroupBlock = (MDF_LINK)next->getCurrentFilePos();
  }
  if (cg) {
    dg.FirstChannelGroup = (MDF_LINK)cg->getCurrentFilePos();
  }
  if (tr) {
    dg.TriggerBlock = (MDF_LINK)tr->getCurrentFilePos();
  }
  FWRITE((&dg, 1, dg.Head.BlockSize, mdfFile));
  if (next) {
    next->write(mdfFile);
  }
  if (cg) {
    cg->write(mdfFile);
  }
  if (tr) {
    tr->write(mdfFile);
  }
  return 0;
}

unsigned long dgNode::size()
{
  if (isSizeDirty()) {
    unsigned long s = dg.Head.BlockSize;
    if (next) {
      ADD_UL(s, next->size(), Mdf3::maxSize(getVersion()));
    }
    if (cg) {
      ADD_UL(s, cg->size(), Mdf3::maxSize(getVersion()));
    }
    if (tr) {
      ADD_UL(s, tr->size(), Mdf3::maxSize(getVersion()));
    }

    MUL_ADD_UL(s, dataRecords, RecordSize, Mdf3::maxSize(getVersion()));

    // PRINTF(("dgNode.size = %lu, RecordSize = %d, dataRecords = %d\n", s, RecordSize, dataRecords));
    setSize(s);
    return s;
    }
  else {
    return getLastSize();
  }
}

MdfStatus dgNode::new_cg(MDF_UINT8 dlc)
{
  cgNode *local_cg;
  MdfStatus stat = MDF_OK;

  local_cg = cg;
  // Insert new cg at first place
  cg = new cgNode(getVersion());
  PRINTF(("new_cg (%p)\n", cg));

  if(!cg) {
    PRINTF(("new_cg failed, unable to create new cgNode\n"));
    return MDF_ERROR_MEMORY;
  }
  // Reinsert previously first cg at second place
  cg->next = local_cg;


  cg->cg.SizeOfDataRecord = dlc + sizeof(KVMDF_TIMESTAMP);

  // Add time channel (after the data)
  stat = cg->new_cn("Timestamp",
             "t",
             "s",
             dlc * 8,
             sizeof(KVMDF_TIMESTAMP)*8,
             MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT,
             MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR,
             0.00001,
             0.0
             );
  cg->cn->cn.ChannelType = MDF_CHANNEL_TYPE_TIME;

  dg.NumberOfChannelGroups++;
  setSizeDirty();
  return stat;
}

int dgNode::addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data)
{

  int ret_code = -1;
  if (((canId & canMask) == (id & canMask)) && mux_checker.inEvent(data, RecordSize - sizeof(KVMDF_TIMESTAMP)) ) {
    //data group of either independent signals or signals with relevant multiplexed value
    memcpy(record, data, RecordSize - sizeof(KVMDF_TIMESTAMP));
    memcpy(&record[RecordSize-sizeof(KVMDF_TIMESTAMP)], &ts, sizeof(KVMDF_TIMESTAMP));

    if (cg) {
      cg->increaseNumberOfMessages();
    }

    if (bq) {
      #if !defined(SIMULATION)
        bq->push(record);
      #endif
      dataRecords++;
    }
    else {
      PRINTF(("BQueue is not allocated!!!\n"));
      return -1;
    }
    setSizeDirty();
    ret_code = 0;
  }
  if (next) {
    setSizeDirty();
    return next->addMsg(id, ts, data);
  }
  return ret_code;
}

MdfStatus dgNode::new_sig(MDF_UINT32 id, char *longname, char *shortname, char *unit, MDF_UINT16 StartOffset, MDF_UINT16 NumBits, MDF_UINT16 SigDataType, MDF_UINT16 ChannelConversionType, MDF_REAL Factor, MDF_REAL Offset)
{
  if (canId == id) {
    setSizeDirty();
    return cg->new_cn(longname, shortname, unit, StartOffset, NumBits, SigDataType, ChannelConversionType, Factor, Offset);
  }
  if (next) {
    setSizeDirty();
    return next->new_sig(canId, longname, shortname, unit, StartOffset, NumBits, SigDataType, ChannelConversionType, Factor, Offset);
  }
  return MDF_ERROR_MEMORY;
}

MdfStatus dgNode::setRecordSize(MDF_UINT8 recordSize)
{
  RecordSize = recordSize;
  record = new char[recordSize];
  if (record) {
    return MDF_OK;
  }
  return MDF_ERROR_MEMORY;
}

int dgNode::write_data(FILE *mdfFile)
{

  dg.DataBlock = (MDF_LINK)FTELL(mdfFile);
  PRINTF(("write_data(%p)\n", this));
  PRINTF(("  dg.DataBlock     = %#x\n", dg.DataBlock));
  PRINTF(("  bq->size()       = %d\n", bq->size()));
  PRINTF(("  RecordSize       = %d\n", RecordSize));

  while (bq->size()) {
    bq->pop(record);
    FWRITE((record, RecordSize, 1, mdfFile));
  }

  if (next) {
    return next->write_data(mdfFile);
  }
  return 0;
}

/*******************************************************************************
* hdNode - Node for HDBLOCK
*******************************************************************************/
hdNode::hdNode(int version)
{
  setVersion(version);
  memset(&hd, 0, sizeof(HDBLOCK));
  tx = NULL; // new txNode("Hello world");
  pr = NULL;
  dg = NULL;
  cur_dg = NULL;

  hd.Head.BlockTypeId = MDF_ID_HDBLOCK;
  if (getVersion() >= 320) {
    hd.Head.BlockSize =   sizeof(HDBLOCK);
  }
  else {
    hd.Head.BlockSize =   sizeof(HDBLOCK)
                        - sizeof(MDF_UINT64)    // TimeStamp
                        - sizeof(MDF_INT16)     // UtcTimeOffset
                        - sizeof(MDF_UINT16)    // TimeQualityClass
                        - 32* sizeof(MDF_CHAR); // TimerIdentification
  }
  hd.FirstDataGroupBlock = 0;
  hd.MeasurementFileComment = 0;
  hd.ProgramBlock = 0;

  hd.NumberOfDataGroups = 0;
  memcpy(hd.Date, "01:01:1970", sizeof(hd.Date));
  memcpy(hd.Time, "00:00:00", sizeof(hd.Time));
  hd.Author[0] = '\0';
  hd.Organization[0] = '\0';
  hd.Project[0] = '\0';
  hd.Subject[0] = '\0';

  // The following data was added since 3.20 and only their default is noted here
  hd.TimeStamp = 0;
  hd.UtcTimeOffset = 0;
  hd.TimeQualityClass = 0;
  memset(hd.TimerIdentification, '\0', sizeof(hd.TimerIdentification));
}

hdNode::~hdNode()
{
  PRINTF(("~hdNode: dg = %p, tx = %p, pr = %p\n", dg, tx, pr));
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
  if (tx) {
    delete tx;
  }
  if (pr) {
    delete pr;
  }
  PRINTF(("~hdNode (%p)\n", this));
}

int hdNode::write(FILE *mdfFile)
{
  if (dg) {
    hd.FirstDataGroupBlock = (MDF_LINK)dg->getCurrentFilePos();
  }
  if (tx) {
    hd.MeasurementFileComment = (MDF_LINK)tx->getCurrentFilePos();
  }
  if (pr) {
    hd.ProgramBlock = (MDF_LINK)pr->getCurrentFilePos();
  }

  FWRITE((&hd, hd.Head.BlockSize, 1, mdfFile));

  if(dg) {
    dg->write(mdfFile);
  }
  if (tx) {
    tx->write(mdfFile);
  }
  if (pr) {
    pr->write(mdfFile);
  }


  return 0;
}

unsigned long hdNode::size()
{
  if (isSizeDirty()) {
   unsigned long s = hd.Head.BlockSize;
    if (dg) {
      ADD_UL(s, dg->size(), Mdf3::maxSize(getVersion()));
    }
    if (tx) {
      ADD_UL(s, tx->size(), Mdf3::maxSize(getVersion()));
    }
    if (pr) {
      ADD_UL(s, pr->size(), Mdf3::maxSize(getVersion()));
    }
    // PRINTF(("hdNode.size = %lu\n", s));
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

MdfStatus hdNode::new_dg(MDF_UINT32 canId, MDF_UINT32 canMask, const MuxChecker& mux, MDF_UINT8 dlc)
{
  dgNode *local_dg;
  char   prefix[32];
  MdfStatus stat;
  local_dg = dg;
  while (dg) {
    if ((dg->canId == canId)&&(dg->mux_checker.getValue() == mux.getValue())) {
      cur_dg = dg;
      dg = local_dg;
      return MDF_OK; //dgblock for these kind of signals is already created
    }
    dg = dg->next;
  }
  // Insert new dg at first place
  dg = new dgNode(getVersion());
  if (!dg) {
    PRINTF(("new_dg failed, unable to create new dgNode\n"));
    return MDF_ERROR_MEMORY;
  }
  // Reinsert previously first dg at second place
  dg->next = local_dg;
  dg->canId = canId;
  dg->canMask = canMask;
  dg->mux_checker=MuxChecker(mux);
  PRINTF(("Added dg_node with mux value:%d",dg->mux_checker.getValue()));
  stat = dg->new_cg(dlc);
  if (stat != MDF_OK) {
    return stat;
  }

  sprintf(prefix, "MDF_%08x_%08x", canId, dg->mux_checker.getValue());
  dg->setRecordSize(sizeof(KVMDF_TIMESTAMP) + dlc);

  dg->bq = new BQueue(prefix, 100000, dg->RecordSize);
  if (!dg->bq) {
    return MDF_ERROR_MEMORY;
  }
  hd.NumberOfDataGroups++;
  cur_dg = dg;
  setSizeDirty();
  return MDF_OK;
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
  MDF_CHAR date[sizeof(hd.Date)+1];
  MDF_CHAR time[sizeof(hd.Time)+1];

  // DD:MM:YYYY without null termination
  sprintf(date, "%02u:%02u:%04u", day % 100U, month % 100U, year % 10000U);
  memcpy(hd.Date, date, sizeof(hd.Date)); // remove '\0' in the end

  // HH:MM:SS without null termination
  sprintf(time, "%02u:%02u:%02u", hour % 100U, minute % 100U, second % 100U);
  memcpy(hd.Time, time, sizeof(hd.Time)); // remove '\0' in the end

  return 0;
}

int hdNode::addMsg(MDF_UINT32 id, KVMDF_TIMESTAMP ts, MDF_UINT8 *data)
{
  if (dg) {
    setSizeDirty();
    return dg->addMsg(id, ts, data);
  }
  return -1;
}

int hdNode::write_data(FILE *mdfFile)
{
  if (dg) {
    return dg->write_data(mdfFile);
  }
  return -1;
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

/*******************************************************************************
* ccNode - Node for CCBLOCK
*******************************************************************************/
ccNode::ccNode(int version, const char *unit, MDF_UINT16 type, MDF_REAL p1, MDF_REAL p2)
{
  setVersion(version);
  memset(&cc, 0, sizeof(CCBLOCK));

  strncpy(cc.PhysicalUnit, unit, sizeof(cc.PhysicalUnit) - 1);

  cc.Head.BlockTypeId = MDF_ID_CCBLOCK;
  if (type == MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR) {
    cc.Head.BlockSize = sizeof(CCBLOCK);
    // phys = int * p2 + p1
    cc.P1 = p1; // offset
    cc.P2 = p2; // factor
    cc.SizeInformation = 2; // number of parameters
  }
  else if (type == MDF_CONVERSION_TYPE_ONE_TO_ONE) {
    cc.Head.BlockSize = sizeof(CCBLOCK) - 2*sizeof(MDF_REAL);
    cc.SizeInformation = 0;
  }
  else {
    PRINTF(("ccNode:: unsupported type %d\n", type));
    cc.Head.BlockSize = sizeof(CCBLOCK) - 2*sizeof(MDF_REAL);
    cc.SizeInformation = 0;
  }
  cc.ConversionType = type;
}

ccNode::~ccNode()
{

}

int ccNode::write(FILE *mdfFile)
{
  setCurrentFilePos((int)FTELL(mdfFile));
  FWRITE((&cc, cc.Head.BlockSize, 1, mdfFile));
  return 0;
}

unsigned long ccNode::size()
{
 unsigned long s = cc.Head.BlockSize;
 return s;
}

/*******************************************************************************
* trNode - Node for TRBLOCK
*******************************************************************************/

trNode::trNode(int version, int number_of_trigger_events, char *trigger_comment)
{
  setVersion(version);
  memset(&tr, 0, sizeof(TRBLOCK));
  tr.Head.BlockTypeId = MDF_ID_TRBLOCK;
  tr.Head.BlockSize = (MDF_UINT16)(sizeof(TRBLOCK) - sizeof(TREVENT*)
                    + sizeof(TREVENT) * number_of_trigger_events);
  if (trigger_comment) {
    comment = new txNode(version, trigger_comment);
  }
  else {
    comment = NULL;
  }
  tr.NumberOfTriggerEvents = number_of_trigger_events;
  tr.TriggerEvent = new TREVENT[number_of_trigger_events];
}

trNode::~trNode()
{
  PRINTF(("~trNode\n"));
  if (tr.TriggerEvent) {
    delete tr.TriggerEvent;
  }
  if (comment) {
    delete comment;
  }
}

int trNode::write(FILE *mdfFile)
{
  setCurrentFilePos((int)FTELL(mdfFile));
  if (comment) {
    tr.TriggerComment = (MDF_LINK)comment->getCurrentFilePos();
  }

  if (tr.Head.BlockSize) {
    FWRITE((&tr, 1, tr.Head.BlockSize - sizeof(TREVENT*), mdfFile));
    FWRITE((tr.TriggerEvent, tr.NumberOfTriggerEvents, sizeof(TREVENT), mdfFile));
  }

  if (comment) {
    comment->write(mdfFile);
  }

  return 0;
}

unsigned long trNode::size()
{
  if (isSizeDirty()) {
    unsigned long s = tr.Head.BlockSize;
    if (comment) {
      ADD_UL(s, comment->size(), Mdf3::maxSize(getVersion()));
    }
    PRINTF(("trNode.size = %lu\n", s));
    setSize(s);
    return s;
  }
  else {
    return getLastSize();
  }
}

/*******************************************************************************
* idNode - Node for IDBLOCK
*******************************************************************************/
idNode::idNode(int version)
{
  setVersion(version);
  memset(&id, 0, sizeof(IDBLOCK));
  memcpy(id.FileIdentifier,   MDF_FILE_ID, sizeof(id.FileIdentifier));

  if (version == 320) {
    memcpy(id.FormatIdentifier, MDF_VERSION_STRING_320, sizeof(id.FormatIdentifier));
  }
  else if (version == 203) {
    memcpy(id.FormatIdentifier, MDF_VERSION_STRING_203, sizeof(id.FormatIdentifier));
  }
  else {
    char tmp[10];
    sprintf(tmp, "%1d.%2d     ", (version / 100) % 10U, version % 100U);
    memcpy(id.FormatIdentifier, tmp, sizeof(id.FormatIdentifier));
    PRINTF(("Will use '%s' as version identifier string, only tested for 2.03 and 3.20 though\n", id.FormatIdentifier));
  }

  memcpy(id.ProgramIdentifier,MDF_PROGRAM_ID, sizeof(id.ProgramIdentifier));
  id.DefaultByteOrder = MDF_LITTLE_ENDIAN;
  id.DefaultFloatingPointFormat = 0;
  id.VersionNumber = version;
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

unsigned long idNode::size()
{
  unsigned long s = sizeof(IDBLOCK);
  // PRINTF(("idNode.size = %lu\n", s));
  return s;
}

}; // end-of-namespace Mdf3Nodes;
#if defined(TEST_MDF)

int main(int argc, char* argv[]) {
  int i;

  #if defined(DEBUG) && defined(CRTMEMCHECK)
  _CrtMemState mem1, mem2, mem3;
  int tmpDbgFlag;

  _CrtSetReportMode( _CRT_ERROR, _CRTDBG_MODE_DEBUG );
  /*
    * Set the debug-heap flag to keep freed blocks in the
    * heap's linked list - This will allow us to catch any
    * inadvertent use of freed memory
    */
  tmpDbgFlag = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);
  tmpDbgFlag |= _CRTDBG_CHECK_ALWAYS_DF;
  //tmpDbgFlag |= _CRTDBG_CHECK_CRT_DF;


  tmpDbgFlag |= _CRTDBG_DELAY_FREE_MEM_DF;
  tmpDbgFlag |= _CRTDBG_LEAK_CHECK_DF;
  _CrtSetDbgFlag(tmpDbgFlag);
#endif

#if defined(DEBUG) && defined(CRTMEMCHECK)
    _CrtMemCheckpoint(&mem1);
    //OutputDebugString("Dump mem before");
    //_CrtMemDumpStatistics(&mem1);
#endif

  Mdf3 *mdf = new Mdf3();
  mdf->initAndSetVersion(320);
  MdfStatus stat = MDF_OK;

  // Set file name
  mdf->create("kalle.mdf");
  // Create message
  stat = mdf->new_dg(0x100, 5);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }

  // Create signal
  stat = mdf->new_sig(0x100, "Databas::Laban", "Laban", "L-unit", 0, 4*8, MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }

  stat = mdf->new_dg(0x101, 5);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }
  stat = mdf->new_sig(0x101, "Databas::Nisse", "Nisse", "N-unit" ,0, 4*8, MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }


  stat = mdf->new_dg(0x102, 5);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }
  stat = mdf->new_sig(0x102, "Databas::Pelle", "Pelle", "P-unit", 0, 3*8, MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }
  stat = mdf->new_sig(0x102, "Databas::Kurre", "Kurre", "K-unit", 3*8, 2*8, MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_LE);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }


  stat = mdf->new_dg(0x103, 8);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }
  stat = mdf->new_sig(0x103, "Databas::Ramp", "Ramp", "R-unit", 0, 4*8, MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT_BE);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }
  stat = mdf->new_sig(0x103, "Databas::Foo", "Foo", "F-unit", 4*8, 4*8, MDF_SIGNAL_DATA_TYPE_UNSIGNED_INT, MDF_CONVERSION_TYPE_PARAMETRIC_LINEAR, 2.0, 35.0);
  if (stat != MDF_OK) { printf("error %d at line %d\n", stat, __LINE__); }

  // Write header once before adding data
  mdf->write_header();


  // Add data
  mdf->addMsg(0x101, 0,(MDF_UINT8 *)"Hello");
  mdf->addMsg(0x101, 1,(MDF_UINT8 *)"World");
  mdf->addMsg(0x102, 2,(MDF_UINT8 *)"hELLO");
  mdf->addMsg(0x102, 3,(MDF_UINT8 *)"wORLD");
  mdf->addMsg(0x100, 4,(MDF_UINT8 *)"\xAA\x55\xAA\x55\xAA");


  for (i = 5; i < 3800; i++) {
    unsigned int value[2];
    value[0] = (i) % 5;
    value[1] = (i) % 5;
    mdf->addMsg(0x103, i, (MDF_UINT8 *)value);
    if (i % 100000 == 0 && mdf->size() == (unsigned long)-1) {
      break;
    }
  }

  // Write all data (to update all dg-block's DataRecord-field
  mdf->write_data();

  // Write header again to make sure all links are OK
  mdf->write_header();

  printf("size = %lu\n", mdf->size());


  // Short test to see if ADD_UL works
  //unsigned long foo = MDF_MAX_FILE_SIZE - 5;
  //for (i = 0; i < 55; i++) {
  //  printf("i=%d\t",i); ADD_UL(foo, 1);
  //}

  delete mdf;

#if defined(DEBUG) && defined(CRTMEMCHECK)
    _CrtMemCheckpoint(&mem2);
    //OutputDebugString("Dump mem after");
    //_CrtMemDumpStatistics(&mem2);
    _CrtMemDifference(&mem3, &mem1, &mem2);
    OutputDebugString("Dump difference before and after");
    _CrtMemDumpStatistics(&mem3);
#endif

  return 0;
}
#endif // TEST_MDF
