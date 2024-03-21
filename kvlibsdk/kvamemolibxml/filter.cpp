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
** Description: Class to handle filters in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#include <cstdio>
#include <sstream>
#include <string>
#include <cstring>

#include "filter.h"
#include "util.h"
#include "convert.h"
#include "xmlwriter.h"
#include "errorhandler.h"
#include "kvdebug.h"

Filter::Filter(std::string name) :
  mIsPass(true),
  mProt(0),
  mFlags(0),
  mId(0),
  mIdMin(0),
  mIdMask(0xffffffff),
  mDlc(0),
  mDlcMin(0),

  mDataType(0),
  mStartbit(0),
  mLength(0),
  mCounterThreshold(0),
  mCounterMax(0)
{
  mName = name;
  mIdx = INVALID_IDX;
}

Filter::~Filter()
{
  mData.clear();
  mChan.clear();
}

bool Filter::isEqual(const Filter* other) const
{
  bool equal = false;

  switch (filtertype_bin_to_xml(mType)) {
  case MESSAGE_PASS_TYPE:
  case MESSAGE_STOP_TYPE:
  case MESSAGE_COUNTING_PASS_TYPE: {
    equal = (mIsPass  == other->mIsPass &&
             mProt    == other->mProt &&
             mFlags   == other->mFlags &&
             mId      == other->mId &&
             mIdMin   == other->mIdMin &&
             mIdMask  == other->mIdMask &&
             mDlc     == other->mDlc &&
             mDlcMin  == other->mDlcMin);
    if (filtertype_bin_to_xml(mType) == MESSAGE_COUNTING_PASS_TYPE) {
      equal = (equal &&
               mCounterThreshold == other->mCounterThreshold &&
               mCounterMax       == other->mCounterMax);
    }
    break;
  }
  case FLAGS_PASS_TYPE:
  case FLAGS_STOP_TYPE:
  case FLAGS_COUNTING_PASS_TYPE: {
      equal = (mIsPass   == other->mIsPass &&
               mProt     == other->mProt &&
               mFlags    == other->mFlags &&
               mId       == other->mId &&
               mIdMin    == other->mIdMin &&
               mIdMask   == other->mIdMask &&
               mDlc      == other->mDlc &&
               mDlcMin   == other->mDlcMin &&
               mDataType == other->mDataType &&
               mStartbit == other->mStartbit &&
               mLength   == other->mLength &&
               mData     == other->mData);

      if (filtertype_bin_to_xml(mType) == FLAGS_COUNTING_PASS_TYPE) {
        equal = (equal &&
                 mCounterThreshold == other->mCounterThreshold &&
                 mCounterMax       == other->mCounterMax);
      }
    break;
  }
    case SIGNAL_PASS_TYPE:
    case SIGNAL_STOP_TYPE:
    case SIGNAL_COUNTING_PASS_TYPE: {
      equal = (mIsPass  == other->mIsPass &&
               mProt    == other->mProt &&
               mFlags   == other->mFlags);
      if (filtertype_bin_to_xml(mType) == SIGNAL_COUNTING_PASS_TYPE) {
                 equal = (equal &&
                          mCounterThreshold == other->mCounterThreshold &&
                          mCounterMax       == other->mCounterMax);
      }
      break;
    }
  }

  PRINTF(("COMPARING %s (%s) with %s (%s). They are %sequal",
          mName.c_str(),
          FilterTypeStrings[filtertype_bin_to_xml(mType)],
          other->mName.c_str(),
          FilterTypeStrings[filtertype_bin_to_xml(other->mType)],
          equal ? "" : "not "));

  return equal;
}

bool Filter::usesChannel(int chan)
{
  for( std::vector<int>::iterator it = mChan.begin(); it != mChan.end(); ++it) {
    if (chan == (*it)) return true;
  }
  return false;
}

void Filter::print()
{
  std::stringstream sstm;
  for(std::vector<int>::iterator it = mChan.begin(); it != mChan.end(); ++it) {
    sstm << *it << ", ";
  }

  PRINTF(("%d %s [", mIdx, mName.c_str()));
  PRINTF(("  IsPass       = %s", mIsPass ? "Yes" : "No"));
  PRINTF(("  channels     = %s", sstm.str().c_str()));
  PRINTF(("  type         = 0x%x", mType));
  PRINTF(("  HLP mask     = 0x%x", mProt));
  PRINTF(("  Flags        = 0x%x", mFlags));
  PRINTF(("  Msg id       = %u", mId));
  PRINTF(("  Msg id min   = %u", mIdMin));
  PRINTF(("  Dlc          = %u", mDlc));
  PRINTF(("  Dlc min      = %u", mDlcMin));
  PRINTF(("  Datatype     = %u", mDataType));
  PRINTF(("  Startbit     = %u", mStartbit));
  PRINTF(("  Length       = %u", mLength));
  PRINTF(("  Data ["));
  for (unsigned int k = 0; k < mData.size(); k++) {
    PRINTF(("    0x%08x ", mData[k]));
  }
  PRINTF((  "]"));
  PRINTF(("  CntThreshold = %u", mCounterThreshold));
  PRINTF(("  CntMax       = %u", mCounterMax));
  PRINTF(("]"));
}

int Filter::parseBinary(const char *buffer, const unsigned char /* version */)
{
  FilterVarFd *fVar = (FilterVarFd *)buffer;

  PRINTF(("PARSING %s", FilterTypeStrings[filtertype_bin_to_xml(fVar->type)]));

  mIsPass = fVar->type & FILTER_INTERNAL_FLAG_PASS ? true : false;
  mType   = fVar->type;
  mProt   = fVar->HLPTypeSpec;
  mFlags  = fVar->flags;

  if (fVar->type & FILTER_INTERNAL_FLAG_ID) {
    idMsg *msg = &fVar->ident.msg;
    mId     = msg->id;
    mIdMin  = msg->id_min;
    mIdMask = msg->id_mask;
    mDlc    = msg->dlc;
    mDlcMin = msg->dlc_min;
  }
  else if (fVar->type & FILTER_INTERNAL_FLAG_SIGNAL) {
    idSignFd *sig = &fVar->ident.sign;
    mId       = sig->id;
    mIdMin    = sig->id;
    mDlc      = sig->dlc;
    mDlcMin   = sig->dlc;
    mDataType = sig->dataType;
    mStartbit = sig->startbit;
    mLength   = sig->length;
    mData.push_back(sig->data);
  }

  mCounterThreshold = fVar->counter_threshold;
  mCounterMax       = fVar->counter_max;

  return 1;
}

int Filter::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);
  uint8_t filterType = filtertype_bin_to_xml(mType);

  xml.beginElement(uint8_to_name(filterType,
                                 FilterTypeStrings,
                                 FilterTypeValues,
                                 FilterTypeLength));

  switch (filterType) {

  case MESSAGE_PASS_TYPE:
  case MESSAGE_STOP_TYPE:
  case MESSAGE_COUNTING_PASS_TYPE:

    xml.writeAttr(XML_FILTER_ATTR_ID_PROT, uint8_to_protocol(mProt));
    if (mProt & HLP_SPEC_MASK) {
      xml.writeAttr(XML_FILTER_ATTR_ID_MSK, uint8_to_protocol_msk(mProt));
    }

    xml.writeAttr(XML_FILTER_ATTR_ID,     "%u", mId);
    xml.writeAttr(XML_FILTER_ATTR_ID_MIN, "%u", mIdMin);

    if (!(mFlags & FILTER_FLAG_IGNORE_DLC)) {
      xml.writeAttr(XML_FILTER_ATTR_DLC, "%u", mDlc);
    }

    xml.writeAttrYesNo(XML_FILTER_ATTR_ID_EXT,  mFlags & FILTER_FLAG_EXT);

    if (filterType == MESSAGE_COUNTING_PASS_TYPE) {
      xml.writeAttr(XML_FILTER_ATTR_COUNTER_THRESHOLD, "%u", mCounterThreshold);
      xml.writeAttr(XML_FILTER_ATTR_COUNTER_MAX, "%u", mCounterMax);
    }
    break;

  case SIGNAL_PASS_TYPE:
  case SIGNAL_STOP_TYPE:
  case SIGNAL_COUNTING_PASS_TYPE:

    xml.writeAttr(XML_FILTER_ATTR_ID_PROT, uint8_to_protocol(mProt));
    if (mProt & HLP_SPEC_MASK) {
      xml.writeAttr(XML_FILTER_ATTR_ID_MSK, uint8_to_protocol_msk(mProt));
    }

    xml.writeAttr(XML_FILTER_ATTR_ID,  "%u", mId);

    if (!(mFlags & FILTER_FLAG_IGNORE_DLC)) {
      xml.writeAttr(XML_FILTER_ATTR_DLC, "%u", mDlc);
    }

    xml.writeAttrYesNo(XML_FILTER_ATTR_ID_EXT,  mFlags & FILTER_FLAG_EXT);
    xml.writeAttr(XML_FILTER_ATTR_STARTBIT,  "%u", mStartbit);
    xml.writeAttr(XML_FILTER_ATTR_BITLENGTH, "%u", mLength);
    xml.writeAttr(XML_FILTER_ATTR_DATATYPE, datatype_to_string(mDataType));
    xml.writeAttr(XML_FILTER_ATTR_ENDIAN, endian_to_string(mDataType));
    xml.writeAttr(XML_FILTER_ATTR_DATA, "0x%x", mData[0]);

    if (filterType == SIGNAL_COUNTING_PASS_TYPE) {
      xml.writeAttr(XML_FILTER_ATTR_COUNTER_THRESHOLD, "%u", mCounterThreshold);
      xml.writeAttr(XML_FILTER_ATTR_COUNTER_MAX, "%u", mCounterMax);
    }
    break;

  case FLAGS_PASS_TYPE:
  case FLAGS_STOP_TYPE:
  case FLAGS_COUNTING_PASS_TYPE:

    xml.writeAttrYesNo(XML_FILTER_ATTR_FLAG_STD, mFlags & FILTER_FLAG_STD);
    xml.writeAttrYesNo(XML_FILTER_ATTR_FLAG_EXT, mFlags & FILTER_FLAG_EXT);
    xml.writeAttrYesNo(XML_FILTER_ATTR_FLAG_EF,  mFlags & FILTER_FLAG_ERRORFRAME);

    if (filterType == FLAGS_COUNTING_PASS_TYPE) {
      xml.writeAttr(XML_FILTER_ATTR_COUNTER_THRESHOLD, "%u", mCounterThreshold);
      xml.writeAttr(XML_FILTER_ATTR_COUNTER_MAX, "%u", mCounterMax);
    }

    break;

  } // switch


  for (std::vector<int>::const_iterator it = mChan.begin(); it != mChan.end(); ++it) {
    xml.beginElement(XML_FILTER_CHAN);
    xml.writeElementString(*it);
    xml.endElement();
  }

  xml.endElement(); // Filter

  return 0;
}


size_t FilterrefList::getFilters(int chan, std::vector<Xmlref*>& filters)
{
  Filter *f = NULL;
  for( std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    f = (Filter*) (*it);
    if (f->mIsPass && f->usesChannel(chan)) filters.push_back(*it);
  }
  for( std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    f = (Filter*) (*it);
    if (!f->mIsPass && f->usesChannel(chan)) filters.push_back(*it);
  }
  return filters.size();
}

unsigned int FilterrefList::getNumberOfFilters(std::vector<Xmlref*>& filters, bool isPass)
{
  Filter *f = NULL;
  unsigned int count = 0;
  for( std::vector<Xmlref*>::iterator it = filters.begin(); it != filters.end(); ++it) {
    f = (Filter*) (*it);
    if (f->mIsPass == isPass) count++;
  }
  return count;
}

unsigned int FilterrefList::getNumberOfPassFilters(std::vector<Xmlref*>& filters)
{
  return getNumberOfFilters(filters, true);
}

unsigned int FilterrefList::getNumberOfStopFilters(std::vector<Xmlref*>& filters)
{
  return getNumberOfFilters(filters, false);
}

void FilterrefList::getActiveFiltersMask(std::vector<Xmlref*>& filters, std::vector<uint32_t>& mask)
{
  uint32_t m = 0;
  int cnt = 0;
  for( std::vector<Xmlref*>::iterator it = filters.begin(); it != filters.end(); ++it) {
    if (cnt == 31) {
      mask.push_back(m);
      m = 0;
      cnt = 0;
    }
    m |= (1 << cnt);
    cnt++;
  }
  mask.push_back(m);
}

int FilterrefList::createBinary(unsigned char *buffer, unsigned char version)
{
  unsigned char *pb;
  BlockHead     *ph;
  uint8_t ch;
  uint8_t filtNo;

  pb = buffer;
  ph = (BlockHead*) pb;

  if (KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD == version) {
    ConfigFileFilter* filt = NULL;

    // Copy each channel to a new block
    for (ch = 0; ch < MAX_NR_OF_CAN_CHANNELS; ch++) {
      std::vector<Xmlref*> filterList;

      ph = (BlockHead*) pb;
      ph->id = BLOCK_ID_FILTER;
      ph->len = sizeof(BlockHead) + sizeof(ConfigFileFilter);

      filt = (ConfigFileFilter*)(pb + sizeof(BlockHead));
      memset(filt, 0, sizeof(ConfigFileFilter));

      filt->channel = ch;
      filt->used    = 1;

      if (getFilters(ch, filterList)) {
        std::vector<uint32_t> mask;

        filt->totalNumberOfFilters = (uint8_t) filterList.size();

        // Pass filters first
        filt->beginPassFilters     = 0;
        filt->numberOfPassFilters  = getNumberOfPassFilters(filterList);

        // Stop filters after the last pass filter
        filt->beginStopFilters     = filt->numberOfPassFilters;
        filt->numberOfStopFilters  = getNumberOfStopFilters(filterList);

        PRINTF(("totalNumberOfFilters: %d", filt->totalNumberOfFilters));
        PRINTF(("beginPassFilters    : %d", filt->beginPassFilters));
        PRINTF(("numberOfPassFilters : %d", filt->numberOfPassFilters));
        PRINTF(("beginStopFilters    : %d", filt->beginStopFilters));
        PRINTF(("numberOfStopFilters : %d", filt->numberOfStopFilters));

        //        Can we prepare this at an earlier step?
        getActiveFiltersMask(filterList, mask);
        for (size_t k=0; k < mask.size(); k++) {
          if (k >= FILTER_WORDS) {
            mask.clear();
            //set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE, "Too many active filters on channel %d.", filt->channel);
          }
          filt->activeFiltersMask[k] = mask.at(k);
          //PRINTF(("activeFiltersMask[k]: 0x%I64x\n", *((__int64*)&filt->activeFiltersMask[k])));
        }
        mask.clear();

        //        Can we prepare this at an earlier step?
        filtNo = 0;
        for (std::vector<Xmlref*>::iterator it = filterList.begin(); it != filterList.end(); ++it) {
          //          for (size_t k=0; k < filterList.size(); k++) {
          //            f = (Filter*) filterList.at(k);
          //            if ( k >= MAX_NR_OF_FILTERS) {
          //              set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE, "Too many active filters on channel %d.", filtfd->channel);
          //            }
          copyFilterToStruct((Filter*)(*it), &filt->filterVariables[filtNo]);
          PRINTF(("Adding filter type '%s' on chan %d. id=%x flags=%x type=%x", (*it)->mName.c_str(), ch
                  , filt->filterVariables[filtNo].ident.msg.id, filt->filterVariables[filtNo].flags
                  , filt->filterVariables[filtNo].type));
          filtNo++;
        }
        filterList.clear();
      }
      pb += ph->len;
      print_block(ph);
    }
  }
  else if (KVASER_BINARY_VERSION_ELEMENT_MAJOR == version) {
    ConfigFileFilterFd* filt = NULL;

    // Copy each channel to a new block
    for (ch = 0; ch < MAX_NR_OF_CAN_CHANNELS; ch++) {
      std::vector<Xmlref*> filterList;

      ph = (BlockHead*) pb;
      ph->id = BLOCK_ID_FILTER_FD;
      ph->len = sizeof(BlockHead) + sizeof(ConfigFileFilter);

      filt = (ConfigFileFilterFd*)(pb + sizeof(BlockHead));
      memset(filt, 0, sizeof(ConfigFileFilter));

      filt->channel = ch;
      filt->used    = 1;

      if (getFilters(ch, filterList)) {
        std::vector<uint32_t> mask;

        filt->totalNumberOfFilters = (uint8_t) filterList.size();

        // Pass filters first
        filt->beginPassFilters     = 0;
        filt->numberOfPassFilters  = getNumberOfPassFilters(filterList);

        // Stop filters after the last pass filter
        filt->beginStopFilters     = filt->numberOfPassFilters;
        filt->numberOfStopFilters  = getNumberOfStopFilters(filterList);

        PRINTF(("totalNumberOfFilters: %d", filt->totalNumberOfFilters));
        PRINTF(("beginPassFilters    : %d", filt->beginPassFilters));
        PRINTF(("numberOfPassFilters : %d", filt->numberOfPassFilters));
        PRINTF(("beginStopFilters    : %d", filt->beginStopFilters));
        PRINTF(("numberOfStopFilters : %d", filt->numberOfStopFilters));

        //        Can we prepare this at an earlier step?
        getActiveFiltersMask(filterList, mask);
        for (size_t k=0; k < mask.size(); k++) {
          if (k >= FILTER_WORDS) {
            mask.clear();
            // set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE, "Too many active filters on channel %d.", filt->channel);
          }
          filt->activeFiltersMask[k] = mask.at(k);
          //PRINTF(("activeFiltersMask[k]: 0x%I64x\n", *((__int64*)&filt->activeFiltersMask[k])));
        }
        mask.clear();

        //        Can we prepare this at an earlier step?
        filtNo = 0;
        for (std::vector<Xmlref*>::iterator it = filterList.begin(); it != filterList.end(); ++it) {
          //          for (size_t k=0; k < filterList.size(); k++) {
          //            f = (Filter*) filterList.at(k);
          //            if ( k >= MAX_NR_OF_FILTERS) {
          //              set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE, "Too many active filters on channel %d.", filtfd->channel);
          //            }
          copyFilterToStructFd((Filter*)(*it), &filt->filterVariables[filtNo]);
          PRINTF(("Adding filter type '%s' on chan %d. id=%x flags=%x type=%x", (*it)->mName.c_str(), ch
                  , filt->filterVariables[filtNo].ident.msg.id, filt->filterVariables[filtNo].flags
                  , filt->filterVariables[filtNo].type));
          filtNo++;
        }
        filterList.clear();
      }
      pb += ph->len;
      print_block(ph);
    }
  }

  return (int32_t)(pb - buffer);
}

int FilterrefList::parseBinary(const char *buffer, const unsigned char version, uint32_t &filtNo)
{
  Filter *fp = NULL;

  if (KVASER_BINARY_VERSION_ELEMENT_MAJOR_OLD == version) {
    ConfigFileFilter *filt = (ConfigFileFilter *)buffer;

    if (!filt->used) return 0;

    PRINTF(("Parsing filter for channel %d, (%s)", filt->channel, filt->used ? "used" : "not used"));
    PRINTF(("totalNumberOfFilters : %d", filt->totalNumberOfFilters));
    PRINTF(("beginPassFilters     : %d", filt->beginPassFilters));
    PRINTF(("numberOfPassFilters  : %d", filt->numberOfPassFilters));
    PRINTF(("beginStopFilters     : %d", filt->beginStopFilters));
    PRINTF(("numberOfStopFilters  : %d", filt->numberOfStopFilters));

    for (int lFiltNo = 0; lFiltNo < filt->totalNumberOfFilters; ++lFiltNo) {
      fp = new Filter("Filter" + intToString(filtNo));
      fp->mChan.push_back(filt->channel);
      fp->parseBinary((const char*)&filt->filterVariables[lFiltNo], version);

      // Concatenate "equal" filters on different channels.
      // Filter *existingFilter = find(fp);
      // if (existingFilter) {
      //   existingFilter->mChan.insert(existingFilter->mChan.end(), fp->mChan.begin(), fp->mChan.end());
      //   delete fp;
      // }
      // else {
        add(fp);
        ++filtNo;
      // }
    }
  }
  else if (KVASER_BINARY_VERSION_ELEMENT_MAJOR == version) {
    ConfigFileFilterFd *filt = (ConfigFileFilterFd *)buffer;

    if (!filt->used) return 0;

    PRINTF(("Parsing filter for channel %d, (%s)", filt->channel, filt->used ? "used" : "not used"));
    PRINTF(("totalNumberOfFilters : %d", filt->totalNumberOfFilters));
    PRINTF(("beginPassFilters     : %d", filt->beginPassFilters));
    PRINTF(("numberOfPassFilters  : %d", filt->numberOfPassFilters));
    PRINTF(("beginStopFilters     : %d", filt->beginStopFilters));
    PRINTF(("numberOfStopFilters  : %d", filt->numberOfStopFilters));

    for (int lFiltNo = 0; lFiltNo < filt->totalNumberOfFilters; ++lFiltNo) {
      fp = new Filter("Filter" + intToString(filtNo));
      fp->mChan.push_back(filt->channel);
      fp->parseBinary((const char*)&filt->filterVariables[lFiltNo], version);

      //        However, this "feature" might screw up the internal order of filters in the XML file.
      //        So, if we would like generate an xml-file with the same number of filters and the same
      //        order we need to do something better here.
      //        As it works now a filter that is used on several channels will be expanded into several filters.
      //
      // Concatenate "equal" filters on different channels.
      // Filter *existingFilter = find(fp);
      // if (existingFilter) {
      //   existingFilter->mChan.insert(existingFilter->mChan.end(), fp->mChan.begin(), fp->mChan.end());
      //   delete fp;
      // }
      // else {
        add(fp);
        ++filtNo;
      // }
    }
  }
  else {
    PRINTF(("%s: UNSUPPORTED VERSION %d", __FUNCTION__, version));
    return 0;
  }
  return 1;
}

int FilterrefList::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_FILTER_BLOCK);

  for(std::vector<Xmlref*>::const_iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) {
      (*it)->createXml(writer);
    }
  }

  xml.endElement();

  return 0;
}

void FilterrefList::parseXml(xmlNode *root_node)
{
  xmlNode  *filter_node = NULL;
  xmlNode  *child_node = NULL;
  xmlNode  *chan_node = NULL;
  Filter   *pFilter = NULL;
  uint8_t   type;

  if (!root_node) throw_nullpointer(__FUNCTION__);

  filter_node = findFirstElementNode(root_node, XML_FILTER_BLOCK);
  if (!filter_node) {
    return;
  }

  for (child_node = filter_node->children; child_node != NULL; child_node = child_node->next) {
    if ( isFilterElement(child_node) ) {
      type = node_name_to_uint8(child_node, FilterTypeStrings, FilterTypeValues, FilterTypeLength);
      pFilter = new Filter(FilterTypeStrings[type]);

      switch (type) {
      case MESSAGE_PASS_TYPE:
      case MESSAGE_STOP_TYPE:
        {
          pFilter->mIsPass = (MESSAGE_PASS_TYPE  == type);
          pFilter->mId = num_to_uint32(child_node, XML_FILTER_ATTR_ID);
          pFilter->mIdMin = num_to_uint32(child_node, XML_FILTER_ATTR_ID_MIN);
          if (yes_to_uint8_or_default(child_node, XML_FILTER_ATTR_ID_EXT, 0)) {
            pFilter->mId |= CAN_IDENT_IS_EXTENDED;
            pFilter->mIdMin |= CAN_IDENT_IS_EXTENDED;
            pFilter->mFlags |= FILTER_FLAG_EXT;
          } else {
            pFilter->mFlags |=  FILTER_FLAG_STD;
          }

          if ( hasAttribute(child_node, XML_FILTER_ATTR_DLC) ) {
            pFilter->mType = FILTER_TYPE_ID_AND_DLC;
            pFilter->mDlc = num_to_uint8(child_node, XML_FILTER_ATTR_DLC);
            pFilter->mDlcMin = pFilter->mDlc;
          }
          else {
            pFilter->mType = FILTER_TYPE_ONLY_ID;
            pFilter->mDlc = 0;
            pFilter->mDlcMin = 0;
            pFilter->mFlags |=  FILTER_FLAG_IGNORE_DLC;
          }

          pFilter->mProt = protocol_to_uint8_or_default(child_node);
          pFilter->mProt |= protocol_msk_to_uint8_or_default(child_node);
        }
        break;


      case FLAGS_PASS_TYPE:
      case FLAGS_STOP_TYPE:
      case FLAGS_COUNTING_PASS_TYPE:
        {
          int nFlags = 0;

          pFilter->mType = FILTER_TYPE_JUST_FLAGS;
          pFilter->mIsPass = (FLAGS_STOP_TYPE != type);
          if (yes_to_uint8_or_default(child_node, XML_FILTER_ATTR_FLAG_STD, 0)) {
            pFilter->mFlags |= FILTER_FLAG_STD;
            nFlags++;
          }
          if (yes_to_uint8_or_default(child_node, XML_FILTER_ATTR_FLAG_EXT, 0)) {
            pFilter->mFlags |= FILTER_FLAG_EXT;
            nFlags++;
          }
          if (yes_to_uint8_or_default(child_node, XML_FILTER_ATTR_FLAG_EF, 0)) {
            pFilter->mFlags |= FILTER_FLAG_ERRORFRAME;
            nFlags++;
          }

          if ( hasAttribute(child_node, XML_FILTER_ATTR_COUNTER_MAX) ||
               hasAttribute(child_node, XML_FILTER_ATTR_COUNTER_THRESHOLD) ){
            pFilter->mType |= FILTER_INTERNAL_FLAG_CNTR;
            pFilter->mFlags |=  FILTER_FLAG_IGNORE_DLC;
            pFilter->mCounterThreshold = num_to_uint16(child_node,XML_FILTER_ATTR_COUNTER_THRESHOLD );
            pFilter->mCounterMax = num_to_uint16(child_node, XML_FILTER_ATTR_COUNTER_MAX);
          }

          if (nFlags != 1) {
            set_error_and_throw( KvaXmlStatusERR_ATTR_VALUE,
                                 "Filter flags must have one (and only one) flag, got %d.\n", nFlags);
            break;
          }
        }
        break;


      case SIGNAL_PASS_TYPE:
      case SIGNAL_COUNTING_PASS_TYPE:
      case SIGNAL_STOP_TYPE:
        {
          pFilter->mType = FILTER_TYPE_SIGNAL;
          pFilter->mIsPass = (SIGNAL_STOP_TYPE == type)?false:true;

          pFilter->mId = num_to_uint32(child_node, XML_FILTER_ATTR_ID);
          pFilter->mIdMin = pFilter->mId;
          if (yes_to_uint8_or_default(child_node, XML_FILTER_ATTR_ID_EXT, 0)) {
            pFilter->mId |= CAN_IDENT_IS_EXTENDED;
            pFilter->mIdMin |= CAN_IDENT_IS_EXTENDED;
            pFilter->mFlags |= FILTER_FLAG_EXT;
          } else {
            pFilter->mFlags |=  FILTER_FLAG_STD;
          }

          if ( hasAttribute(child_node, XML_FILTER_ATTR_DLC) ) {
            pFilter->mDlc = num_to_uint8(child_node, XML_FILTER_ATTR_DLC);
            pFilter->mDlcMin = pFilter->mDlc;
          }
          else {
            pFilter->mDlc = 0;
            pFilter->mDlcMin = 0;
            pFilter->mFlags |=  FILTER_FLAG_IGNORE_DLC;
          }

          pFilter->mProt = protocol_to_uint8_or_default(child_node);
          pFilter->mProt |= protocol_msk_to_uint8_or_default(child_node);
          pFilter->mStartbit = num_to_uint16(child_node, XML_FILTER_ATTR_STARTBIT);
          pFilter->mLength = num_to_uint16(child_node, XML_FILTER_ATTR_BITLENGTH);
          pFilter->mDataType = datatype_to_uint8(child_node);
          pFilter->mData.push_back(num_to_uint32(child_node, XML_FILTER_ATTR_DATA));

          if (SIGNAL_COUNTING_PASS_TYPE == type) {
            pFilter->mType = FILTER_TYPE_SIGNAL_COUNTER;
            pFilter->mCounterThreshold = num_to_uint16(child_node,XML_FILTER_ATTR_COUNTER_THRESHOLD );
            pFilter->mCounterMax = num_to_uint16(child_node, XML_FILTER_ATTR_COUNTER_MAX);
          }
        }
        break;


      case MESSAGE_COUNTING_PASS_TYPE:
        {
          pFilter->mIsPass = true;
          pFilter->mType = FILTER_TYPE_ID_COUNTER;
          pFilter->mCounterThreshold = num_to_uint16(child_node,XML_FILTER_ATTR_COUNTER_THRESHOLD );
          pFilter->mCounterMax = num_to_uint16(child_node, XML_FILTER_ATTR_COUNTER_MAX);

          pFilter->mId = num_to_uint32(child_node, XML_FILTER_ATTR_ID);
          pFilter->mIdMin = pFilter->mId;
          if (yes_to_uint8_or_default(child_node, XML_FILTER_ATTR_ID_EXT, 0)) {
            pFilter->mId |= CAN_IDENT_IS_EXTENDED;
            pFilter->mIdMin |= CAN_IDENT_IS_EXTENDED;
            pFilter->mFlags |= FILTER_FLAG_EXT;
          } else {
            pFilter->mFlags |=  FILTER_FLAG_STD;
          }

          if ( hasAttribute(child_node, XML_FILTER_ATTR_DLC) ) {
            pFilter->mDlc = num_to_uint8(child_node, XML_FILTER_ATTR_DLC);
            pFilter->mDlcMin = pFilter->mDlc;
          }
          else {
            pFilter->mDlc = 0;
            pFilter->mDlcMin = 0;
            pFilter->mFlags |=  FILTER_FLAG_IGNORE_DLC;
          }

          pFilter->mProt = protocol_to_uint8_or_default(child_node);
          pFilter->mProt |= protocol_msk_to_uint8_or_default(child_node);
        }
        break;


      default:
        throw_general_failure("getXMLFilters() internal error: Invalid filter type.");
        break;
      }

      // Get the associated channels
      for (chan_node = child_node->children; chan_node != NULL; chan_node = chan_node->next) {
        if ( isFilterChannelElement(chan_node) ) {
          int ch = (int) element_string_to_long(chan_node);
          if (ch > MAX_NR_OF_CAN_CHANNELS - 1) throw_value_range(chan_node, XML_FILTER_CHAN, ch, 0, MAX_NR_OF_CAN_CHANNELS - 1);
          pFilter->mChan.push_back(ch);
        }
      }
      add(pFilter);
      PRINTF(("Parsed filter %s, %s", pFilter->mName.c_str(), pFilter->mIsPass ? "pass" : "stop"));
    }
  } // loop
}

Filter* FilterrefList::find(Filter *f)
{
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if (f->isEqual((Filter*)(*it))) return (Filter*)(*it);
  }
  return NULL;
}

void FilterrefList::copyFilterToStruct(Filter *f , FilterVar* fv)
{
  if (!f || !fv) throw_nullpointer(__FUNCTION__);

  fv->type = f->mType;
  fv->type |= f->mIsPass?FILTER_INTERNAL_FLAG_PASS:FILTER_INTERNAL_FLAG_STOP;
  fv->HLPTypeSpec       = f->mProt;
  fv->counter_max       = f->mCounterMax;
  fv->counter_threshold = f->mCounterThreshold;
  fv->flags             = f->mFlags;

  switch(f->mType) {
  case FILTER_TYPE_ID_AND_DLC:
  case FILTER_TYPE_ONLY_ID:
  case FILTER_TYPE_JUST_FLAGS:
  case FILTER_TYPE_ID_COUNTER:
    fv->ident.msg.id = f->mId;
    fv->ident.msg.id_min = f->mIdMin;
    fv->ident.msg.id_mask = f->mIdMask;
    fv->ident.msg.dlc = f->mDlc;
    fv->ident.msg.dlc_min = f->mDlcMin;
    break;

  case FILTER_TYPE_SIGNAL:
  case FILTER_TYPE_SIGNAL_COUNTER:
    fv->ident.sign.id      = f->mId;
    if (f->mData.size() > 0) fv->ident.sign.data = f->mData.at(0);
    fv->ident.sign.dlc      = f->mDlc;
    fv->ident.sign.dataType = f->mDataType;
    fv->ident.sign.startbit = (uint8_t)f->mStartbit;
    fv->ident.sign.length   = (uint8_t)f->mLength;
    break;

  case FILTER_INTERNAL_FLAG_CNTR:
    // NOP
    break;

  default:
    set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE, "FILTER type %d is not implemented.", f->mType);
    break;
  }
}

void FilterrefList::copyFilterToStructFd(Filter *f , FilterVarFd* fv)
{
  if (!f || !fv) throw_nullpointer(__FUNCTION__);

  fv->type = f->mType;
  fv->type |= f->mIsPass?FILTER_INTERNAL_FLAG_PASS:FILTER_INTERNAL_FLAG_STOP;
  fv->HLPTypeSpec       = f->mProt;
  fv->counter_max       = f->mCounterMax;
  fv->counter_threshold = f->mCounterThreshold;
  fv->flags             = f->mFlags;

  switch(f->mType) {
  case FILTER_TYPE_ID_AND_DLC:
  case FILTER_TYPE_ONLY_ID:
  case FILTER_TYPE_JUST_FLAGS:
  case FILTER_TYPE_ID_COUNTER:
    fv->ident.msg.id = f->mId;
    fv->ident.msg.id_min = f->mIdMin;
    fv->ident.msg.id_mask = f->mIdMask;
    fv->ident.msg.dlc = f->mDlc;
    fv->ident.msg.dlc_min = f->mDlcMin;
    break;

  case FILTER_TYPE_SIGNAL:
  case FILTER_TYPE_SIGNAL_COUNTER:
    fv->ident.sign.id      = f->mId;
    if (f->mData.size() > 0) fv->ident.sign.data = f->mData.at(0);
    fv->ident.sign.dlc      = f->mDlc;
    fv->ident.sign.dataType = f->mDataType;
    fv->ident.sign.startbit = f->mStartbit;
    fv->ident.sign.length   = f->mLength;
    break;

  case FILTER_INTERNAL_FLAG_CNTR:
    // NOP
    break;

  default:
    set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE, "FILTER type %d is not implemented.", f->mType);
    break;
  }
}
