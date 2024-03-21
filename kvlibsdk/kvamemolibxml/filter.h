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
** Description: Class to handle filers in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#ifndef FILTER_H_
#define FILTER_H_

#include <string>
#include <vector>
#include "kvaMemoLibXMLConst.h"
#include "xmlref.h"

class Filter : public Xmlref {
  public:
    bool mIsPass;
    std::vector<int> mChan;
    // Matching the types in logger_config_file.h
    uint8_t   mType;   // FILTER_TYPE_ONLY_ID,...
    uint8_t   mProt;   // HLP_TYPE_J1939
    uint16_t  mFlags;  // FILTER_FLAG_STD, FILTER_FLAG_IGNORE_DLC, ...

    uint32_t  mId;
    uint32_t  mIdMin;
    uint32_t  mIdMask;
    uint8_t   mDlc;
    uint8_t   mDlcMin;

    uint8_t   mDataType;
    uint16_t  mStartbit;
    uint16_t  mLength;
    std::vector<uint32_t>  mData;

    uint16_t mCounterThreshold;
    uint16_t mCounterMax;

    Filter(std::string);
    ~Filter();
    bool isEqual(const Filter* other) const;

    void print();
    bool usesChannel(int chan);
    int parseBinary(const char *buffer, const unsigned char version);
    int createXml(xmlTextWriterPtr writer) const;
};

class FilterrefList : public XmlrefList {
 private:
  unsigned int getNumberOfFilters(std::vector<Xmlref*>& filters, bool isPass);
  Filter* find(Filter *f);
  void copyFilterToStructFd(Filter *f , FilterVarFd* fv);
  void copyFilterToStruct(Filter *f , FilterVar* fv);

 public:
  size_t getFilters(int chan, std::vector<Xmlref*>& filters);
  unsigned int getNumberOfPassFilters(std::vector<Xmlref*>& filters);
  unsigned int getNumberOfStopFilters(std::vector<Xmlref*>& filters);
  void getActiveFiltersMask(std::vector<Xmlref*>& filters, std::vector<uint32_t>& mask);

  int createBinary(unsigned char *buffer, unsigned char version);
  int parseBinary(const char *buffer, const unsigned char version, uint32_t &filtNo);
  int createXml(xmlTextWriterPtr writer) const;
  void parseXml(xmlNode *root_node);
};


#endif
