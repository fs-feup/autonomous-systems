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
** Description: Class to handle references in XML for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <libxml/xmlwriter.h>

#include "xmlref.h"
#include "kvdebug.h"
#include "errorhandler.h"
#include "filter.h"
#include "util.h"

using namespace std;

// ===========================================================================
// Xmlref
// ===========================================================================
Xmlref::Xmlref(std::string name)
{
  clearws(name);
  mName = name;
  mIdx = INVALID_IDX;
}
// ===========================================================================
// XmlrefList
// ===========================================================================
int XmlrefList::add(Xmlref* p)
{
  p->mIdx = mLastIdx++;
  mList.push_back(p);
  return p->mIdx;
};

bool XmlrefList::isUnique(std::string name)
{
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)->mName == name) return false;
  }
  return true;
}

int XmlrefList::getIdx(std::string name)
{
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)->mName == name) return (*it)->mIdx;
  }
  return INVALID_IDX;
}

Xmlref* XmlrefList::get(std::string name)
{
  clearws(name);
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)->mName == name) return (*it);
  }
  return NULL;
}
XmlrefList::~XmlrefList() {
  clear();
}

void XmlrefList::clear()
{
  mLastIdx = 0;
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) delete (*it);
  }
  mList.clear();
}

void XmlrefList::print()
{
  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) (*it)->print();
  }
}

int XmlrefList::createBinary(unsigned char *buffer, unsigned char version)
{
  unsigned char *pb = buffer;

  for(std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) {
      pb += (*it)->createBinary(pb, version);
    }
  }

  return (int32_t)(pb - buffer);
}

int XmlrefList::createXml(xmlTextWriterPtr writer) const
{
  for(std::vector<Xmlref*>::const_iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) {
      (*it)->createXml(writer);
    }
  }
  return 0;
}
