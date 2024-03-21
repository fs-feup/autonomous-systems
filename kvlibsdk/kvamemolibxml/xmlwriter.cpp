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
** Description: Class to write XML
** -----------------------------------------------------------------------------
*/

#include <sstream>
//#include <iostream>

#include "xmlwriter.h"
#include "errorhandler.h"

#include "util.h"
#include "convert.h"

using namespace std;

const char XML_ACTIVE_ATTR[] = "active";
const char XML_YES[] = "YES";
const char XML_NO[] = "NO";

void XMLWriter::writeElementString(int num)
{
  writeElementString("%d", num);
}

void XMLWriter::writeElementString(const char * format, ...)
{
  va_list args;
  va_start (args, format);
  vsnprintf(mBuf, sizeof(mBuf), format, args);
  va_end (args);
  if (xmlTextWriterWriteString(mWriter, BAD_CAST mBuf) == -1) {
    string element = popCurrentElement();
    throw_xml_writer_element_failure (element.c_str());
  }
}

void XMLWriter::writeAttr(const char * name, std::string str)
{
  writeAttr(name, "%s", str.c_str());
}

void XMLWriter::writeAttr(const char * name, const char * format, ...)
{
  va_list args;
  va_start (args, format);
  vsnprintf(mBuf, sizeof(mBuf), format, args);
  va_end (args);
  if (xmlTextWriterWriteAttribute(mWriter, BAD_CAST name, BAD_CAST mBuf) == -1) {
    string element = popCurrentElement();
    throw_xml_writer_attribute_failure (element.c_str(), name);
  }
}

void XMLWriter::writeAttrYesNo(const char * name, int expr)
{
  if (expr) {
    writeAttr(name, XML_YES);
  } else {
    writeAttr(name, XML_NO);
  }
}

void XMLWriter::beginElement(const char * str)
{
  string element(str);
  if (xmlTextWriterStartElement(mWriter, BAD_CAST str) == -1) {
    throw_xml_writer_element_failure (str);
  }
  mStack.push_back(element);
}

void XMLWriter::beginActiveElement(const char * str)
{
  beginElement(str);
  writeAttr(XML_ACTIVE_ATTR, XML_YES);
}

void XMLWriter::beginRefElement(const char * str, int max_cnt, int min_cnt)
{
  beginElement(str);
  if (max_cnt > 0) writeAttr(XML_REF_MAX_ELEMENT_COUNT_ATTR, intToString(max_cnt));
  if (min_cnt > 0) writeAttr(XML_REF_MIN_ELEMENT_COUNT_ATTR, intToString(min_cnt));
}


void XMLWriter::endElement()
{
  string element = popCurrentElement();
  if (xmlTextWriterEndElement(mWriter) == -1) {
    throw_xml_writer_element_failure (element.c_str());
  }
}

string XMLWriter::popCurrentElement()
{
  string element = "UNDEFINED";
  if ( !mStack.empty() ) {
    element = mStack.back();
    mStack.pop_back();
  }
  return element;
}

void XMLWriter::beginDocument()
{
  if (xmlTextWriterStartDocument(mWriter, "1.0", NULL, "yes") == -1) {
    throw_xml_writer_failure ("Unable to start document.");
  }
  beginElement(XML_KVASER_ROOT_NODE);
}

void XMLWriter::endDocument()
{
  endElement();

  if (xmlTextWriterEndDocument(mWriter) == -1) {
    throw_xml_writer_failure("Unable to end XML document.");
  }
}


