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
** Description: Description of Kvaser XML format v2.0
** -----------------------------------------------------------------------------
*/

#include "xmlreference.h"
#include "xmlwriter.h"
#include "kvaMemoLibXMLConst.h"

void writeKvaserSpecification(xmlTextWriterPtr writer)
{
  XMLWriter w(writer);

  w.beginDocument();

  // Version
  w.beginRefElement(XML_KVASER_VERSION, 1, 1);
  w.endElement();
  w.beginRefElement(XML_KVASER_BIN_VERSION, 1, 1);
  w.endElement();

  // Settings
  w.beginRefElement(XML_SETTINGS, 1);
    w.beginRefElement(XML_MODE, 1);
    w.endElement();
    w.beginRefElement(XML_AFTERBURNER, 1);
    w.endElement();
    w.beginRefElement(XML_TARGET_EAN);
    w.endElement();
    w.beginRefElement("COMMENT");
    w.endElement();
  w.endElement();

  // CAN Bus
  w.beginRefElement(XML_BUSPARAM_BLOCK, 1);
    w.beginRefElement(XML_BUSPARAM, MAX_NR_OF_CAN_CHANNELS);
    w.endElement();
  w.endElement();

   // Trigger block
   w.beginRefElement(XML_TRIGGER_BLOCK, 1);
    // Triggers
    w.beginRefElement(XML_TRIGGERS, 1, 1);
      for (unsigned int k=0; k < TrigvarTypeLength; k++){
        w.beginRefElement(TrigvarTypeStrings[k], MAX_NR_OF_TRIGVARS);
        w.endElement();
      }
    w.endElement();

    // Statements
    w.beginRefElement(XML_STATEMENT_BLOCK, 1);
      w.beginRefElement(XML_STATEMENT, MAX_NR_OF_STATEMENTS);
        w.beginRefElement(XML_EXPRESSION, 1, 1);
        w.endElement();
        w.beginRefElement(XML_ACTIONS, 1, 1);
          for (unsigned int k=0; k < ActionTypeLength; k++){
            w.beginRefElement(ActionTypeStrings[k], MAX_ACTIONS_PER_STATEMENT);
            w.endElement();
          }
        w.endElement();
      w.endElement();
    w.endElement();
  w.endElement();

  // Transmit lists
  w.beginRefElement(XML_TRANSMIT_LIST_BLOCK, 1);
    w.beginRefElement(XML_TRANSMIT_LIST, MAX_NR_OF_AUTO_TRANSMIT_CHAINS);
      w.beginRefElement(XML_TRANSMIT_MSG, MAX_NR_OF_AUTO_TRANSMIT_MSGS);
      w.endElement();
    w.endElement();
  w.endElement();

  // Messages
  w.beginRefElement(XML_MSG_BLOCK, 1);
    w.beginRefElement(XML_MSG);
    w.endElement();
  w.endElement();

  // Filters
  w.beginRefElement(XML_FILTER_BLOCK, 1);
    for (unsigned int k=0; k < FilterTypeLength; k++){
      w.beginRefElement(FilterTypeStrings[k], MAX_NR_OF_FILTERS);
        w.beginRefElement(XML_FILTER_CHAN, MAX_NR_OF_CAN_CHANNELS);
        w.endElement();
      w.endElement();
    }
  w.endElement();

  // Scripts
  w.beginRefElement(XML_SCRIPT_BLOCK, 1);
    w.beginRefElement(XML_SCRIPT, EAGLE_MAX_NR_OF_SCRIPTS);
      w.beginRefElement(XML_SCRIPT_FILE, 1);
      w.endElement();
      w.beginRefElement(XML_SCRIPT_PATH, 1);
      w.endElement();
    w.endElement();
  w.endElement();

  w.endDocument();
}
