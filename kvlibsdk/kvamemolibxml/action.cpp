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
** Description: Class to handle actions in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** ---------------------------------------------------------------------------
*/

#include "action.h"
#include "xmlwriter.h"
#include "convert.h"

#include <stdio.h>


Action::Action(std::string name) : Xmlref(name)
{
}

void Action::print()
{
  PRINTF(("    %d %s [", mIdx, mName.c_str()));
  PRINTF(("      type    %d", mType));
  PRINTF(("    ]\n"));
}

uint32_t Action::actionParam()
{
  return ACTION_LOG;
}

int Action::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, ActionTypeStrings, ActionTypeValues, ActionTypeLength));
  xml.endElement();

  return 0;
}


// ----------------------------------------------------------------------------------

PulseAction::PulseAction(std::string name) : Action(name)
{
  mDuration = 0;
}

void PulseAction::print()
{
  PRINTF(("    %d %s [", mIdx, mName.c_str()));
  PRINTF(("      type     %d", mType));
  PRINTF(("      duration %u", mDuration));
  PRINTF(("    ]\n"));
}

uint32_t PulseAction::actionParam()
{
  return mDuration;
}

int PulseAction::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, ActionTypeStrings, ActionTypeValues, ActionTypeLength));
  xml.writeAttr(XML_ACTIONS_ATTR_DURATION, "%u", mDuration);
  xml.endElement();

  return 0;
}

// ----------------------------------------------------------------------------------

TransmitAction::TransmitAction(std::string name) : Action(name)
{
  mTlistName = "";
  mTlistIndex = 0;
}

void TransmitAction::print()
{
  PRINTF(("    %d %s [", mIdx, mName.c_str()));
  PRINTF(("      type                %d", mType));
  PRINTF(("      transmit list name  %s", mTlistName.c_str()));
  PRINTF(("      transmit list idx   %d", mTlistIndex));
  PRINTF(("    ]\n"));
}

uint32_t TransmitAction::actionParam()
{
  return mTlistIndex;
}

int TransmitAction::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(uint8_to_name(mType, ActionTypeStrings, ActionTypeValues, ActionTypeLength));
  xml.writeAttr(XML_ACTIONS_ATTR_TRANSMITLIST, mTlistName);
  xml.endElement();

  return 0;
}
