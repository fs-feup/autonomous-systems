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
** Description: Class to handle scripts in XML settings for
**              Kvaser Memorator Pro(2nd generation)
** -----------------------------------------------------------------------------
*/

#include <script.h>
#include <cstdio>
#include <string>
#include <cstring>

#include "util.h"
#include "convert.h"
#include "errorhandler.h"
#include "xmlwriter.h"

// ---------------------------------------------------------------------------
static KvaXmlStatus buffer_to_scriptfile (const char *filename, void *buf, size_t buflen)
{
  FILE* fp;

  fp = fopen(filename, "wb");
  if (!fp) {
    set_error(KvaXmlStatusFail, "Error: Could not create output file '%s'.\n", filename);
    return KvaXmlStatusFail;
  }
  fwrite(buf, 1, buflen, fp);
  fclose(fp);
  return KvaXmlStatusOK;
}


Script::Script(std::string name) : Xmlref(name)
{
  mIsPrimary   = false;
  mIsExternal  = false;
  mDefaultCh   = 0;
  mFilename    = "";
  mScriptLen   = 0;
}

void Script::print()
{
  PRINTF(("%d %s [", mIdx, mName.c_str()));
  PRINTF(("  Primary    = %s", mIsPrimary ? "Yes" : "No"));
  PRINTF(("  External   = %s", mIsExternal ? "Yes" : "No"));
  PRINTF(("  Default ch = %u", mDefaultCh));
  PRINTF(("  Filename h = %s", mFilename.c_str()));
  PRINTF(("  Script len = %u", mScriptLen));
  PRINTF(("]\n"));
}

int Script::createBinary(unsigned char *buffer, unsigned char /* version */)
{
  unsigned char *pb;
  BlockHead     *ph;

  pb = buffer;
  ph = (BlockHead*) pb;

  if (mIsExternal) {
    TMhydraScriptConfigSd* script = NULL;

    ph->id = BLOCK_ID_SCRIPT_SD;
    ph->len = sizeof(BlockHead) + sizeof(TMhydraScriptConfigSd);

    script = (TMhydraScriptConfigSd*) (pb + sizeof(BlockHead));
    memset(script, 0, sizeof (TMhydraScriptConfigSd));

    script->used           = 1;
    script->primary        = mIsPrimary != 0;
    script->defaultChannel = mDefaultCh;
    script->script_ext     = mIsExternal != 0;
    script->scriptLen      = mScriptLen;

    memcpy(script->scriptName, mFilename.c_str(), mFilename.length());

  } else {
    TMhydraScriptConfig* script = NULL;

    ph->id = BLOCK_ID_SCRIPT;
    ph->len = sizeof(BlockHead) + sizeof(TMhydraScriptConfig) - MAX_SCRIPT_CODE_SIZE + mScriptLen; // Packing internal scripts

    script = (TMhydraScriptConfig*) (pb + sizeof(BlockHead));
    memset(script, 0, sizeof (TMhydraScriptConfig));

    script->used           = 1;
    script->primary        = mIsPrimary != 0;
    script->defaultChannel = mDefaultCh;
    script->script_ext     = mIsExternal != 0;
    script->scriptLen      = mScriptLen;

    memcpy(script->scriptCode, mScriptCode, mScriptLen);

    PRINTF(("used %u", script->used));
    PRINTF(("primary %u", script->primary));
    PRINTF(("scriptlen %u", script->scriptLen));
    PRINTF(("scriptext %u", script->script_ext));
    PRINTF(("bh->len %u", ph->len));
  }

  pb += ph->len;
  print_block(ph);

  return (int32_t)(pb - buffer);
}

int Script::parseBinary(const char *buffer, const bool is_external,
                        const char *scriptpath, uint32_t &scriptNo)
{
  std::string filename;

  if (!buffer) throw_nullpointer(__FUNCTION__);

  if (is_external) {
    TMhydraScriptConfigSd* script = (TMhydraScriptConfigSd *) buffer;

    if (!script->used) return 0;

    mIsPrimary  = script->primary != 0;
    mDefaultCh  = script->defaultChannel;
    mIsExternal = script->script_ext != 0;
    mScriptLen  = script->scriptLen;
    mFilename   = std::string(script->scriptName);
  }
  else {
    TMhydraScriptConfig* script = (TMhydraScriptConfig *) buffer;

    if (!script->used) return 0;

    mIsPrimary  = script->primary != 0;
    mDefaultCh  = script->defaultChannel;
    mIsExternal = script->script_ext != 0;
    mScriptLen  = script->scriptLen;
    mFilename   = "script_" + intToString(scriptNo) + ".txe";
    mScriptLen  = script->scriptLen;
    memcpy(mScriptCode, script->scriptCode, mScriptLen);

    if (strlen(scriptpath) > 0)  {
      mPath = std::string(scriptpath);
      filename = mPath + "/" + mFilename;
    }
    else  {
      filename = mFilename;
    }
    if (buffer_to_scriptfile(filename.c_str(), (void *) mScriptCode, mScriptLen)) {
      throw_xml_writer_failure ("Failed to create script file");
    }
  }

  return 1;
}

int Script::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_SCRIPT);

  xml.writeAttrYesNo(XML_SCRIPT_ATTR_PRIM, mIsPrimary);
  xml.writeAttr(XML_SCRIPT_ATTR_CHAN, "%u", mDefaultCh);
  xml.writeAttrYesNo(XML_SCRIPT_ATTR_EXT, mIsExternal);

  xml.beginElement(XML_SCRIPT_FILE);
  xml.writeElementString(mFilename.c_str());
  xml.endElement();

  if (mPath.length() > 0) {
    xml.beginElement(XML_SCRIPT_PATH);
    xml.writeElementString(mPath.c_str());
    xml.endElement();
  }

  xml.endElement();

  return 0;
}

int ScriptrefList::parseBinary(const char *buffer, const bool is_external,
                               const char *scriptpath, uint32_t &scriptNo)
{
  Script *script = new Script("Script" + intToString(scriptNo));
  if (script->parseBinary(buffer, is_external, scriptpath, scriptNo) != 0) {
    add(script);
    ++scriptNo;
  }
  else {
    delete script;
  }

  return 1;
}

int ScriptrefList::createXml(xmlTextWriterPtr writer) const
{
  XMLWriter xml(writer);

  xml.beginElement(XML_SCRIPT_BLOCK);

  for(std::vector<Xmlref*>::const_iterator it = mList.begin(); it != mList.end(); ++it) {
    if ((*it)) {
      (*it)->createXml(writer);
    }
  }

  xml.endElement();

  return 0;
}

void ScriptrefList::parseXml(xmlNode *root_node)
{
  xmlNode *script_block_node = NULL;
  xmlNode *script_node = NULL;
  int cntPrim = 0;
  size_t size = 0;

  if (!root_node) throw_nullpointer(__FUNCTION__);
  script_block_node = findFirstElementNode(root_node, XML_SCRIPT_BLOCK);
  if (!script_block_node) {
    return;
  }

  for (script_node = script_block_node->children; script_node != NULL; script_node = script_node->next) {
    if ( isScriptElement(script_node) ) {
      Script *script = new Script("Script");

      print_prop(script_node);

      if (mLastIdx > EAGLE_MAX_NR_OF_SCRIPTS - 1) {
        set_error_and_throw(KvaXmlStatusERR_VALUE_RANGE,
                            "Too many active t-scripts. Only %d t-scripts can be downloaded to current device.",
                            EAGLE_MAX_NR_OF_SCRIPTS);
      }

      script->mIsPrimary = yes_to_uint8(script_node, XML_SCRIPT_ATTR_PRIM) != 0;
      script->mIsExternal = yes_to_uint8_or_default(script_node, XML_SCRIPT_ATTR_EXT, 0) != 0;
      script->mDefaultCh = num_to_uint8(script_node, XML_SCRIPT_ATTR_CHAN);
      script->mFilename = getScriptName(script_node);
      PRINTF(("script filename: '%s'\n", script->mFilename.c_str()));

      if (!script->mIsExternal) {

        if ( !getFileSize(script->mFilename.c_str(), &size) ) {
          set_error_and_throw(KvaXmlStatusERR_SCRIPT_ERROR,
            "Could not open script file '%s'.\n",
            script->mFilename.c_str());
        }

        if ( size > MAX_SCRIPT_CODE_SIZE ) {
          set_error_and_throw(KvaXmlStatusERR_SCRIPT_ERROR,
            "Script file '%s' is too large with with %d bytes. Max script size is %d bytes.",
            script->mFilename.c_str(), size, MAX_SCRIPT_CODE_SIZE);
        }

        script->mScriptLen = (uint32_t)size;

        if ( !copyFileToBuffer(script->mFilename.c_str(), script->mScriptCode, MAX_SCRIPT_CODE_SIZE) ) {
          set_error_and_throw(KvaXmlStatusERR_SCRIPT_ERROR, "Could not copy script file '%s'.");
        }
      } else {
        if (script->mFilename.length() > 12) {
          set_error_and_throw(KvaXmlStatusERR_SCRIPT_ERROR,
              "Script filename '%s' is too large with with %d bytes. Max script size is %d bytes.",
              script->mFilename.c_str(), script->mFilename.length(), 12);
        }
        memcpy(script->mScriptCode, script->mFilename.c_str(), script->mFilename.length());
        script->mScriptLen = 40;
      }
      PRINTF(("t-script filename: '%s' len=%u  size %u, ext %d\n",
              script->mFilename.c_str(), (unsigned int)script->mFilename.length(), script->mScriptLen, script->mIsExternal));
      add(script);
    }
  }

  // Only one script can be primary
  for (std::vector<Xmlref*>::iterator it = mList.begin(); it != mList.end(); ++it) {
    cntPrim += ((Script*)*it)->mIsPrimary;
    if (cntPrim > 1) {
      set_error_and_throw(KvaXmlStatusERR_SCRIPT_ERROR, "More than one script is set to 'primary'.");
    }
  }
}
