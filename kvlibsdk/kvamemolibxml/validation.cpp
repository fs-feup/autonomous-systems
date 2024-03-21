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
** Description: Validate XML depending on EAN
** -----------------------------------------------------------------------------
*/

#include <cstring>
#include "util.h"
#include "validation.h"
#include "errorhandler.h"
#include "convert.h"
#include "kvaMemoLibXMLConst.h"
#include "xmlvalidator.h"
#include "busparams.h"
#include "translist.h"
#include "statement.h"
#include "action.h"
#include "script.h"

static KvaXmlValidationStatus ValidationWarnings[MAX_NR_OF_WARNINGS] = {KvaXmlValidationStatusOK};
static KvaXmlValidationStatus ValidationErrors[MAX_NR_OF_ERRORS] = {KvaXmlValidationStatusOK};
static int ValidationWarningCount = 0;
static int ValidationErrorCount = 0;
static int CurrValidationWarningCount = 0;
static int CurrValidationErrorCount = 0;
static char ValidationErrorMessage[MAX_NR_OF_ERRORS][XML_ERROR_MESSAGE_LENGTH] = {0};
static char ValidationWarningMessage[MAX_NR_OF_WARNINGS][XML_ERROR_MESSAGE_LENGTH] = {0};

// ===========================================================================
//  Validators
// ===========================================================================

// ---------------------------------------------------------------------------
KvaXmlValidationStatus XmlStatusToValidationStatusErr(const KvaXmlStatus status)
{
  switch (status) {
  case KvaXmlStatusERR_EXPRESSION:
    return KvaXmlValidationStatusERR_EXPRESSION;

  case KvaXmlStatusERR_SCRIPT_ERROR:
    return KvaXmlValidationStatusERR_SCRIPT;

  case KvaXmlStatusERR_VALUE_RANGE:
  case KvaXmlStatusERR_VALUE_UNIQUE:
  case KvaXmlStatusERR_VALUE_CONSECUTIVE:
    return KvaXmlValidationStatusERR_NUM_OUT_OF_RANGE;

  case KvaXmlStatusERR_ATTR_NOT_FOUND:
  case KvaXmlStatusERR_ATTR_VALUE:
  case KvaXmlStatusERR_ELEM_NOT_FOUND:
  case KvaXmlStatusERR_XML_PARSER:
  case KvaXmlStatusERR_DTD_VALIDATION:
    return KvaXmlValidationStatusERR_PARSER;

  default:
    return KvaXmlValidationStatusFail;
  }
}

// ---------------------------------------------------------------------------
void clearValidationStatus (void)
{
  ValidationErrorCount = 0;
  ValidationWarningCount = 0;
  CurrValidationWarningCount = 0;
  CurrValidationErrorCount = 0;
  memset(&ValidationErrors, 0, sizeof(ValidationErrors));
  memset(&ValidationWarnings, 0, sizeof(ValidationWarnings));
}

void getValidationCount (int *countErr, int *countWarn)
{
  CurrValidationErrorCount = 0;
  CurrValidationWarningCount = 0;
  *countErr = ValidationErrorCount;
  *countWarn = ValidationWarningCount;
}

// ---------------------------------------------------------------------------
void setValidationError ( const KvaXmlValidationStatus status, const char * format, ... )
{
  if ( ValidationErrorCount < MAX_NR_OF_ERRORS - 1) {
    va_list args;
    va_start (args, format);
    vsnprintf(&ValidationErrorMessage[ValidationErrorCount][0], XML_ERROR_MESSAGE_LENGTH - 1, format, args);
    va_end (args);
    ValidationErrors[ValidationErrorCount] = status;
    ValidationErrorCount++;
  }
  else if (ValidationErrorCount < MAX_NR_OF_ERRORS) {
    snprintf(&ValidationErrorMessage[ValidationErrorCount][0],
             XML_ERROR_MESSAGE_LENGTH - 1,
             "The configuration contains too many errors; aborting.");
    ValidationErrors[ValidationErrorCount] = KvaXmlValidationStatusERR_ABORT;
    ValidationErrorCount++;
  }
}

// ---------------------------------------------------------------------------
static void setValidationWarning (KvaXmlValidationStatus status, const char * format, ... )
{
  if ( ValidationWarningCount < MAX_NR_OF_WARNINGS - 1) {
    va_list args;
    va_start (args, format);
    vsnprintf(&ValidationWarningMessage[ValidationWarningCount][0], XML_ERROR_MESSAGE_LENGTH - 1, format, args);
    va_end (args);
    ValidationWarnings[ValidationWarningCount] = status;
    ValidationWarningCount++;
  }
  else if (ValidationWarningCount < MAX_NR_OF_WARNINGS)
  {
    snprintf(&ValidationWarningMessage[ValidationWarningCount][0],
      XML_ERROR_MESSAGE_LENGTH - 1,
      "The configuration contains too many warnings; aborting.");

    ValidationWarnings[ValidationWarningCount] = KvaXmlValidationStatusWARN_ABORT;
    ValidationWarningCount++;
  }
}

// ---------------------------------------------------------------------------
KvaXmlValidationStatus getValidationError (char * buf, unsigned int len)
{
  KvaXmlValidationStatus status = KvaXmlValidationStatusOK;
  if (len > XML_ERROR_MESSAGE_LENGTH) {
    len = XML_ERROR_MESSAGE_LENGTH;
  }
  if (CurrValidationErrorCount < ValidationErrorCount) {
    if (buf) strncpy(buf, &ValidationErrorMessage[CurrValidationErrorCount][0], len - 1);
    status = ValidationErrors[CurrValidationErrorCount];
    CurrValidationErrorCount++;
  }
  return status;
}

// ---------------------------------------------------------------------------
KvaXmlValidationStatus getValidationWarning (char * buf, unsigned int len)
{
  KvaXmlValidationStatus status = KvaXmlValidationStatusOK;
  if (len > XML_ERROR_MESSAGE_LENGTH) {
    len = XML_ERROR_MESSAGE_LENGTH;
  }
  if (CurrValidationWarningCount < ValidationWarningCount) {
    status = ValidationWarnings[CurrValidationWarningCount];
    if (buf) strncpy(buf, &ValidationWarningMessage[CurrValidationWarningCount][0], len - 1);
    CurrValidationWarningCount++;
  }
  return status;
}

// ---------------------------------------------------------------------------
// Check that disabled triggers are not used.
void validateDisabledTriggers (XmlStruct &xr)
{
  std::vector<std::string> undefined;
  std::string expr;

  PRINTF(("validateDisabledTriggers"));

  // Check that all triggers used in expression are defined.
  for(std::vector<Xmlref*>::iterator it = xr.mTriggerStatements.mStatements.mList.begin(); it != xr.mTriggerStatements.mStatements.mList.end(); ++it) {
    expr = ((Statement*)(*it))->mInFixExpr.c_str();
    xr.mTriggerStatements.getTriggerStatus(expr, undefined);
    for (size_t k = 0; k < undefined.size() ; k++) {
      setValidationError(KvaXmlValidationStatusERR_UNDEFINED_TRIGGER, "The trigger '%s' is not defined.\n", undefined.at(k).c_str());
    }
  }
  undefined.clear();
}

// ---------------------------------------------------------------------------
// Check that either LOG_ALL, triggers or script is starts the logging
void validateActiveLogging (XmlStruct &xr)
{
  PRINTF(("validateActiveLogging"));

  // LOG_ALL. No need to check triggers or scripts
  if (xr.mTriggerStatements.mLogAll) {
    PRINTF(("LOG_ALL that starts the logging."));
    return;
  }

  // Active script. No need to check triggers
  if (xr.mScripts.mList.size() > 0) {
    PRINTF(("Active script that starts the logging."));
    return;
  }

  // Logging activated by trigger?
  // We only need to find an logging enabling action here. Expressions are checked during xml parsing.
  Statement *st = NULL;
  for (std::vector<Xmlref*>::iterator it = xr.mTriggerStatements.mStatements.mList.begin();
      it != xr.mTriggerStatements.mStatements.mList.end(); ++it) {
    st = (Statement*)(*it);
    for (std::vector<Xmlref*>::iterator actIt = st->mActions.mList.begin(); actIt != st->mActions.mList.end(); ++actIt) {
      if (((Action*)(*actIt))->mType == ACTION_START_LOG) {
        // There is an active statement with an active trigger with an action that starts the logging
        PRINTF(("Active statement with an active trigger with an action that starts the logging"));
        return;
      }
    }
  }

  PRINTF(("Validation: The configuration does not contain any expression to start the logging.\n"));
  setValidationWarning(KvaXmlValidationStatusWARN_NO_ACTIVE_LOG,
                       "The configuration does not contain any expression to start the logging.");
}

// ---------------------------------------------------------------------------
void validateTransmitAndSilentMode (XmlStruct &xr)
{
  bool channelSilent[MAX_NR_OF_CAN_CHANNELS] = {false};

  PRINTF(("validateTransmitAndSilentMode"));

  // Get the defined channels
  for (uint8_t ch = 0; ch < xr.mBusParams.mList.size(); ++ch) {
    channelSilent[ch] = ((Busparams*)xr.mBusParams.mList[ch])->mIsSilent;

    if (channelSilent[ch]) {
      PRINTF(("Channel %d is silent", ch + 1));
    }
  }

  // Check transmit list messages vs silent channels
  Translist *tList = NULL;
  for(std::vector<Xmlref*>::iterator it = xr.mTransmitlists.mList.begin(); it != xr.mTransmitlists.mList.end(); ++it) {
    tList = (Translist*)(*it);
    PRINTF(("list %s", tList->mName.c_str()));
    for(std::vector<TransMsg*>::iterator mIt = tList->mMessages.begin(); mIt != tList->mMessages.end(); ++mIt) {
      PRINTF(("msg %s", (*mIt)->mName.c_str()));
      if (channelSilent[(*mIt)->mCh]) {
        PRINTF(("Sound the alarm! tList %s, tMsg %s", tList->mName.c_str(), (*mIt)->mName.c_str()));
        setValidationError(KvaXmlValidationStatusERR_SILENT_TRANSMIT,
                           "Validation: Channel %u is in SILENT mode but is used by transmit message '%s' in transmit list '%s'.",
                           (*mIt)->mCh + 1, (*mIt)->mName.c_str(), tList->mName.c_str());
        PRINTF(("Validation: Channel %u is in SILENT mode but is used by transmit message '%s' in transmit list '%s'.",
                (*mIt)->mCh + 1, (*mIt)->mName.c_str(), tList->mName.c_str()));
      }
    }
  }
}

// ---------------------------------------------------------------------------
void validateSpecialTriggerCount (XmlStruct &xr)
{
  unsigned int  nofExternals = 0;
  unsigned int  nofStartups = 0;

  PRINTF(("Validating external triggers"));

  // Count occurrence of specific triggers.
  for (std::vector<Xmlref*>::iterator it = xr.mTriggerStatements.mTriggers.mList.begin();
       it != xr.mTriggerStatements.mTriggers.mList.end(); ++it) {
    switch ((*it)->mType) {
    case TRIGVAR_TYPE_EXTERNAL:
      nofExternals++;
      break;
    case TRIGVAR_TYPE_STARTUP:
      nofStartups++;
      break;
    }
  }

  if (nofExternals > 1) {
    PRINTF(("Validation: There are more than one active external trigger"));
    setValidationWarning(KvaXmlValidationStatusWARN_MULTIPLE_EXT_TRIGGER, "Using more than one external trigger requires firmware version 3.7 or better.");
    //        We can probably remove this warning in a couple of years from now (2017-09-14).
  }
  if (nofStartups > 1) {
    PRINTF(("Validation: There are more than one start up trigger defined"));
    setValidationError(KvaXmlValidationStatusERR_MULTIPLE_START_TRIGGER, "There are more than one start up trigger defined.");
  }
}

// ---------------------------------------------------------------------------
void validateTriggerDiskFull (XmlStruct &xr)
{
  PRINTF(("Validating TRIGVAR_TYPE_DISK_FULL"));

  // Starting logger on disk full?
  Statement *st = NULL;
  for (std::vector<Xmlref*>::iterator it = xr.mTriggerStatements.mStatements.mList.begin();
       it != xr.mTriggerStatements.mStatements.mList.end(); ++it) {
    st = (Statement*)(*it);
    for (std::vector<Xmlref*>::iterator actIt = st->mActions.mList.begin(); actIt != st->mActions.mList.end(); ++actIt) {
      if (((Action*)(*actIt))->mType == ACTION_START_LOG) {
        Trigger *trig = NULL;
        for (std::vector<std::string>::iterator trigIt = st->mTriggNames.begin(); trigIt != st->mTriggNames.end(); ++trigIt) {
          trig = (Trigger*)xr.mTriggerStatements.mTriggers.get(*trigIt);
          if (trig && trig->mType == TRIGVAR_TYPE_DISK_FULL) {
            setValidationError(KvaXmlValidationStatusERR_DISK_FULL_STARTS_LOG,
                               "%s is used to start logging when the disk is full.", trig->mName.c_str());
            PRINTF(("%s is used to start logging when the disk is full", trig->mName.c_str()));
          }
        }
      }
    }
  }

  // Trigger on disk full in fifo mode
  if (xr.mTriggerStatements.mFifoMode) {
    for (std::vector<Xmlref*>::iterator it = xr.mTriggerStatements.mTriggers.mList.begin();
       it != xr.mTriggerStatements.mTriggers.mList.end(); ++it) {
      if ((*it)->mType == TRIGVAR_TYPE_DISK_FULL) {
        setValidationWarning(KvaXmlValidationStatusWARN_DISK_FULL_AND_FIFO,
                             "%s triggers on disk full, but it is used in FIFO-mode and will never evaluate true.", (*it)->mName.c_str());
        PRINTF(("%s triggers on disk full, but it is used in FIFO-mode and will never evaluate true.", (*it)->mName.c_str()));
      }
    }
  }
}

// ---------------------------------------------------------------------------
void validateNumericalValues (XmlStruct &xr)
{
  PRINTF(("Validating Numerical Values"));

  if (xr.mAfterburner > KVASER_MAX_AFTERBURNER_TIME) {
    PRINTF(("Configured time to live (%d ms) after loss of CAN power is out of range. Allowed range is 0 to %d ms.",
            xr.mAfterburner, KVASER_MAX_AFTERBURNER_TIME));
    setValidationError(KvaXmlValidationStatusERR_NUM_OUT_OF_RANGE,
                       "Configured time to live (%d ms) after loss of CAN power is out of range. Allowed range is 0 to %d ms.",
                       xr.mAfterburner, KVASER_MAX_AFTERBURNER_TIME);
  }
}

// ---------------------------------------------------------------------------
void validateScripts (XmlStruct &xr)
{
  PRINTF(("Validating Script Block"));

  if (xr.mScripts.mList.size() > EAGLE_MAX_NR_OF_SCRIPTS) {
    PRINTF(("Too many active t-scripts. Only %d t-scripts can be downloaded to current device.\n",
            EAGLE_MAX_NR_OF_SCRIPTS));
    setValidationError(KvaXmlValidationStatusERR_SCRIPT_TOO_MANY,
                       "Too many active t-scripts. Only %d t-scripts can be downloaded to current device.",
                       EAGLE_MAX_NR_OF_SCRIPTS);
  }

  for (std::vector<Xmlref*>::iterator it = xr.mScripts.mList.begin(); it != xr.mScripts.mList.end(); ++it) {
    Script* script = (Script*)(*it);
    size_t  size   = 0;

    if (script->mDefaultCh > MAX_NR_OF_CAN_CHANNELS - 1) {
      setValidationError(KvaXmlValidationStatusERR_NUM_OUT_OF_RANGE,
                         "The script (%s) default channel (%u) is out of range. Allowed range is 0 to %d.",
                         script->mFilename.c_str(), script->mDefaultCh, MAX_NR_OF_CAN_CHANNELS);

      PRINTF(("The script (%s) default channel (%u) is out of range. Allowed range is 0 to %d.",
              script->mFilename.c_str(), script->mDefaultCh, MAX_NR_OF_CAN_CHANNELS));
    }

    if (!getFileSize(script->mFilename.c_str(), &size)) {
      setValidationError(KvaXmlValidationStatusERR_SCRIPT_NOT_FOUND,
                         "File not found; can't open '%s'.", script->mFilename.c_str());
      PRINTF(("File not found; can't open '%s'.", script->mFilename.c_str()));
    }

    if (size > MAX_SCRIPT_CODE_SIZE) {
      PRINTF(("Script file '%s' is too large with with %u bytes. Max script size is %d bytes.\n",
              script->mFilename.c_str(), (unsigned int)size, MAX_SCRIPT_CODE_SIZE));
      setValidationError(KvaXmlValidationStatusERR_SCRIPT_TOO_LARGE,
                         "Script file '%s' is too large with with %d bytes. Max script size is %d bytes.",
                         script->mFilename.c_str(), size, MAX_SCRIPT_CODE_SIZE);
    }
  }
}

// ---------------------------------------------------------------------------
void validateXmlElements (xmlNode *root_node)
{
  std::vector<std::string> ignoredElements, extraElements, missingElements;

  checkXmlContent(root_node, ignoredElements, extraElements, missingElements);

  for (size_t k=0; k < ignoredElements.size(); k++) {
    PRINTF(("Warning: ignored element '%s'", ignoredElements[k].c_str()));
    setValidationWarning(KvaXmlValidationStatusWARN_IGNORED_ELEMENT, "%s", ignoredElements[k].c_str());
  }
  ignoredElements.clear();
  for (size_t k=0; k < extraElements.size(); k++) {
    PRINTF(("Error: too many '%s' elements", extraElements[k].c_str()));
    setValidationError(KvaXmlValidationStatusERR_ELEMENT_COUNT, "%s", extraElements[k].c_str());
  }
  extraElements.clear();
  for (size_t k=0; k < missingElements.size(); k++) {
    PRINTF(("Error: too few '%s' elements", missingElements[k].c_str()));
    setValidationError(KvaXmlValidationStatusERR_ELEMENT_COUNT, "%s", missingElements[k].c_str());
  }
  missingElements.clear();
}
