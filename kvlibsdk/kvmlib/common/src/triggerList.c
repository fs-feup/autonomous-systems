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
** Put measurements into a linked list based on the trigger blocks used.
** ---------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "triggerList.h"
#include "kvdebug.h"
#include "util.h"

// Allocate memory and copy a trigger to the global list.
void CopyTriggerToList(TriggerList *list, Trigger *trigger)
{
  TriggerItem *newTriggerItem = NULL;
  TriggerItem *triggerItem = NULL;

  // Allocate memory and copy the data
  newTriggerItem = (TriggerItem*) malloc(sizeof(TriggerItem));
  if (newTriggerItem == NULL) {
    PRINTF(("Out of memory."));
    return;
  }

  memcpy(&newTriggerItem->trigger,trigger, sizeof(Trigger));
  newTriggerItem->next = NULL;
  newTriggerItem->prev = NULL;


  // First entry?
  if (list->first == NULL && list->last == NULL) {
    // Put it in the list
    list->first = newTriggerItem;
    list->last = newTriggerItem;
    list->count = 1;
  } else {
    triggerItem = list->last;
    triggerItem->next = newTriggerItem;
    newTriggerItem->prev = triggerItem;
    list->last = newTriggerItem;
    list->count++;
  }
}

// Free memory and remove all trigger items from the list.
void DeleteTriggerList(TriggerList *list) {
  TriggerItem *triggerItem = NULL;
  TriggerItem *nextTriggerItem = NULL;
  triggerItem = list->first;
  if (triggerItem == NULL) {
    // No entries
    return;
  }
  list->first = NULL;
  while (triggerItem->next != NULL) {
    nextTriggerItem = triggerItem->next;
    free(triggerItem);
    triggerItem = nextTriggerItem;
  }
  list->last = NULL;
  list->count = 0;
}





// Print the list in reverse order (used for debug)
void PrintReverseTriggerList(TriggerList *list) {
  TriggerItem *triggerItem = NULL;

  PRINTF(("Track\t     Prev\t     Trigg\t       Pre\t    Latest\n"));
  PRINTF(("--------\t  --------\t  --------\t  --------\t  --------\n"));
  triggerItem = list->last;

  while (triggerItem != NULL) {
    Trigger *t = &triggerItem->trigger;
    if (t) {
      PRINTF(("%8d\t %8u\t  %8u\t  %8u\t  %8u\n",t->track, t->prevTriggerBlockSecNum,t->secNum,
              t->preTriggerBlockSecNum, t->prevWPSecNum));
    }
    triggerItem = triggerItem->prev;
  }
}

// Count Print the list in reverse order (used for debug)
int countTriggersOnTrack(TriggerList *list, int track) {
  TriggerItem *triggerItem = list->last;
  int counter = 0;
  while (triggerItem != NULL) {
    Trigger *t;
    t = &triggerItem->trigger;
    if (track == t->track) counter++;
    triggerItem = triggerItem->prev;
  }
  return counter;
}



// Get a pointer to the selected trigger block counting from the end of the list
Trigger* GetTrigger(TriggerList *list, int triggerNo) {
  TriggerItem *triggerItem = NULL;

  int k = 0;
  triggerItem = list->last;

  while (triggerItem != NULL) {
    if (k == triggerNo) {
      return &triggerItem->trigger;
    }
    triggerItem = triggerItem->prev;
    k++;
  }
  return NULL;
}

// Get the next data sector in a trigger, given the the current sector
uint32_t GetNextDataSector(Trigger *t, uint32_t currentDataSec)
{
  uint32_t nextSec = 0;

  // Increase by one sector and check the result
  nextSec = currentDataSec + 1;
  // Wrap around if the sector is outside the disk
  if (nextSec == t->trackEnd) {
    nextSec = t->trackStart + 8; //FIRST_AVAILABLE_SECTOR;
  }

  // Pre-trigger used
  if (t->preTriggerBlockSecNum) {
    // The pre-trigger has wrapped
    if (t->preTriggerIsWrapped) {

      // The trigger block; wrap to first pre-trigger data
      if (nextSec == t->secNum) {
        nextSec = t->preTriggerBlockSecNum + 1;
      }
      // Back at the first read data; jump to the first trigger data.
      if (nextSec == t->prevWPSecNum + 1) {
        nextSec = t->secNum + 1;
      }
    } else {
      // The trigger block; jump to first trigger data
      if (nextSec == t->secNum) {
        nextSec = t->secNum + 1;
      }
    }
  }

  // Wrap around if the sector is outside the disk
  if (nextSec == t->trackEnd) {
    nextSec = t->trackStart + 8; //FIRST_AVAILABLE_SECTOR;
  }
  //PRINTF(("Sector: %lu -> %lu", currentDataSec, nextSec));
  return nextSec;

}

// Get the first data sector from a trigger
uint32_t GetFirstDataSector(Trigger *t)
{
  uint32_t nextSec = 0;

  //Pre-trigger
  if (t->preTriggerBlockSecNum) {
    // Has the pre-trigger wrapped?
    if (t->preTriggerIsWrapped) {
      // The sector AFTER (wrapped) the latest written block contains the
      // oldest data
      nextSec = t->prevWPSecNum + 1;
      // Wrap around if needed
      if (nextSec == t->secNum) {
        nextSec = t->preTriggerBlockSecNum + 1;
      }
    } else {
      // The actual pre-trigger block does not contain data
      nextSec = t->preTriggerBlockSecNum + 1;
    }
  } else {
    // Only trigger; trigger block does not contain data
    nextSec = t->secNum + 1;
  }

  // Wrap around if the sector is outside the disk
  if (nextSec == t->trackEnd) {
    nextSec = t->trackStart + 8; //FIRST_AVAILABLE_SECTOR;
  }

  return nextSec;
}

// Check if a data sector is a pre-trigger data sector
int isPretriggerData(Trigger *t,uint32_t secNum) {
  if (!t->preTriggerBlockSecNum) {
    return 0;
  }

  if(t->preTriggerBlockSecNum < secNum && secNum < t->secNum)
  {
    return 1;
  }
  if (t->trackIsWrapped && t->secNum < t->preTriggerBlockSecNum) {
    if ((t->preTriggerBlockSecNum < secNum && secNum < t->trackEnd) ||
         (t->trackStart + 7 < secNum && secNum < t->secNum))
    {
      return 1;
    }
  }
  return 0;
}

// Get the number of sectors used by a trigger (incl. housekeeping)
uint32_t getNoSectors(TriggerList *list, int triggerNo) {
  Trigger  *t = NULL;
  Trigger  *tc = NULL;
  uint32_t startSector = 0;
  uint32_t endSector = 0;

  t = GetTrigger(list, triggerNo);
  tc = t;
  if (t == NULL) {
    PRINTF(("getNoSectors: Could not open trigger %u.\n", triggerNo));
    return 0;
  }

  // Get the start sector
  if (t->preTriggerBlockSecNum) {
    startSector = t->preTriggerBlockSecNum + 2; // Two trigger blocks
  } else {
    startSector = t->secNum + 1; // One trigger block
  }

  // The end sector is given by the next trigger
  // or the last written sector.
  t = GetTrigger(list,triggerNo + 1);

  if (t == NULL) {
    PRINTF(("tc->track %d gLastKnownWPSecNum: %u\n",tc->track,tc->lastWrittenSector));
    endSector = tc->lastWrittenSector;
  } else if (t->track != tc->track) {
    PRINTF(("Next trigger belongs to next track.\n"));
    endSector = tc->lastWrittenSector;
  }
  else {
    if (t->preTriggerBlockSecNum) {
      endSector = t->preTriggerBlockSecNum - 1;
    }
    else {
      endSector = t->secNum - 1;
    }
  }
  PRINTF(("startSector %u endSector: %u\n", startSector,endSector));
  if (startSector > endSector ) {
    if (tc->trackIsWrapped) {
      // Wrapped
      return tc->trackEnd - startSector - 1 + endSector - (tc->trackStart + 8) + 2;
    } else {
      // Corner case 01: Log with pretrigger and get a trigger created by a timer
      // without a single log message.
      return 0;
    }
  } else {
    return endSector - startSector + 1;
  }
}

// Get the last of the sectors used by a trigger (incl. housekeeping?)
uint32_t getEndSector(TriggerList *list, int triggerNo) {
  Trigger  *t = NULL;
  Trigger  *tc = NULL;
  uint32_t endSector = 0;

  t = GetTrigger(list, triggerNo);
  tc = t;
  if (t == NULL) {
    PRINTF(("getEndSector: Could not open trigger %u.\n", triggerNo));
    return 0;
  }

  // The end sector is given by the next trigger or the last written sector.
  t = GetTrigger(list, triggerNo + 1);

  if (t == NULL) {
    PRINTF(("tc->track %d gLastKnownWPSecNum: %u\n",
            tc->track, tc->lastWrittenSector));
    endSector = tc->lastWrittenSector;
  } else if (t->track != tc->track) {
    PRINTF(("Next trigger belongs to next track.\n"));
    endSector = tc->lastWrittenSector;
  }
  else {
    if (t->preTriggerBlockSecNum) {
      endSector = t->preTriggerBlockSecNum - 1;
    }
    else {
      endSector = t->secNum - 1;
    }
  }

  if (endSector == LIO_FIRST_AVAILABLE_SECTOR - 1) {
    // The first trigger after a user enabled FIFO mode on a full SD card
    endSector = tc->lastWrittenSector;
  }
  PRINTF(("endSector: %u\n", endSector));
  return endSector;
}

