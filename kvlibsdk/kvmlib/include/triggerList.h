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
** Put measurements into a linked list based on the trigger blocks used
** -----------------------------------------------------------------------------
*/

#ifndef TRIGGER_LIST_H
#define TRIGGER_LIST_H
#include <stdint.h>
#include "kvdebug.h"

/*
 The layout of the disk and triggers

 |---------------- SD Disk --------------------|
 |------ Track 0 ------||------- Track 1 ------|
 |TxxxxXTxxxXTxxxxxW---||PxXxxTxxxxPxxXTxxxW---|

 T = Trigger  x = Data sector W = writepointer
 P = Pre-trigger X = prevWPSecNum - = free sector
*/
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  uint32_t secNum;                    // Location of this trigger
  uint32_t prevTriggerBlockSecNum;    // Location of the previous trigger
  uint32_t prevWPSecNum;              // Previous location of write pointer, before trigger
  uint32_t preTriggerBlockSecNum;     // Location of pre-trigger
  uint8_t  preTriggerIsWrapped;       // The pre-trigger has wrapped
  uint8_t  isHkTrigger;               // This trigger is made from a housekeeping block
  uint16_t track;                     // Track number
  uint32_t trackStart;                // The sector on the disk where the track begins
  uint32_t trackEnd;                  // The sector on the disk where the track ends
  uint8_t  trackIsWrapped;            // Measurements have wrapped on this track
  uint32_t lastWrittenSector;         // The last written sector
  uint32_t preTriggerSectors;         // The number of sectors in the pre-trigger
  uint32_t serialNumber;              // The serialnumber of the device that created the trigger
  uint32_t eanHi;                     // EAN high bytes
  uint32_t eanLo;                     // EAN low bytes
} Trigger;

// The item in the list that contains the trigger data
typedef struct TriggerItem{
  struct TriggerItem *prev;
  struct TriggerItem *next;
  Trigger  trigger;
} TriggerItem;

// The list to store the items
typedef struct {
  TriggerItem *first;
  TriggerItem *last;
  int count;
  int tracks;
  uint32_t *trackStart;
  uint32_t *trackEnd;
  uint16_t numOfKMFFiles;
  uint32_t fileSize;
} TriggerList;

// Copy a trigger to a trigger list.
void CopyTriggerToList(TriggerList *list, Trigger *trigger);

// Delete all items in a list and free their memory
void DeleteTriggerList(TriggerList *list);

// Get a pointer to the selected trigger block counting from the end of the list.
// This number matches the file number used in kvaMemolib.
Trigger* GetTrigger(TriggerList *list, int triggerNo);

// Get the first data sector in a trigger
uint32_t GetFirstDataSector(Trigger *t);

// Get the next data sector in a trigger, given the the current sector
uint32_t GetNextDataSector(Trigger *t, uint32_t currentDataSec);

// Get the number of sectors used by a trigger (incl. housekeeping)
uint32_t getNoSectors(TriggerList *list, int triggerNo);

// Get the last of the sectors used by a trigger (incl. housekeeping?)
uint32_t getEndSector(TriggerList *list, int triggerNo);

// Check if a sector belongs to the pre-trigger
int isPretriggerData(Trigger *t,uint32_t secNum);

// Print the triggers in a list (for debug)
void PrintReverseTriggerList(TriggerList *list);

// Counter the number of triggers that belong track
int countTriggersOnTrack(TriggerList *list, int track);

#ifdef __cplusplus
}
#endif

#endif // TRIGGER_LIST_H
