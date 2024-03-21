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
#ifndef _BUF_QUEUE_H_
#define _BUF_QUEUE_H_

/*****************************************************************************

  BQueue is a queue where you can push elements until you want to read them,
  and once you begin reading them it's write protected until it's empty.

 *****************************************************************************/

class BQueue {
private:
  unsigned int size_of_queue;   // total number of elements in the queue in
                                // memory
  unsigned int size_of_element; // the size of one element in bytes
  char *storage;                // storage where everthing is stored
  unsigned int head;            // keeps track of the head in the queue
  unsigned int current_size;    // keeps track of current size in memory
  unsigned int total_size;      // Total size of queue
  unsigned int nof_blocks;      // Number of memory blocks written in file
  unsigned int current_block;   // Index of current block in file

  FILE *tmp_file;               // File where temporary blocks are stored
  bool open_temp_file(char *prefix);
  bool write_block();           // Writes and empties the current memory to
                                // tmp_file
  bool read_block();            // Reads one block from tmp_file, overwrites
                                // everything in storage
  bool read_only;               // True if the queue is in read only mode

  char *filename;               // Name of tmp-file
  char file_prefix[50];         // Prefix of tmp-file

public:
  BQueue(char *tmp_prefix, unsigned int queue_size, unsigned int element_size);
  ~BQueue();                    // Remove the queue
  bool push(void *mem);         // A copy get placed in the end of the line
  bool pop(void *mem);          // The head of the line gets chopped off
  //void front(void *mem);      // Mem gets a copy of the front of the line
  unsigned int size();          // Returns the current size
};


#endif // _BUF_QUEUE_H_
