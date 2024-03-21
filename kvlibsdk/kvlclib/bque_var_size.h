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

#ifndef BQUE_VAR_SIZE_H
#define BQUE_VAR_SIZE_H

#include <vector>

/*****************************************************************************

  bque_var_size is a buffered queue where you can push elements with VARIABLE SIZE until
  you want to read them, and once you begin reading them it's write protected until
  it's empty.

*****************************************************************************/

class bque_var_size {
  private:
    char *storage;                       // storage where everthing is stored until
                                         // we write to file
    unsigned int storage_size;           // the size, in bytes, of storage
    std::vector<unsigned int>block_size; // how many storage's have we written to file
                                         // and the size for each of them


    FILE *tmp_file;                      // File where storage's are stored
    char *filename;                      // Name of tmp-file
    char file_prefix[50];                // Prefix of tmp-file

    bool read_only;                      // True if the queue is in read only mode
    unsigned int push_index;             // where to write next data in storage
    unsigned int pop_index;              // where to read next data from storage

    unsigned int n_bytes_read;           // how many bytes have we read from file
    unsigned int n_pushed_element;       // number of elements we have saved

    bool open (char *prefix);            // opens a file for temporary storage
    void close ();                       // closes the file above
    bool write_block();                  // Writes and empties the current memory to
                                         // tmp_file
    bool read_block();                   // Reads one block from tmp_file, overwrites
                                         // everything in storage

  public:
    bque_var_size (unsigned int n_element, unsigned int max_element_size, char *name);
    //store "n_element" with "max_element_size" and use a file named "name" as a buffer

    ~bque_var_size ();

    bool push (void *data, unsigned int n_bytes);

    bool pop (void *data);  //NOTE!!!! data shall hold at least "max_element_size" bytes

    unsigned int get_n_element();
};

#endif
