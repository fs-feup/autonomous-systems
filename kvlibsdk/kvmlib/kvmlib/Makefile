#
#             Copyright 2017 by Kvaser AB, Molndal, Sweden
#                         http://www.kvaser.com
#
#  This software is dual licensed under the following two licenses:
#  BSD-new and GPLv2. You may use either one. See the included
#  COPYING file for details.
#
#  License: BSD-new
#  ==============================================================================
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in the
#        documentation and/or other materials provided with the distribution.
#      * Neither the name of the <organization> nor the
#        names of its contributors may be used to endorse or promote products
#        derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
#  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#
#  License: GPLv2
#  ==============================================================================
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
#
#
#  IMPORTANT NOTICE:
#  ==============================================================================
#  This source code is made available for free, as an open license, by Kvaser AB,
#  for use with its applications. Kvaser AB does not accept any liability
#  whatsoever for any third party patent or other immaterial property rights
#  violations that may result from any usage of this source code, regardless of
#  the combination of source code and various applications that it can be used
#  in, or with.
#
#  -----------------------------------------------------------------------------
#

include version.mk

ARCH := $(shell getconf LONG_BIT)

KVMLIB_COMMON     = ../common
KVMLIB_COMMON_SRC = $(KVMLIB_COMMON)/src

SDK_COMMON     = ../../common
SDK_COMMON_SRC = $(SDK_COMMON)/src
SDK_COMMON_INC = $(SDK_COMMON)/inc

include $(SDK_COMMON)/make/gflags.mk

KVLCLIB       = ../../kvlclib
KVADBLIB      = ../../kvadblib/so.ndb
MEMOLIB       = ../memolib_fd/so.ndb
MEMOLIBWRAP   = ../memolib_wrap/so.ndb

CFLAGS  += -Wall -Werror -Wextra  -DCPU=$(ARCH)  -O2 -fomit-frame-pointer $(XTRA_CFLAGS)
CFLAGS  += $(ccflags-y)
CFLAGS  += -D_FILE_OFFSET_BITS=64
CFLAGS  += -I. -I$(SDK_COMMON)/inc -I$(KVLCLIB) -I../include -I$(KVMLIB_COMMON_SRC) -I../../include
CFLAGS  += -DLINUX

ifeq ($(KV_DEBUG_ON),1)
  CFLAGS += -O0 -g
endif

CPPFLAGS  = $(CFLAGS)
CPPFLAGS += -std=c++11

OBJDIR = so.ndb

# Flags for reentrant, position independent code
LIBCFLAGS = $(CFLAGS) -D_REENTRANT -fPIC
LDFLAGS   = -lc -lstdc++ -L./$(OBJDIR) -L$(MEMOLIBWRAP) -ldl
LOADPATH  = ./$(OBJDIR):$(MEMOLIBWRAP):$(MEMOLIB)

LIBRARY := $(LIBNAME).$(MAJOR).$(MINOR).$(BUILD)
SONAME  := $(LIBNAME).$(MAJOR)

SRCS := kvmlib.cpp
SRCS += filehandler.cpp
SRCS += versionhandler.cpp

#files from $(SDK_COMMON_SRC)
SRCS += TimeConv.cpp
SRCS += kvdebug.cpp

#files from $(KVMLIB_COMMON_SRC)
SRCS += util.cpp

#files from $(KVLCLIB)
SRCS += KvaLogReader.cpp
SRCS += KvaConverterMisc.cpp
SRCS += KvaReaderMaker.cpp
SRCS += KvaProperties.cpp
SRCS += KvaLogReader_Kme24.cpp
SRCS += KvaLogReader_Kme25.cpp
SRCS += KvaLogReader_Kme40.cpp
SRCS += KvaLogReader_Kme50.cpp
SRCS += KvaLogReader_Kme60.cpp
SRCS += KvaLogReader_memoLogEvent.cpp
SRCS += SimpleMap.cpp
SRCS += KvaLogWriter.cpp
SRCS += KvaWriterMaker.cpp
SRCS += KvaLogWriter_Kme24.cpp
SRCS += KvaLogWriter_Kme25.cpp
SRCS += KvaLogWriter_Kme40.cpp
SRCS += KvaLogWriter_Kme50.cpp
SRCS += KvaLogWriter_Kme60.cpp
SRCS += KvaLogWriter_PlainAsc.cpp

#look for .cpp-files and .c-files here
vpath %.cpp $(SDK_COMMON_SRC):$(KVMLIB_COMMON_SRC):$(KVLCLIB)
vpath %.c   $(SDK_COMMON_SRC):$(KVMLIB_COMMON_SRC):$(KVLCLIB)

OBJS  = $(patsubst %.cpp, $(OBJDIR)/%.o, $(filter %.cpp, $(SRCS)))
DEPS  = $(patsubst %.cpp, $(OBJDIR)/%.d, $(filter %.cpp, $(SRCS)))
OBJS += $(patsubst %.c,   $(OBJDIR)/%.o, $(filter %.c,   $(SRCS)))
DEPS += $(patsubst %.c,   $(OBJDIR)/%.d, $(filter %.c,   $(SRCS)))

.PHONY: all uninstall clean install check

all: $(OBJDIR) $(OBJDIR)/$(LIBRARY)

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR)/$(LIBRARY): $(OTHERDEPS) $(OBJS) $(OBJDIR)
	@echo --------------------------------------------------------------------
	@echo "Building $(LIBRARY) $(IS_DEBUG)"
	$(CC) $(CPPFLAGS) -shared -Wl,-soname,$(SONAME) -o $@ $(OBJS) -Wl,-Bstatic -fPIC -Wl,-rpath=. -L$(KVADBLIB) -lkvadblib -Wl,-Bdynamic $(LDFLAGS) -lkvamemolib
	ln -sf $(LIBRARY) $(OBJDIR)/$(LIBNAME)
	ln -sf $(LIBRARY) $(OBJDIR)/$(SONAME)
	@echo --------------------------------------------------------------------

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) $(LIBCFLAGS)  -c -o $@ $<
	$(CC) $(CFLAGS) $(LIBCFLAGS) -MM $< | sed -r '1 s/^/$(OBJDIR)\//' > $(patsubst %.c, $(OBJDIR)/%.d, ${<F})

$(OBJDIR)/%.o: %.cpp
	$(CC) $(CPPFLAGS) $(LIBCFLAGS)  -c -o $@ $<
	$(CC) $(CPPFLAGS) $(LIBCFLAGS) -MM $< | sed -r '1 s/^/$(OBJDIR)\//' > $(patsubst %.cpp, $(OBJDIR)/%.d, ${<F})

$(OBJDIR)/check.o: check.cpp
	$(CC) $(CPPFLAGS) $(LIBCFLAGS)  -c -o $@ $<
	$(CC) $(CPPFLAGS) $(LIBCFLAGS) -MM $< | sed -r '1 s/^/$(OBJDIR)\//' > $(patsubst %.cpp, $(OBJDIR)/%.d, ${<F})

$(OBJDIR)/check: $(OBJDIR)/check.o
	$(CC) $(CPPFLAGS) $(LIBCFLAGS) check.cpp -o $@ $(LDFLAGS) -L$(MEMOLIB) -lkvmlib -lkvamemolib -lkvamemolib0700

check: $(OBJDIR)/check
	@echo --------------------------------------------------------------------
	@echo "Running test program"
	@echo --------------------------------------------------------------------
	LD_LIBRARY_PATH=$(LOADPATH) $(OBJDIR)/check

clean:
	rm -rf $(OBJDIR)

install:
	rm -f $(DESTDIR)$(libdir)/$(LIBNAME)
	rm -f $(DESTDIR)$(libdir)/$(SONAME)
	rm -f $(DESTDIR)$(libdir)/$(LIBRARY)
	$(INSTALL_LIB) -D $(OBJDIR)/$(LIBRARY) $(DESTDIR)$(libdir)/$(LIBRARY)
	ln -sf $(LIBRARY) $(DESTDIR)$(libdir)/$(LIBNAME)
	ln -sf $(LIBRARY) $(DESTDIR)$(libdir)/$(SONAME)
	$(LDCONFIG)

	#change name on pshpack1 and poppack to kvpshpack and kvpoppack
	cp -p ../../include/kvmlib.h            tmp_kvmlib.h
	sed -r -i 's,pshpack1\.h,kvpshpack\.h,' tmp_kvmlib.h
	sed -r -i 's,poppack\.h,kvpoppack\.h,'  tmp_kvmlib.h

	$(INSTALL_DATA) -D tmp_kvmlib.h $(DESTDIR)$(includedir)/kvmlib.h
	$(INSTALL_DATA) -D $(SDK_COMMON_INC)/poppack.h $(DESTDIR)$(includedir)/kvpoppack.h
	$(INSTALL_DATA) -D $(SDK_COMMON_INC)/pshpack1.h $(DESTDIR)$(includedir)/kvpshpack.h

uninstall:
	rm -f $(DESTDIR)$(libdir)/$(LIBNAME) $(DESTDIR)$(libdir)/$(SONAME) $(DESTDIR)$(libdir)/$(LIBRARY)
	$(LDCONFIG)
	rm -f $(DESTDIR)$(includedir)/kvmlib.h
	rm -f $(DESTDIR)$(includedir)/kvpoppack.h
	rm -f $(DESTDIR)$(includedir)/kvpshpack.h

include $(SDK_COMMON)/make/pkgconfig.mk

-include $(DEPS)

