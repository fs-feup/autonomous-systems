                          ____________________

                           KVASER LIBRARY SDK
                          ____________________


Table of Contents
_________________

1 Contents
2 Requirements
3 Build and Install
4 Other available targets


This software is furnished under a license and may be used and copied
only in accordance with the terms of such license. See file COPYING.


1 Contents
==========

  This package contains the follwowing Kvaser libraries for use in a
  Linux environment:

  kvmlib: Kvaser Memorator Library. Library for accessing Kvaser Memorator
          (2nd generation) devices. This library is used to extract log
          data, initialize disk, read and write configuration to a device,
          handle on device databases and more.
          Installed library files:
            - /usr/lib/libkvamemolib700.so
            - /usr/lib/libkvamemolib.so
            - /usr/lib/libkvmlib.so

  kvamemolibxml: Kvaser Memorator Configuration XML. This library is used
                 for converting XML settings to a format for use by Kvaser
                 Memorator (2nd Generation) devices.
                 Installed library files:
                   - /usr/lib/libkvamemolibxml.so

  kvadblib: Interface to CAN Database. This library is used to create and
            access CAN databases.
            Installed library files:
              - /usr/lib/libkvadblib.so

  kvlclib: Kvaser Logfile Converter Library. This library is used to
           convert CAN log files to other formats, e.g. ASCII, CVS,
           KME50 and MDF4.
           Installed library files:
             - /usr/lib/libkvlclib.so


2 Requirements
==============

  The package pkg-config is needed by all Kvaser libraries.

  The packages libxml2-dev pkg-config are needed in order to build kvamemolibxml.

  The package zlib1g-dev is needed to build kvlclib.



3 Build and Install
===================

  To build everything, run

    make

  To run self-tests :

    make check

  To install everything, run

    sudo make install

  To uninstall everything, run

    sudo make uninstall

  To include gdb debug information on the generated binaries, run

    make KV_DEBUG_ON=1


4 Other available targets
=========================

  Builds a binary tar file with only .so and .h files.

    make
    make dist

  Removes all generated files.

    make clean
