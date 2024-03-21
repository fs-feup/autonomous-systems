                     ______________________________________

                      KVASER LIBRARY (BINARY DISTRIBUTION)
                     ______________________________________


Table of Contents
_________________

1 Contents
2 Requirements
3 Install and Uninstall


This software is furnished under a license and may be used and copied
only in accordance with the terms of such license. See file COPYING.


1 Contents
==========

  This package contains the follwowing Kvaser libraries for use in a
  Linux environment:

  libkvmlib: Kvaser Memorator Library. Library for accessing Kvaser Memorator
             (2nd generation) devices. This library is used to extract log
             data, initialize disk, read and write configuration to a device,
             handle on device databases and more. Includes libkvamemolib and
             libkvamemolib0700.

  libkvamemolibxml: Kvaser Memorator Configuration XML. This library is used
                    for converting XML settings to a format for use by Kvaser
                    Memorator (2nd Generation) devices.

  libkvadblib: Interface to CAN Database. This library is used to create and
               access CAN databases.


2 Requirements
==============

  The package pkg-config is needed by all Kvaser libraries.

  The package libxml2 is needed by libkvamemolibxml.

  The package zlib1g is needed by kvlclib.


3 Install and Uninstall
=======================

  To install everything, run

    sudo make install

  To uninstall everything, run

    sudo make uninstall

