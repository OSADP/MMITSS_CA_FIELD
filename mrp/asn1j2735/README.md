# About

This directory includes C++11 source code which provide library APIs for UPER (Unaligned Packed
Encoding Rules) encoding and decoding of J2735 messages, including Basic Safety Message (BSM),
Signal Phase and Timing Message (SPaT), MAP message, Signal Request Message (SRM), and Signal
Status Message (SSM).

The standard of SAE J2735 message sets is [version SAE J2735_201603](http://www.sae.org/standardsdev/dsrc/).

The library APIs can be used in a roadside unit (RSU) as well as in an on-board unit (OBU).

# Build and Install

This directory is included in the top-level (directory 'mrp') Makefile and does not need to build
manually. After the compilation process, a shared library ('libdsrc.so') is created in the
'asn1j2735/lib' subdirectory. User's application links the 'libdsrc.so' to access the UPER encoding
and decoding library APIs.

# Interface Functions

The interface functions are defined in 'AsnJ2735Lib.h', including
- a set of functions for UPER encoding of SAE J2735 messages (e.g., encode_spat_payload(), etc.); and
- a set of functions for UPER decoding of SAE J2735 messages (e.g., decode_spat_payload(), etc.);
