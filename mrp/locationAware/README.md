# About

This directory includes C++11 source code which provide library APIs for locating BSMs on MAP,
determining the upcoming signalized intersection, vehicle's travel lane and the signal group
that controls vehicle's movement, and determining the distance to the stop-bar.

The library APIs can be used in a roadside unit (RSU) as well as in an on-board unit (OBU).

# Build and Install

This directory is included in the top-level (directory 'mrp') Makefile and does not need to build
manually. After the compilation process, a shared library ('liblocAware.so') is created in the
'locationAware/lib' subdirectory. User's application links the 'liblocAware.so' to access the
aforementioned library functions.

# Interface Functions

The interface functions are defined in 'locAware.h', including
1. locate and track vehicle's BSMs on MAP;
2. determine the upcoming MMITSS intersection, travel lane and associated signal group; and
3. determine distance and travel time to the stop-bar.
