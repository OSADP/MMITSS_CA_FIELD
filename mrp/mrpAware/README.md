# About

This directory includes C++11 source code for component MRP_Aware, which provides the main
functions of MMITSS traffic and priority control.

# Build and Install

This directory is included in the top-level (directory 'mrp') Makefile and does not need to
build manually. After the compilation process, an executable file ('mrpAware') is created in
the 'mrpAware/obj' subdirectory.

# Functions

The MRP_Aware component provides the following functions:
1. Process BSMs, locate and track BSMs on MAP, determine vehicle's travel lane, distance- and time-to-arrival
at the stop-bar. When needed, command vehicular phase call or green extension to MRP_TCI via MRP_DataMgr;
2. Process pedestrian SRMs. When needed, command pedestrian phase call to MRP_TCI via MRP_DataMgr;
3. Process SRMs and associate SRMs with BSMs, determine the priority strategy, and command priority control to MRP_TCI via MRP_DataMgr;
4. Encodes and send SSM payload to RSU msgTransceiver via MRP_DataMgr; and
5. Process and send vehicle trajectory data to MRP_DataMgr.
