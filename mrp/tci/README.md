# About

This directory includes C++11 source code for component MRP_TCI (Traffic Controller Interface).
It supports bidirectional communications with Caltrans 2070 controller utilizing AB3418 protocol
over serial port connections.

# Build and Install

This directory is included in the top-level (directory 'mrp') Makefile and does not need to build
manually. After the compilation process, an executable file ('tci') is created in the 'tci/obj'
subdirectory.

# Functions

The MRP_TCI component provides the following functions:
1. Poll controller's configuration data, and populate Class Card data elements;
2. Read serial data output from the traffic signal controller, populate and send SPaT data elements to MRP_DataMgr;
3. Read loop detector count and occupancy data from the traffic signal controller, pack and send data messages to MRP_DataMgr; and
4. Process received MMITSS traffic and priority control command messages, pack and send control commands to the traffic signal controller.
