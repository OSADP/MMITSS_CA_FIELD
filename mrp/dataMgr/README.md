# About

This directory includes C++11 source code for component MRP_DataMgr (Data Manager). It serves
as a data bridge between MRP components and non-MRP components (e.g., RSU, cloud server, etc).

# Build and Install

This directory is included in the top-level (directory 'mrp') Makefile and does not need to
build manually. After the compilation process, an executable file ('dataMgr') is created in
the 'dataMgr/obj' subdirectory.

# Functions

The MRP_DataMgr component provides the following functions:
1. Receive BSM and SRM messages from RSU msgTransceiver, pre-process and distribute the messages to the appropriate MRP components;
2. Receive pedestrian SRMs from the cloud server, validate and then forward the messages to MRP_Aware;
3. Receive SPaT data from MRP_TCI, encode and send SPaT to RSU msgTransceiver and the cloud server;
4. Read the intersection MAP description file, encode and send MapData to RSU msgTransceiver and the cloud server;
5. Receive loop detector count and occupancy data from MRP_TCI; receive vehicle trajectory data from MRP_Aware,
and calculates intersection performance measures;
6. Receive encoded SSMs from MRP_Aware, and forward to RSU msgTransceiver;
7. Receive MMITSS traffic and priority control commands from MRP_Aware, and forward the messages to MRP_TCI; and
8. Respond to polling requests from other MRP components regarding the shared data stored in MRP_DataMgr.
