# About

This directory includes C++ source code for component RSU_msgTransceiver (Message Transceiver).
RSU_msgTransceiver is the only MMITSS-CA component that runs on the RSU. It communicates with
MRP_DataMgr running on the MRP PC to transmit and receive over-the-air messages.

# Build and Install

This directory is included in the top-level (directory '/home/MMITSS-CA/rsu') Makefile and does not
need to build manually. After the compilation process, an executable file ('msgTransceiver') is
created in the '/home/MMITSS-CA/rsu/bin' directory.

See README in 'Savari/MMITSS-CA/rsu' directory for steps to build and install RSU_msgTransceiver on RSU.

# Functions

The RSU_msgTransceiver component provides the following functions:
1. register the application as a User or Provider, depending on the configuration, to the WME stack;
2. when received an UDP packet from the MRP_DataMgr (i.e., SPaT, MAP, and SSM payload), manage to
broadcast the messages over-the-air; and
3. when received an WMSP packet from the WME stack, send the payload (i.e., BSM and SRM) to MRP_DataMgr.
