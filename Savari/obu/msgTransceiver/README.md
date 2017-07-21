# About

This directory includes C++ source code for component OBU_msgTransceiver (Message Transceiver).
OBU_msgTransceiver and OBU_Aware are the two MMITSS-CA components that runs on the OBU. They communicate
with each other to transmit and receive over-the-air messages.

# Build and Install

This directory is included in the top-level (directory '/home/MMITSS-CA/obu') Makefile and does not
need to build manually. After the compilation process, an executable file ('msgTransceiver') is
created in the '/home/MMITSS-CA/obu/bin' directory.

See README in 'Savari/MMITSS-CA/obu' directory for steps to build and install OBU_msgTransceiver on OBU.

# Functions

The OBU_msgTransceiver component provides the following functions:
1. register the application as a User or Provider, depending on the configuration, to the WME stack;
2. when received an UDP packet from the OBU_Aware (i.e., BSM and SRM payload), manage to broadcast
the messages over-the-air; and
3. when received an WMSP packet from the WME stack, send the payload (i.e., MAP, SPaT and SSM) to OBU_Aware.
