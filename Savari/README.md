# About

This directory includes C++ source code for RSU and OBU components that work with Savari devices.

The subdirectories include:

 Subdirectory   | Install Location        |  Contents
 ---------------|-------------------------|------------------
 **build**      | Savari SDK VMware image | Common definitions for RSU and OBU builds on Savari devices
 **obu**        | Savari SDK VMware image | Source code and build scripts for Savari RSU components
 **rsu**        | Savari SDK VMware image | Source code and build scripts for Savari OBU components

# Build (with Savari SOBOS SDK VMware Image) and Install

### Savari SOBOS SDK toolchain (required)

The executables running on RSU and OBU need to be built with Savari RSU and OBU toolchain, which are included
in Savari software development kit (SDK). Please contact [Savari](http://savari.net/) regarding the accessibility
of Savari SOBOS SDK and VMware image.

### Add common definitions for RSU and OBU builds (only need to be done once)
- open ~/.bashrc; add 'export SAVARI_MK_DEFS=/home/MMITSS-CA/build/Savari.mk'; and
- source ~/.bashrc

### Build and Install RSU and OBU components

See README in the 'obu' and 'rsu' subdirectories for instructions on build and install OBU and RSU components on
Savari devices.
