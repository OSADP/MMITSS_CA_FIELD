# About

This directory includes C++11 source code for software components of MRP (MMITSS Roadside Processor),
and configurations for running MRP software components. The sub-directories include (see README at
each subdirectory for more detailed descriptions):

 Subdirectory       | Contents
 ------------------ |-------------
 **asn1**           | source code generated by the open source ASN.1 compiler 'asn1c', based on SAE J2735 version J2735_201603_ASN.
 **asn1j2735**      | Library APIs for Unaligned Packed Encoding Rules (UPER) encoding and decoding of DSRC messages, including Basic Safety Message (BSM),Signal Phase and Timing Message (SPaT), MAP message, Signal Request Message (SRM), and Signal Status Message (SSM).
 **build**          | Common definitions for MRP builds on Linux-like systems
 **conf**           | Configuration files for software components hosted by the MRP machine
 **dataMgr**        | Source code for the MRP_DataMgr component (executable)
 **locationAware**  | Library APIs for locating BSMs on MAP, identifies vehicle's travel lane and signal group that controls vehicle's movement, and determines distance- and time-to-arrival at the stop-bar.
 **mrpAware**       | Source code for the MRP_Aware component (executable)
 **script**         | Linux shell scripts to start, stop executables hosted by the MRP machine
 **tci**            | Source code for the MRP_TCI component (executable)
 **utils**          | Library APIs for configuring of MRP software components, DSRC radio interface, and data logging, pack and unpack serialized UDP messages, Linux socket and timestamps utilities.

# Operating System and Compiler Version

The MMITSS-CA package should be run on Linux-like systems (e.g. the MRP computer).

The operating requirements on the Linux-based MRP computer are
- Processing power:  Intel Core i5 or equivalent
- Minimum memory:    2 GB
- Hard drive space:  100 GB or above
- Connectivity:      Ethernet, 2 RS232 ports
- Operating system:  Ubuntu 16.02, Linux Kernel 4.08 or above

The Linux-based MRP computer installed in the California Connected Vehicle testbed has
- CPU:               Intel Core i5-3570 CPU @ 3.40GHz
- Total memory:      4 GB
- Hard drive space:  500 GB
- Operating system:  Ubuntu 16.04.2 LTS, Linux Kernel 4.10.0-26-generic

# Build and Install

Source code of MMITSS-CA MRP components should be downloaded or copied to the directory /home/MMITSS-CA/mrp,
including ten (10) subdirectories as described in the [About] section above.

Makefile in this directory auto-builds all subdirectories, and configures running MRP executables as Systemd service.

### Add common definitions for MMITSS-CA builds (only need to be done once)
- open ~/.bashrc; add 'export MRP_MK_DEFS=/home/MMITSS-CA/mrp/build/mrp_linux.mk'; and
- source ~/.bashrc

### Enable running MRP executables as Systemd service (only need to be done once)
- open /home/MMITSS-CA/mrp/script/mmitss.mrp.service; under [Service], replace User and Group as desired;
- cd /home/MMITSS-CA/mrp; and
- make startup

### Match MRP computer Hostname with MAP_Name in the intersection nmap file
Using intersection of "El Camino Real and Page Mill Rd" as an example (see [California testbed intersection nmap files](conf/CAtestbed.nmap))
- MAP_Name ecr-california.nmap
- RSU_ID ecr-california
- MRP computer hostname should be **ecr-california** (hostname is used as an argument to MRP executables)
- Modify file /etc/hostname and /etc/hosts to change computer hostname

### Build and install MMITSS-CA MRP executables
- cd /home/MMITSS-CA/mrp;
- make all (for the first time); or
- make mrp (once the 'asn1' directory has been make earlier); and
- make install

### Start MMITSS-CA MRP service
- sudo systemctl restart mmitss.mrp.service

### Stop MMITSS-CA MRP service
- sudo systemctl stop mmitss.mrp.service