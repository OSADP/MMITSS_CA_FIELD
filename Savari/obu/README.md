# About

This directory includes C++ source code, build and run scripts for Savari OBU components OBU_msgTransceiver and OBU_Aware.

### Subdirectory and contents

 Subdirectory       | Install Location        |  Contents
 -------------------|-------------------------|------------------
 **bin**            | OBU                     | Executable and shell script to run OBU_msgTransceiver and OBU_Aware as service
 **conf**           | OBU                     | Configuration files for OBU_msgTransceiver and OBU_Aware
 **msgTransceiver** | Savari SDK VMware image | Source code for OBU_msgTransceiver components
 **obuAware**       | Savari SDK VMware image | Source code for OBU_Aware components

# Build (with Savari SOBOS SDK VMware Image)

### Version of Savari ASD SDK toolchain for this build
- Version 3.1.2.

### Steps for OBU_msgTransceiver and OBU_Aware build (in the VMware machine) includes:

1. In the VMware machine, mkdir /home/MMITSS-CA/obu;
2. download or copy 'savari/build' to /home/MMITSS-CA/;
3. download or copy 'savari/obu/msgTransceiver', 'savari/obu/obuAware', and 'savari/obu/Makefile' to /home/MMITSS-CA/obu;
4. cd /home/MMITSS-CA/obu; make all; make install; and
5. copy '/home/MMITSS-CA/obu/bin/msgTransceiver' and '/home/MMITSS-CA/obu/bin/obuAware' to savari/obu/bin/.

# Install OBU_msgTransceiver and OBU_Aware on OBU

1. In OBU, mkdir '/home/MMITSS', '/nojournal/mmitss_logs/wme', and '/nojournal/mmitss_logs/wme';
2. copy (with SCP) 'savari/obu/bin' and 'savari/obu/conf' to /home/MMITSS; chmod +x /home/MMITSS/obu/*;
3. copy '/home/MMITSS/bin/msgTransMon' and '/home/MMITSS/bin/awareMon' to /etc/init.d/; and
4. in /etc/rc.d/, create a symbolic link 'ln -s ../msgTransMon S96msgTransMon', and 'ln -s ../awareMon S99awareMon'

# Start and Stop 'msgTransceiver' and 'obuAware' on OBU

'msgTransceiver' and 'obuAware' will be start on OBU reboot.

To manually start or stop 'msgTransceiver':
1. /etc/init.d/msgTransMon start; or
2. /etc/init.d/msgTransMon stop

To manually start or stop 'obuAware':
1. /etc/init.d/awareMon start; or
2. /etc/init.d/awareMon stop
