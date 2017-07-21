# About

This directory includes C++ source code, build and run scripts for component (Savari) RSU_msgTransceiver.

RSU_msgTransceiver is the only MMITSS-CA software component that runs on the RSU.

### Subdirectory and contents

 Subdirectory       | Install Location        |  Contents
 -------------------|-------------------------|------------------
 **bin**            | RSU                     | Executable and shell script to run RSU_msgTransceiver as service
 **conf**           | RSU                     | Configuration file for RSU_msgTransceiver
 **msgTransceiver** | Savari SDK VMware image | Source code for RSU_msgTransceiver components

# Build (with Savari SOBOS SDK VMware Image)

### Version of Savari RSU SDK toolchain for this build
- Version 3.1.2.

### Steps for RSU_msgTransceiver build (in the VMware machine) includes:

1. In the VMware machine, mkdir /home/MMITSS-CA/rsu;
2. download or copy 'savari/build' to /home/MMITSS-CA/;
3. download or copy 'savari/rsu/msgTransceiver' and 'savari/rsu/Makefile' to /home/MMITSS-CA/rsu;
4. cd /home/MMITSS-CA/rsu; make all; make install; and
5. copy /home/MMITSS-CA/rsu/bin/msgTransceiver to savari/rsu/bin/.

# Install RSU_msgTransceiver on RSU

1. In RSU, mkdir /home/MMITSS-CA, and mkdir /nojournal/wmelogs;
2. copy (with SCP -r) 'savari/rsu/bin' and 'savari/rsu/conf' to /home/MMITSS-CA; chmod +x /home/MMITSS-CA/bin/*;
3. copy /home/MMITSS-CA/bin/msgTransMon to /etc/init.d/; and
4. in /etc/rc.d/, create a symbolic link 'ln -s etc/init.d/msgTransMon S99msgTransMon'

# Start and Stop 'msgTransceiver' on RSU

### Enable 'msgTransceiver' as start up service
To set 'msgTransceiver' to run on RSU reboot
- /etc/init.d/msgTransMon enable

### Manually Start 'msgTransceiver'
- /etc/init.d/msgTransMon start

### Manually Stop 'msgTransceiver'
- /etc/init.d/msgTransMon stop
