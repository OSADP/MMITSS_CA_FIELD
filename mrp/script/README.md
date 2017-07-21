# About

This directory includes Linux bash scripts to run MRP components.

# List of Files

 File               | Contents
 -------------------|-------------
 mmitss.mrp.service | File to enable running MRP executables as Systemd service 
 start-mrp.sh       | Linux shell script called by mmitss.mrp.service to start MRP executables 
 stop-mrp.sh        | Linux shell script called by mmitss.mrp.service to stop MRP executables

See [Build and Install] section of README in /home/MMITSS-CA/MRP directory for systemctl commands 
to start, stop, and restart mmitss.mrp.service. 
