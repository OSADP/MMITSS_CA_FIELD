#!/bin/sh
PROGNAME_SCRIPT="start-mrp.sh"
PROGNAME_TCI="tci"
PROGNAME_MGR="dataMgr"
PROGNAME_AWR="mrpAware"

echo $(date +"%x %r") "stop $PROGNAME_SCRIPT"
/usr/bin/killall  -TERM "$PROGNAME_SCRIPT"
sleep 1s
echo $(date +"%x %r") "stop $PROGNAME_TCI"
/usr/bin/killall  -TERM "$PROGNAME_TCI"
sleep 1s
echo $(date +"%x %r") "stop $PROGNAME_MGR"
/usr/bin/killall  -TERM "$PROGNAME_MGR"
sleep 1s
echo $(date +"%x %r") "stop $PROGNAME_AWR"
/usr/bin/killall  -TERM "$PROGNAME_AWR"
sleep 1s
