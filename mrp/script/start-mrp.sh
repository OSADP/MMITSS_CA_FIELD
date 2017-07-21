#!/bin/sh
MRP_PATH="/home/MMITSS-CA/mrp"
MRP_BIN_PATH="$MRP_PATH/bin"
MRP_CONF_PATH="$MRP_PATH/conf"
host=$(hostname)
PROGNAME_TCI="tci"
PROGARGS_TCI="-n $host -s $MRP_CONF_PATH/mrpTci.conf -v"
PROGNAME_MGR="dataMgr"
PROGARGS_MGR="-n $host -s $MRP_CONF_PATH/dataMgr.conf -v"
PROGNAME_AWR="mrpAware"
PROGARGS_AWR="-n $host -s $MRP_CONF_PATH/mrpAwr.conf -v"

if [ ! -e "$MRP_BIN_PATH/$PROGNAME_TCI" ]; then
	echo "$MRP_BIN_PATH/$PROGNAME_TCI not exit!"
	exit 1
elif [ ! -e "$MRP_BIN_PATH/$PROGNAME_MGR" ]; then
	echo "$MRP_BIN_PATH/$PROGNAME_MGR not exit!"
	exit 1
elif [ ! -e "$MRP_BIN_PATH/$PROGNAME_AWR" ]; then
	echo "$MRP_BIN_PATH/$PROGNAME_AWR not exit!"
	exit 1
fi

while [ 1 ]; do
	ps -ef | grep "$PROGNAME_TCI" | grep -v grep > /dev/null
	RETCODE=$?
	if [ $RETCODE -eq 1 ]; then
		echo $(date +"%x %r") "start $PROGNAME_TCI"
		/usr/bin/screen -S $PROGNAME_TCI -d -m "$MRP_BIN_PATH/$PROGNAME_TCI" $PROGARGS_TCI
		sleep 2s
	fi

	ps -ef | grep "$PROGNAME_MGR" | grep -v grep > /dev/null
	RETCODE=$?
	if [ $RETCODE -eq 1 ]; then
		echo $(date +"%x %r") "start $PROGNAME_MGR"
		/usr/bin/screen -S $PROGNAME_MGR -d -m "$MRP_BIN_PATH/$PROGNAME_MGR" $PROGARGS_MGR
		sleep 2s
	fi

	ps -ef | grep "$PROGNAME_AWR" | grep -v grep > /dev/null
	RETCODE=$?
	if [ $RETCODE -eq 1 ]; then
		echo $(date +"%x %r") "start $PROGNAME_AWR"
		/usr/bin/screen -S $PROGNAME_AWR -d -m "$MRP_BIN_PATH/$PROGNAME_AWR" $PROGARGS_AWR
		sleep 2s
	fi

	sleep 60s
done
