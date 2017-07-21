#!/bin/sh
RETCODE=1
PROGPATH="/home/MMITSS-CA/bin"
PROGNAME="msgTransceiver"
CNFFILE="/home/MMITSS-CA/conf/rsu.conf"
PROGARGS="-s $CNFFILE"

if [ ! -e "$PROGPATH/$PROGNAME" ]; then
	echo "$PROGPATH/$PROGNAME not exit!"
	exit 1
fi

while [ 1 ];do
	ps -ef | grep "$PROGNAME" | grep -v grep > /dev/null
	RETCODE=$?
	if [ $RETCODE -eq 1 ]; then
		echo "starting $PROGNAME"
		exec "$PROGPATH/$PROGNAME" $PROGARGS &
	fi
  sleep 30s
done
