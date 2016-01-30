#!/bin/sh
RETCODE=1
PROGPATH="/home/mrp/bin"
CONFIGPATH="/home/mrp/conf"
PROGNAME="tci"
PROGARGS="-s $CONFIGPATH/tciSocket.conf -n `hostname` -c -f 2 -p"
#PROGARGS="-s $CONFIGPATH/tciSocket.conf -n `hostname` -f 2 -p"
# PROGARGS: -c turn on sendSoftcall to tci
#           -f turn on simple log (1) or detailed log (2)
#           -p turn on polling timing data (otherwise reading from file)
#           -v turn on verbose

/usr/bin/killall  -TERM "$PROGNAME"
sleep 1s

if [ -e "$PROGPATH/$PROGNAME" ];then
  while [ 1 ];do
    ps -ef | grep "$PROGNAME" | grep -v grep > /dev/null
    RETCODE=$?  
    if [ $RETCODE -eq 1 ]; then
      echo "Starting $PROGNAME"
      /usr/bin/screen -d -m "$PROGPATH/$PROGNAME" $PROGARGS
      sleep 2s
    fi
    sleep 60s
  done
else
  echo "$PROGPATH/$PROGNAME not found"
fi  