#!/bin/sh
RETCODE=1
PROGPATH="/root"
PROGNAME="obuAware"
PROGARGS="-s $PROGPATH/obuSocket.conf -c $PROGPATH/vinBus1.conf -f 2 -d"
# PROGARGS -f 0 = no log; 1 = simple log; 2 = detailed log
#          -d = turn on sending display udp packet
#          -v turn on verbose

while [ 1 ];do
  if [ -e "$PROGPATH/$PROGNAME" ];then
    ps -ef | grep "$PROGNAME" | grep -v grep > /dev/null
    RETCODE=$?  
    if [ $RETCODE -eq 1 ]; then
      echo "starting $PROGNAME"
      exec "$PROGPATH/$PROGNAME" $PROGARGS &
      sleep 2s
    fi
  fi
  sleep 30s
done
