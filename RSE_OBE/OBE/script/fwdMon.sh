#!/bin/sh
RETCODE=1
PROGPATH="/root"
PROGNAMETX="wmetx"
PROGNAMERX="wmerx"
TXPROGARGS="-c $PROGPATH/obuWmefwd.conf -f 2"
RXPROGARGS="-c $PROGPATH/obuWmefwd.conf -f 2 -d"
# PROGARGS  -f 0 = no log; 1 = simple log; 2 = detailed log
#           -v turn on verbose
# for wmerx: -d = turn on sending udp packet to EAR application (default turn on)

while [ 1 ];do
  if [ -e "$PROGPATH/$PROGNAMERX" ];then
    ps -ef | grep "$PROGNAMERX" | grep -v grep > /dev/null
    RETCODE=$?  
    if [ $RETCODE -eq 1 ]; then
      echo "starting $PROGNAMERX"
      exec "$PROGPATH/$PROGNAMERX" $RXPROGARGS &
      sleep 2s
    fi
  fi
  if [ -e "$PROGPATH/$PROGNAMETX" ];then
    ps -ef | grep "$PROGNAMETX" | grep -v grep > /dev/null
    RETCODE=$?  
    if [ $RETCODE -eq 1 ]; then
      echo "starting $PROGNAMETX"
      exec "$PROGPATH/$PROGNAMETX" $TXPROGARGS &
      sleep 2s
    fi
  fi
  sleep 30s
done
