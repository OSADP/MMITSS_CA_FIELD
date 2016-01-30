#!/bin/sh
RETCODE=1
PROGPATH="/root"
PROGNAMETX="wmetx"
PROGNAMERX="wmerx"
GREPARGS="rsuWmefwd.conf"
PROGARGS="-c $PROGPATH/rsuWmefwd.conf -f 2"
# PROGARGS  -f 0 = no log file; 1 = simple log; 2 = detailed log
#           -v turn on verbose

while [ 1 ];do
  if [ -e "$PROGPATH"/"$PROGNAMETX" ];then
    ps -ef | grep "$PROGNAMETX" | grep -v grep > /dev/null
    RETCODE=$?  
    if [ $RETCODE -eq 1 ]; then
      echo "starting $PROGNAMETX"
      exec "$PROGPATH/$PROGNAMETX" $PROGARGS &
      sleep 2s
    fi
  fi
  if [ -e "$PROGPATH"/"$PROGNAMERX" ];then
    ps -ef | grep "$PROGNAMERX" | grep -v grep > /dev/null
    RETCODE=$?  
    if [ $RETCODE -eq 1 ]; then
      echo "starting $PROGNAMERX"
      exec "$PROGPATH/$PROGNAMERX" $PROGARGS &
      sleep 2s
    fi
  fi
  sleep 30s
done
