#!/bin/bash
WORK_DIR="./"
# $Id$
# Purpose: Start ntripclient
# change these 3 according to your needs
Stream=$(cat /tmp/mountpoint)
User='jaeho0608@hanyang.ac.kr'
# User='gnss'
Password='gnss'
# Password='mmc1898!!'
# Host='gnssdata.or.kr'
Host='www.gnssdata.or.kr'
Port='2101'
#Serial='-D /dev/ttyUSB1'
Serial="-D $(./grab_config.py config.ini DEFAULT DEV_RTCM)"
Baud=38400
mode='a'
seconds=120

SleepTime=2     # Wait sec for next reconnect try
(while true; do
  echo "running $seconds seconds duration, ./ntripclient -s $Host -r $Port -m $Stream -u $User -p $Password $Serial -B $Baud -M $mode "

  timeout $seconds ./ntripclient -s $Host -r $Port -m $Stream -u $User -p $Password $Serial -B $Baud -M $mode
  # ./ntripclient -s $Host -r $Port -m $Stream -u $User -p $Password $Serial -B $Baud
  echo 'Connect out ntripclient'
  sleep $SleepTime
  $WORK_DIR/set_ntrip.py
done) 

