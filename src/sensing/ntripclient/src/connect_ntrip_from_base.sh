#!/bin/bash
#
# $Id$
# Purpose: Start ntripclient

# change these 3 according to your needs
User='jaeho0608@hanyang.ac.kr'
Password='gnss'
Host='www.gnssdata.or.kr'
Port='2101'

Stream=$(cat $1/data/mountpoint)
Serial="-D $(exec $1/src/grab_config.py $1/src/config.ini DEFAULT DEV_RTCM)"
Baud=38400
mode='a'

echo " ./ntripclient -s $Host -r $Port -m $Stream -u $User -p $Password $Serial -B $Baud -M $mode "
exec $1/src/ntripclient -s $Host -r $Port -m $Stream -u $User -p $Password $Serial -B $Baud -M $mode

