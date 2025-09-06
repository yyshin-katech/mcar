#! /bin/bash

WORK_DIR="./"

#if ! [ -e /tmp/mountpoint ]; then

echo Running set_ntrip.py...
$WORK_DIR/set_ntrip.py

#fi
# echo set_ntrip completed
# sleep 2

# echo Starting ntrip client..
# $WORK_DIR/startntripclient.sh

# echo startntripclient completed


