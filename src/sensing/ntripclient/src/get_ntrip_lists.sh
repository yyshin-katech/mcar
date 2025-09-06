#!/bin/bash
#
# $Id$
# Purpose: Start ntripclient

# change these 3 according to your needs
User='jaeho0608@hanyang.ac.kr'
Password='gnss'
Host='www.gnssdata.or.kr'
Port='2101'

exec $1/ntripclient -s $Host -r $Port -u $User -p $Password

