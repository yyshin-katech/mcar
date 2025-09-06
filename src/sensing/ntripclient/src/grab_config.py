#! /usr/bin/env python3

import sys
import configparser

if len(sys.argv) < 4:
    print("Not enough arguments, Usage: grab_config CONFIG.INI HEADER CONTENTS")
    exit(1)


conf_file = sys.argv[1]
conf_header = sys.argv[2]
conf_contents = sys.argv[3]

c = configparser.ConfigParser()
c.read(conf_file)

conf_param = c[conf_header][conf_contents]
print(conf_param)

exit(0)

#print(len(sys.argv))
#print(sys.argv[0])
#print(sys.argv[1])

