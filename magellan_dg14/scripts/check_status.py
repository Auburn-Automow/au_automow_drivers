#!/usr/bin/env python

import sys
import os
import serial
import time

def check(port, baud):
    s = serial.Serial(port=port, baudrate=baud)

    s.write("\r\n\r\n\r\n")
    time.sleep(1)
    s.read()

    s.write("$PASHQ,CPD\r\n")
    while True:
        line = s.readline()
        if not line.startswith("$"):
            print line.rstrip()

def main(argv, stdout):
    usage = "usage: %prog [options] /path/to/config"
    parser = OptionParser(usage=usage)
    parser.add_option("-p", "--port", action="store", type="string", dest="port",
            default="/dev/gps",
            help="Serial port connected to GPS")
    parser.add_option("-b", "--baud", action="store", type="int", dest="baud",
            default=115200,
            help="Serial port baud rate")
    (options, args) = parser.parse_args(argv)

    try:
        check(options.port, options.baud)
    except ValueError:
        pass

if __name__ == '__main__':
    from optparse import OptionParser
    main(sys.argv, sys.stdout)
