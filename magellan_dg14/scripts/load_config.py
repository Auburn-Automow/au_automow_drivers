#!/usr/bin/env python

import sys
import os
import serial

def loader(fname, port, baud):
    s = serial.Serial(port=port, baudrate=baud)
    f = open(fname)

    s.write("$PASHS,NME,ALL,A,OFF\r\n")
    s.write("$PASHS,NME,ALL,B,OFF\r\n")


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
    if len(args) < 2:
        parser.error("Please specify a config file.")
        syst.exit(1)

    configFile = args[1]

    try:
        loader(configFile, options.port, options.baud)
    except ValueError:
        pass

if __name__ == '__main__':
    from optparse import OptionParser
    main(sys.argv, sys.stdout)
