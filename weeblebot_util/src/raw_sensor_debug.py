#! /usr/bin/python
import serial
port = serial.Serial('/dev/ttyS0',115200)


try:
    while True:
        line = port.read(100);
        print line
except:
    pass;
