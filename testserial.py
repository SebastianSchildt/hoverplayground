#!/env/python

import serial
import queue
import threading

from writer import *
# from reader import *
from hovercar_parser.parse_hoverdata import serial_reader_run

from read_port import SerialReader
import argparse

parser = argparse.ArgumentParser(description='Give ports')
parser.add_argument('arduino_port', metavar='arduino_port', type=str,
                    help='arduino port')
parser.add_argument('hover_port', metavar='hover_port', type=str,
                    help='hoverboard programmer port')
args = parser.parse_args()

UART=args.hover_port
arduino_port = args.arduino_port


print("Test serial")

ser = serial.Serial(UART, 115200, timeout=None)  # open serial port
pipe=queue.Queue()

threading.Thread(target=serial_writer_run,args=(ser,pipe)).start()
threading.Thread(target=serial_reader_run,args=(ser, None, "log/hoverinfo.yaml")).start()

arduinoThread = SerialReader(arduino_port, pipe)
arduinoThread.start()


while (True):
    time.sleep(1)
    #speed = int(input("Enter target speed: "))
    #steer = int(input("Enter target steer: "))
    #print(f"Sending speed {speed} and steer {steer} to queue")
    #pipe.put((speed,steer))




#
#   uint16_t start;
#   int16_t  steer;
#   int16_t  speed;
#   uint16_t checksum;
