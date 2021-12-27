#!/env/python

import serial
import queue
import threading

from writer import *
from reader import *

from read_port import SerialReader


UART="/dev/ttyUSB0"
#UART="/tmp/test"


print("Test serial")

ser = serial.Serial(UART, 115200, timeout=None)  # open serial port
pipe=queue.Queue()

threading.Thread(target=serial_writer_run,args=(ser,pipe)).start()
threading.Thread(target=serial_reader_run,args=(ser,None)).start()

arduinoThread = SerialReader(pipe)
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
