#!/env/python3

import struct
import time
from queue import Empty
from timeit import default_timer

START_FRAME=int(0xABCD)

#43981

def serial_writer_run(ser, queue):
    print(f"Init serial writer, pipe is {queue}")
    speed=steer=0
    commandFrame = struct.Struct('HhhH')
    timelast=default_timer()
    while True:
        #try queue
        try:
            (sp, st)=queue.get_nowait()
            #print(f"Data: {sp} {st}")
            if sp != speed or st != steer:
                print(f"Sending new speed {sp} and steer {st} to serial")
            speed=sp
            steer=st
        except Empty:
            time.sleep(0.1)
            pass

        c_speed=speed
        c_steer=steer
        if speed < 0: #emulate "cast" to unsigned in 2-s complement
            c_speed= speed + ( 1 << 16)
        if steer < 0: #emulate "cast" to unsigned in 2-s complement
            c_steer= steer + ( 1 << 16)


        checksum=START_FRAME^c_steer^c_speed

        try:
            data=commandFrame.pack(START_FRAME,steer,speed,checksum)
        except:
            print(f"Error with {steer}, {speed}/{c_speed}, {checksum}")
            return

        #only write if last time more than 100ms in past
        now=default_timer()
        if (now-timelast) > 100e-03:
            ser.write(data)
            timelast=default_timer()
        else:
            #print("Wait")
            pass
