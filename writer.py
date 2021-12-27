#!/env/python3

import struct
import time
from queue import Empty

START_FRAME=int(0xABCD)    


def serial_writer_run(ser, queue):
    print("Init serial writer")
    speed=steer=0
    commandFrame = struct.Struct('HhhH')
    while True:
        #try queue
        try:
            (sp, st)=queue.get_nowait()
            if sp != speed or st != steer:
                print(f"Sending new speed {sp} and steer {st} to serial")
            speed=sp
            steer=st
        except Empty:
            pass
    
        checksum=START_FRAME^steer^speed
        data=commandFrame.pack(START_FRAME,steer,speed,checksum)

        ser.write(data)

        time.sleep(1)
    
