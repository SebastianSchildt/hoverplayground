#!/env/python3

import serial
import threading
import json
import queue
import argparse

data_set = {"a0": -1 , "a1": -1}

class SerialReader(threading.Thread):
    def __init__(self, port, pipe):
        threading.Thread.__init__(self)
        print(f"Init Arduino Reader, pipe is {pipe} ")

        self.port = serial.Serial(port)
        self.pipe=pipe
        self.exitFlag = False
        self.exitMutex = threading.Lock()
        self.dataMutex = threading.Lock()
        self.data = data_set

    def run(self):
        port = self.port

        while True:
            # see whether an exit was requested
            with self.exitMutex:
                if self.exitFlag:
                    break
            with self.dataMutex:
                self.data = port.readline().decode("utf-8")
            self.emitNewData(self.data, 500)


    def emitNewData(self,rxdata, range): # set scale
        a0, a1 = self.get(rxdata, range)
        speed, steer = self.convert_to_speed_steer(a0, a1)
        self.pipe.put( (int(speed), int(steer)) )

    def get(self, raw_data, range, range_raw = 1000):
        get_data = {}
        get_data["a0"] = 0
        get_data["a1"] = 0
        with self.dataMutex:  # lock the buffer and copy the requested data out
            try:
                get_data = json.loads(raw_data)
            except json.decoder.JSONDecodeError:
                pass
        a0 = get_data["a0"] *range / range_raw
        a1 = get_data["a1"] *range / range_raw
        return (a0, a1)

    def convert_to_speed_steer(self, a0, a1):
        "+ right"
        "a1 right"
        if a1 > a0:
            speed = a1
        else:
            speed = a0
        steer = a1 -a0
        return (speed, steer)


    def exit(self):
        """ Instruct the serial thread to exit."""
        with self.exitMutex:
            self.exitFlag = True

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Give arduino port')
    parser.add_argument('--port', metavar='N', type=str, default='/dev/ttyUSB0',
                        help='arduino port')
    args = parser.parse_args()

    pipe=queue.Queue()
    thread = SerialReader(args.port, pipe)
    thread.start()

    while True:
        try:
            (sp, st)=pipe.get()
            print(f"Received {sp}, {st}")
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            thread.exit()
            break
