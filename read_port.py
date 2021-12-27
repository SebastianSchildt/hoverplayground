import serial
import threading
import json
import queue

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

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
        self.pipe.put( (int(a0), int(a1)) )

    def get(self, raw_data, range, range_raw = 1000):
        with self.dataMutex:  # lock the buffer and copy the requested data out
            get_data = json.loads(raw_data)
        a0 = get_data["a0"] *range / range_raw
        a1 = get_data["a1"] *range / range_raw
        return (a0, a1)

    def exit(self):
        """ Instruct the serial thread to exit."""
        with self.exitMutex:
            self.exitFlag = True

if __name__ == "__main__":

    # Create thread to read and buffer serial data.
    pipe=queue.Queue()
    thread = SerialReader("/dev/ttyACM1", pipe)
    thread.start()

    while True:
        try:
            (sp, st)=pipe.get()
            print(f"Received {sp}, {st}")
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            thread.exit()
            break
