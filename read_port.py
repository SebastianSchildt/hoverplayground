import serial
import threading
import json
import queue

#define MAX_LINEAR_VELOCITY              (WHEEL_RADIUS * 2 * 3.14159265359 * 61 / 60) // m/s  (BURGER : 61[rpm], WAFFLE : 77[rpm])
#define MAX_ANGULAR_VELOCITY             (MAX_LINEAR_VELOCITY / TURNING_RADIUS)       // rad/s

data_set = {"a0": -1 , "a1": -1}

class SerialReader(threading.Thread):
    def __init__(self, pipe):
        threading.Thread.__init__(self)
        print(f"Init Arduino Reader, pipe is {pipe} ")

        self.port = serial.Serial('/dev/ttyACM0')
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
                self.emitNewData(self.data)


    def emitNewData(self,rxdata):
        json_data=json.loads(rxdata)
        l_speed=json_data["a0"]
        r_speed=json_data["a1"]
        self.pipe.put( (int(l_speed),0))


    def get(self):
        with self.dataMutex:  # lock the buffer and copy the requested data out
            get_data = json.loads(self.data)
        print("ao: ", get_data["a0"])
        print("a1: ", get_data["a1"])
        return get_data

    def exit(self):
        """ Instruct the serial thread to exit."""
        with self.exitMutex:
            self.exitFlag = True

def convert_speed_angular(a0, a1, max_speed, sport_mode = False):
    if not sport_mode:
        max = max_speed /2
    else:
        max = max_speed
    return speed, angular


if __name__ == "__main__":

    # Create thread to read and buffer serial data.
    pipe=queue.Queue()
    thread = SerialReader(pipe)
    thread.start()

    while True:
        try:
            (sp, st)=pipe.get()
            print(f"Received {sp}, {st}")

            #thread.get()
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            thread.exit()
            break
