import sys
import serial
import struct
import json
from enum import Enum
import threading
import queue
from pathlib import Path

class ParserState(Enum):
    WAIT_START1 = 1
    WAIT_START2 = 2
    PAYLOAD     = 3

START_FRAME_1=int(0xCD)
START_FRAME_2=int(0xAB)

def parse_frame(dataBytes):
    rxFrame = struct.Struct('HhhhhhhHH')
    checksumFrame = struct.Struct('HHHHHHHHH')
    data=rxFrame.unpack(dataBytes)
    checksumData = checksumFrame.unpack(dataBytes)

    checksum=checksumData[0]^checksumData[1]^checksumData[2]^checksumData[3]^checksumData[4]^checksumData[5]^checksumData[6]^checksumData[7]

    parsed_data={}

    parsed_data["cmd1"]=data[1]
    parsed_data["cmd2"]=data[2]
    parsed_data["speedR_meas"]=data[3]
    parsed_data["speedL_meas"]=data[4]
    parsed_data["batVoltage"]=data[5]
    parsed_data["boardTemp"]=data[6]
    parsed_data["cmdLed"]=data[7]
    parsed_data["checksum"]=data[8]


    if checksum != data[8]:
        print(f"Checksum error {checksum} != {data[8]}")
        parsed_data["checksumOk"]=f"Checksum error {checksum} != {data[8]}"
    else:
        parsed_data["checksumOk"]="yes"

    return parsed_data

def init_state():
    print("Init reader")
    SerialFrame=bytearray(18)
    SerialFrame[0]=START_FRAME_1
    SerialFrame[1]=START_FRAME_2
    dataPtr=2
    state=ParserState.WAIT_START1

    return dataPtr, state, SerialFrame

def init_write_file(file_path):
    path = Path(file_path).resolve()
    print("write into: ", path)
    fout=open(path, "w")
    return fout

def write_into_file(fout, parsed_data):
    if parsed_data:
        jsondata=json.dumps(parsed_data, sort_keys=True, indent=4)
        fout.write(jsondata)
        fout.write("\n")
        fout.flush()

def parse_from_serial(data_byte, state, dataPtr, SerialFrame):

    parsed_data = None
    if len(data_byte) == 1:
        # print(f'Received {data_byte}')

        if state == ParserState.WAIT_START1:
            if data_byte[0] == START_FRAME_1:
                # print("Start1 to 2")
                state=ParserState.WAIT_START2
        elif state == ParserState.WAIT_START2:
            if data_byte[0] == START_FRAME_2:
                # print("Start2 to payload")
                state=ParserState.PAYLOAD
                dataPtr=2
            else:
                print(f"Reset. {data_byte[0]} != {START_FRAME_2} ")
                state=ParserState.WAIT_START1
        elif state == ParserState.PAYLOAD:
            SerialFrame[dataPtr]=data_byte[0]
            dataPtr+=1
            if dataPtr==18:
                # print("Parsing")
                state=ParserState.WAIT_START1
                parsed_data = parse_frame(SerialFrame)
        else:
            print("Parser foobar. Resetting")
            state=ParserState.WAIT_START1

    return state, dataPtr, parsed_data

def serial_reader_run(ser, queue=None, path=None):
    dataPtr, state, SerialFrame = init_state()
    log = False
    if path:
        fout = init_write_file(path)
        log = True
    while True:
        b=ser.read(size=1)
        state, dataPtr, parsed_data = parse_from_serial(b, state, dataPtr, SerialFrame)
        if parsed_data:
            write_into_file(fout, parsed_data)
        if queue:
            queue.put(parsed_data)

        if log:
            write_into_file(fout, parsed_data)
        else:
            print(parsed_data)

if __name__ == '__main__':
    ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)  # open serial port
    pipe=queue.Queue()

    threading.Thread(target=serial_reader_run,args=(ser, pipe, "json_data.yaml")).start()

    import time
    while (True):
        try:
            parsed_data =pipe.get()
        except KeyboardInterrupt:
            print("Keyboard Interrupt")
            break
