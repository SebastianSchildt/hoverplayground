#!/env/python3

import struct
import json
from enum import Enum

# typedef struct{
#    uint16_t start;
#    int16_t  cmd1;
#    int16_t  cmd2;
#    int16_t  speedR_meas;
#    int16_t  speedL_meas;
#    int16_t  batVoltage;
#    int16_t  boardTemp;
#    uint16_t cmdLed;
#    uint16_t checksum;
# } SerialFeedback;
# 18 bytes


class ParserState(Enum):
    WAIT_START1 = 1
    WAIT_START2 = 2
    PAYLOAD     = 3

START_FRAME_1=int(0xCD)
START_FRAME_2=int(0xAB)


def parse_frame(dataBytes, out):
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


    jsondata=json.dumps(parsed_data, sort_keys=True, indent=4)
    out.write(jsondata)
    out.write("\n")
    out.flush()



def serial_reader_run(ser, queue):
    print("Init reader")
    SerialFrame=bytearray(18)
    SerialFrame[0]=START_FRAME_1
    SerialFrame[1]=START_FRAME_2

    fout=open("rxFrame.txt","w")
    dataPtr=2
    state=ParserState.WAIT_START1


    while True:
        b=ser.read(size=1)

        if len(b) != 1:
            continue

        #print(f'Received {b}')
        if state == ParserState.WAIT_START1:
            if b[0] == START_FRAME_1:
                #print("Start1 to 2")
                state=ParserState.WAIT_START2
        elif state == ParserState.WAIT_START2:
            if b[0] == START_FRAME_2:
                #print("Start2 to payload")
                state=ParserState.PAYLOAD
                dataPtr=2
            else:
                #print(f"Reset. {b[0]} != {START_FRAME_2} ")
                state=ParserState.WAIT_START1
        elif state == ParserState.PAYLOAD:
            SerialFrame[dataPtr]=b[0]
            dataPtr+=1
            if dataPtr==18:
                #print("Parsing")
                state=ParserState.WAIT_START1
                parse_frame(SerialFrame,fout)
        else:
            print("Parser foobar. Resetting")
            state=ParserState.WAIT_START1
