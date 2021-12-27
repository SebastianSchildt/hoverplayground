#!/env/python3

import struct
import json

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




from enum import Enum

class ParserState(Enum):
    WAIT_START1 = 1
    WAIT_START2 = 2
    PAYLOAD     = 3

START_FRAME_1=int(0xAB)    
START_FRAME_2=int(0xCD)    


def parse_frame(data, out):
    rxFrame = struct.Struct('HhhhhhhHH')
    data=rxFrame.unpack(data)
    checksum=data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]^data[7]
    data={}


    data["cmd1"]=data[1]
    data["cmd2"]=data[2]
    data["speedR_meas"]=data[3]
    data["speedL_meas"]=data[4]
    data["batVoltage"]=data[5]
    data["boardTemp"]=data[6]
    data["cmdLed"]=data[7]
    data["checksum"]=data[8]


    if checksum != data[8]:
        print(f"Checksum error {checksum} != {data[8]}")
        data["checksumOk"]=f"Checksum error {checksum} != {data[8]}"
    else:
        data["checksumOk"]="yes"


    json=json.dumps(data, sort_keys=True, indent=4)
    out.write(json)
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
        print(f'Received {b}')
        
        if len(b) != 1:
            continue
        
        print(f'Received {b}')
        if state == ParserState.WAIT_START1:
            if b[0] == START_FRAME_1:
                state=ParserState.WAIT_START2
        elif state == ParserState.WAIT_START2:
            if b[0] == START_FRAME_2:
                state=ParserState.PAYLOAD
                dataPtr=2
            else:
                state=ParserState.WAIT_START1
        elif state == ParserState.PAYLOAD:
            SerialFrame=[dataPtr]=b[0]
            dataPtr+=1
            if dataPtr==18:
                state=ParserState.WAIT_START1
                parse_frame(SerialFrame,fout)
        else:
            print("Parser foobar. Resetting")
            state=ParserState.WAIT_START1


