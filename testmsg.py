#!/env/python3


import struct

rxFrame = struct.Struct('HhhhhhhHH')
b=rxFrame.pack(0xABCD, 1,2,3,4,5,6,7,8)

print(f"Will sent {len(b)} bytes")

f=open("/dev/ttyUSB0",'wb')

f.write(b)

f.write(b'\xcd\xab\x99\xffg\x00\xb8\xff\n\x00\xcf\x0e\xa4\x01\x00\x00\xea\xa4')
f.close()
