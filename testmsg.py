#!/env/python3


import struct

rxFrame = struct.Struct('HhhhhhhHH')
b=rxFrame.pack(0xABCD, 1,2,3,4,5,6,7,8)

print(f"Will sent {len(b)} bytes")

f=open("/dev/pts/5",'wb')

f.write(b)

f.write(b'0xff')
f.close()
