import message_odometer_pb2
import socket
import struct

so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
localaddr = ('', 5002)
so.bind(localaddr)
so.settimeout(0.1)

while True:
    try:
        data = so.recvfrom(1024)[0]
    except KeyboardInterrupt:
        so.close()
        quit()

    frame = message_odometer_pb2.Message_Odometer()
    (msgID, pbdata) = struct.unpack('<I' + str(len(data) - 4) + 's', data)
    if(msgID == 0):
        frame.ParseFromString(pbdata)
        print(frame)
        so.close()
        quit()
