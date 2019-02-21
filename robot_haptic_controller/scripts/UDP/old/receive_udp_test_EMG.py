import socket
import sys
import pickle
import struct

import load_msg as load_msg


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address = ('128.178.145.174', 9027)
print >>sys.stderr, 'starting up on %s port %s' % server_address
sock.bind(server_address)


while True:
    # read data
    print >>sys.stderr, '-------\nwaiting to receive message'
    data, address = sock.recvfrom(4096)

    load_msg.get_positions_from_raw_data(data)
    print(joint_vector)

    print >>sys.stderr, 'received %s bytes from %s' % (len(data), address)
    print >>sys.stderr, data
