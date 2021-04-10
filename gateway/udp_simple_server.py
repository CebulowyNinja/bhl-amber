#!/usr/bin/python3
import socket
import numpy as np
import struct

UDP_IP = '192.168.0.167'
UDP_PORT = 3333

msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f'Server up and listening on {UDP_IP}:{UDP_PORT}')

while True:
    bdata, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    #print(f'Received {len(bdata)} bytes from {addr}')
    count = len(bdata)/2
    integers = struct.unpack('h'*int(count), bdata)
    # TODO analyze integers array (eg. amplitude detection)
