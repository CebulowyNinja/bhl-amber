#!/usr/bin/python3
import socket
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct

UDP_IP = '192.168.0.167'
UDP_PORT = 3333

msgFromServer       = "Hello UDP Client"
bytesToSend         = str.encode(msgFromServer)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f'Server up and listening on {UDP_IP}:{UDP_PORT}')

fig, ax = plt.subplots()
max_x = 256
x = np.arange(0, max_x)
ax.set_ylim(-35000, 35000)
data = np.random.randint(-35000, 35000, max_x)

line, = ax.plot(x, data)

def init():  # give a clean slate to start
    line.set_ydata([np.nan] * len(x))
    return line,

def animate(i):  # update the y values (every 1000ms)
    print(data[0:16])
    line.set_ydata(data)
    return line,

ani = animation.FuncAnimation(
    fig, animate, init_func=init, interval=100, blit=True, save_count=10)

plt.show()

while True:
    bdata, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print(f'Received {len(bdata)} bytes from {addr}')
    count = len(bdata)/2
    data = struct.unpack('h'*int(count), bdata)
    sock.sendto(bytesToSend, addr)
