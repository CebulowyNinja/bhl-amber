#!/usr/bin/python3
import socket
import numpy as np
import struct

import paho.mqtt.client as mqtt
import json
import time

UDP_IP = '192.168.0.167'
UDP_PORT = 3333

MQTT_HOST = '54.36.99.31'
MQTT_PORT = 1883
MQTT_USER = 'mqttuser'
MQTT_PASS = 'mqttpassword'
MQTT_MIC_TOPIC_PREFIX = '/dogs/mic/'

addresses = []

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))


def is_bark(tab_probki):
    prog=1e+6
    tab= np.array(tab_probki)
    
    stala = np.mean(tab)
    #print(stala)
    zero_centred = tab - stala
    srednia=np.sum(zero_centred**2)/256
    #print(srednia)
    return srednia

def notify_user(value, idx):
    print("Dog is barking howl howl")
    message = prepare_payload(value)
    client.publish(MQTT_MIC_TOPIC_PREFIX + str(idx), message)

def prepare_payload(value):
    obj = {'timestamp': time.time(), 'value': value}
    return json.dumps(obj)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(MQTT_USER, password=MQTT_PASS)

client.connect(MQTT_HOST, MQTT_PORT, 60)
client.loop_start()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f'Server up and listening on {UDP_IP}:{UDP_PORT}')

while True:
    bdata, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    if addr in addresses:
        idx = addresses.index(addr)
    else:
        addresses.append(addr)
        idx = addresses.index(addr)
    #print(f'Received {len(bdata)} bytes from {addr} idx:{idx}')
    count = len(bdata)/2
    integers = struct.unpack('h'*int(count), bdata)
    # TODO analyze integers array (eg. amplitude detection)
    value = is_bark(integers)
    if value > 1e+6:
        notify_user(value, idx)

client.loop_stop()
