#!/usr/bin/python3
print("Amber Notify Client")

import paho.mqtt.client as mqtt
import json
import time

MQTT_HOST = '54.36.99.31'
MQTT_PORT = 1883
MQTT_USER = 'mqttuser'
MQTT_PASS = 'mqttpassword'
MQTT_MIC_ID = 1
MQTT_IN_TOPIC = '/dogs/mic/' + str(MQTT_MIC_ID)
MQTT_OUT_TOPIC = '/dogs/alerts'

running = True

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_IN_TOPIC)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global running
    print(msg.topic+" "+str(msg.payload))
    if msg.topic[-1] == '1':
        obj = json.loads(msg.payload)
        if obj['req'] == 'reset':
            print ('Quitting')
            running = False
        else:
            print ('Unknown request')

idx = 0

def dog_barked():
    global idx
    return True

def dog_hr_alert():
    global idx
    return True

def prepare_payload(event):
    obj = {'timestamp': time.time(), 'event': event}
    return json.dumps(obj)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(MQTT_USER, password=MQTT_PASS)

client.connect(MQTT_HOST, MQTT_PORT, 60)
client.loop_start()

while running:
    if dog_barked():
        message = prepare_payload("dog barked")
        client.publish(MQTT_OUT_TOPIC, message)
    if dog_hr_alert():
        message = prepare_payload("dog hr increased")
        client.publish(MQTT_OUT_TOPIC, message)
    time.sleep(1)

client.loop_stop()
