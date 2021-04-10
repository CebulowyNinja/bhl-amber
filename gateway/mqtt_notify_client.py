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
MQTT_APP_ROOT = '/dogs'
MQTT_MIC_TOPIC = MQTT_APP_ROOT + '/mic'
MQTT_DOG_TOPIC = MQTT_APP_ROOT + '/dog'
MQTT_ALERTS_TOPIC = MQTT_APP_ROOT + '/alerts'

running = True
last_update = time.time()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_MIC_TOPIC+"/+")
    client.subscribe(MQTT_DOG_TOPIC+"/+")
    client.subscribe(MQTT_ALERTS_TOPIC+"/req")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global running, last_update
    print(msg.topic+" "+str(msg.payload))
    if str(msg.topic).startswith(MQTT_MIC_TOPIC):
        obj = json.loads(msg.payload)
        if 'value' in obj:
            message = prepare_payload("Dog is barking")
            client.publish(MQTT_ALERTS_TOPIC, message)
        else:
            print ('Unknown format')
    elif str(msg.topic).startswith(MQTT_ALERTS_TOPIC+"/req"):
        obj = json.loads(msg.payload)
        if 'req' in obj:
            if obj['req'] == 'clear':
                print('Clearing')
                message = prepare_payload("Everything is fine")
                client.publish(MQTT_ALERTS_TOPIC, message)
                last_update = time.time()+10
            else:
                print ('Unknown request')
        else:
            print ('Unknown format')
    elif str(msg.topic).startswith(MQTT_DOG_TOPIC):
        obj = json.loads(msg.payload)
        if 'hr' in obj:
            if obj['hr'] > 120:
                message = prepare_payload("Dog is nervous")
                client.publish(MQTT_ALERTS_TOPIC, message)
        else:
            print ('Unknown format')
    else:
        print ('Unknown topic')

idx = 0

def dog_barked():
    global idx, last_update
    if(time.time()-last_update > 10):
        return True
    else:
        return False

def dog_hr_alert():
    global idx, last_update
    if(time.time()-last_update > 10):
        return True
    else:
        return False

def prepare_payload(event):
    obj = {'timestamp': time.time(), 'event': event}
    return json.dumps(obj)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(MQTT_USER, password=MQTT_PASS)

client.connect(MQTT_HOST, MQTT_PORT, 60)
client.loop_forever()
