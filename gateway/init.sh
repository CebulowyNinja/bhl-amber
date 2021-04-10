#!/bin/bash
# run as su
apt update
apt install --no-install-recommends -y mosquitto mosquitto-clients screen ufw
ufw allow ssh
ufw allow 3333
pip3 install paho-mqtt 
