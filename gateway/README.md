# IoT gateway based on Rapberry PI 4B

## API

### MQTT

```
mqtt-port: 1883
mqtt-topics:
    app-topic:   /dogs/#
    mic-topic:   /dogs/mic/+
    dog-topic:   /dogs/dog/+
    event-topic: /dogs/alert
```

### Microphones

```
communication:
    protocol:  UDP
    port:      3333
data:
    format:    int16_t
    s_freq:    22050
    num_bytes: 512
```

### Dog Companion

```
communication:
    protocol:  MQTT
    port:      1883
    topic:     /dogs/dog/+
data:
    format:    json-document
status_flags:
    is_moving:  0x01
    is_on_feet: 0x02
    is_on_side: 0x04
heart_rate:
    min:        20
    max:        255
```

### JSON frames

Events

```json
{
    "timestamp": 0.0,
    "event": "string",
}
```

Dog

```json
{
    "timestamp": 0.0,
    "status": 0.0,
    "hr": 0.0
}
```
