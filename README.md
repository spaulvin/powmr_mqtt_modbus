https://github.com/leodesigner/powmr_comm

HomeAssistant

Need to install integration https://www.home-assistant.io/integrations/utility_meter/

automations.yaml

```
alias: 'Poll All Powmr Registers'
description: 'Polls all Powmr Modbus registers in a single automation.'
trigger:
  - platform: time_pattern
    seconds: '/10'
action:
  - service: mqtt.publish
    data:
      topic: '/iot/node/powmr/c/modbus_read_range'
      payload: '208,22'
mode: single
```

configuration.yaml

```
sensor: !include powmr.yaml
```

watch -n10 'mosquitto_pub -h 192.168.88.145 -t "/iot/node/powmr/c/modbus_read_range" -m 208,20'

KNOWN BUGS:
simultanious modbus_read_single responses can be mixed