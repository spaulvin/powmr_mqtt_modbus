https://github.com/leodesigner/powmr_comm

HomeAssistant

automations.yaml

```
- id: '1723041008766'
  alias: Powmr pool
  description: ''
  trigger:
  - platform: time_pattern
    seconds: '00'
  condition: []
  action:
  - service: mqtt.publish
    data:
      topic: /iot/node/powmr/c/modbus_read_single
      payload: '229'
  mode: single
```

configuration.yaml

```
sensor: !include solar.yaml
```

solar.yaml

```
- platform: mqtt
  state_topic: '/iot/node/powmr/log/modbus/229'
  name: 'powmr_battery_soc'
  unique_id: 'powmr.battery_soc'
  unit_of_measurement: '%'
  value_template: '{{ value | float | multiply(0.1) | round(2) }}'
  device_class: power
```