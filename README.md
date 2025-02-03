# Discthingy for bmx160 & bmp388

### Serial commands

```
restart                                                                         -  Restarts device
reset                                                                           -  Resets sensors
powersave                                                                       -  Toggles low power mode
mode <mode as string>                                                           -  Toggles sensor mode (off, record, debug) - default: record
frequency <hz as int>                                                           -  Sets sensor reading frequency - default: 100
calibrate <count as int>                                                        -  Calibrates sensors
wifi <ssid as string> <password as string> <retry as int>                       -  Connects to wifi and starts web server
sensitivity <accel as float> <gyro as float> <mag as float> <record as float>   -  Sets sensor sensitivity - default: 16384.0 16.4 1 0.005
```

### Wifi OTA screen

![image](https://github.com/cakenes/bmx160-bmp388/assets/25346075/6b132bfd-49a8-4334-a261-514e085df297)

![poc](poc.gif)
