import machine
import time
from onewire import OneWire
from ds18x20 import DS18X20

def sonde_thermique(pin):
    ow = OneWire(machine.Pin(pin))  
    ds = DS18X20(ow)
    devices = ds.scan()
    print('DS18B20 devices found:', devices)
    while True:
        ds.convert_temp()
        time.sleep_ms(750)
        temperatures = []
        for device in devices:
            temp = ds.read_temp(device)
            temperatures.append(temp)
        for i, temp in enumerate(temperatures):
            print('Sensor', i + 1, 'Temperature:', temp, '°C')
        time.sleep(1)