import machine
import time
from as7341 import AS7341

def spectrometre(pin1, pin2):
    i2c = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
    devices = i2c.scan()
    print('I2C devices found:', devices)
    as7341 = AS7341(i2c)
    as7341.setup(mode=AS7341.MODE_1)
    while True:
        spectral_data = as7341.get_spectral_data()
        print('Spectral Data:', spectral_data)
        time.sleep(1)
