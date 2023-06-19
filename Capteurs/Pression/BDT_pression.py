import machine
import time
import bme280

def pression(pin1, pin2):
    i2c = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
    devices = i2c.scan()
    print('I2C devices found:', devices)
    bme = bme280.BME280(i2c=i2c)

    while True:
        temperature = bme.temperature
        humidity = bme.humidity
        pressure = bme.pressure
        print('Temperature (C):', temperature)
        print('Humidity (%):', humidity)
        print('Pressure (hPa):', pressure)
        time.sleep(1)
