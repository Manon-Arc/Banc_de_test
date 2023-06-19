import machine
import time
import math
from bmi088 import BMI088

def photoelectrique(pin1, pin2):
    i2c = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21))
    devices = i2c.scan()
    print('I2C devices found:', devices)
    accel_gyro = BMI088(i2c)
    while True:
        accel = accel_gyro.read_accel()
        gyro = accel_gyro.read_gyro()
        print('Accelerometer (m/s^2):', accel)
        print('Gyroscope (rad/s):', gyro)
        roll = math.atan2(accel[1], accel[2]) * 180 / math.pi
        pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180 / math.pi
        print('Roll:', roll)
        print('Pitch:', pitch)
        time.sleep(0.1)