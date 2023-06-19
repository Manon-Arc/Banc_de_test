import machine
import time
import math
from mpu9250 import MPU9250

def IMU(pin1, pin2):
    i2c = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
    devices = i2c.scan()
    print('I2C devices found:', devices)
    imu = MPU9250(i2c)
    while True:
        accel = imu.acceleration
        gyro = imu.gyro
        mag = imu.magnetic
        print('Accelerometer (m/s^2):', accel)
        print('Gyroscope (rad/s):', gyro)
        print('Magnetometer (uT):', mag)
        roll = math.atan2(accel[1], accel[2]) * 180 / math.pi
        pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180 / math.pi
        print('Roll:', roll)
        print('Pitch:', pitch)
        time.sleep(0.1)
