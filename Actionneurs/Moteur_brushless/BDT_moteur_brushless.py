from machine import Pin, PWM
from time import sleep

motor = PWM(Pin(13),freq=50)
duty_cycle = 50 # Between 0 - 100 %

while True:
    motor.duty(int((duty_cycle/100)*1024))
    sleep(1)
    