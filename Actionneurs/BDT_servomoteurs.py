import machine
import time

p13 = machine.Pin(13)
servo = machine.PWM(p13,freq=50)

while True :
    servo.duty(110)
    time.sleep(2)
    servo.duty(88)
    time.sleep(2)
