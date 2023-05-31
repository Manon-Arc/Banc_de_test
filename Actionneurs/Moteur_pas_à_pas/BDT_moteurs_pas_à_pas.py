import machine, time
from machine import Pin


Step1 = Pin(22, Pin.OUT)
Dir1 = Pin(23, Pin.OUT)


Dir1.value(True) # true = sens horaire false = sens anti - horaire

y = 100

def rotation(vitesse):
    try:
        Step1.value(True) # position n°1 de l'aimant
        time.sleep(10 / vitesse ** 2)
        Step1.value(False) # position n°2 de l'aimant
        time.sleep(10 / vitesse ** 2)
    except ZeroDivisionError:
        Step1.value(False)
        time.sleep(10)


while True:
    rotation(y)
