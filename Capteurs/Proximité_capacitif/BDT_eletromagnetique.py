from machine import Pin

sensor = Pin(12,Pin.OUT)

while True:
    if sensor :
        print("metaux présent")
    else:
        print("nothing")