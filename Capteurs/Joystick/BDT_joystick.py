from time import sleep_ms  # Pour temporiser
from machine import ADC, Pin

vertical = ADC(Pin('A0'))
horizontal = ADC(Pin('A1'))

while True:

    sleep_ms(500)

    x = vertical.read()
    y = horizontal.read()

    if 780 >= x >= 750:
        print("Haut")
    if 280 >= x >= 240:
        print("Bas")
    if 780 >= y >= 750:
        print("Gauche")
    if 280 >= y >= 240:
        print("Droite")
    if x >= 1000:
        print("test")
