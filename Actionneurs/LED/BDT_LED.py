import neopixel
import machine
import time

# Initialisation de la LED (avec paramètre de luminosité)
pixels = neopixel.NeoPixel(machine.Pin(1), brightness = 0.2, auto_write = False, pixel_order = neopixel.GRB)

while True:
    pixels.fill((0, 255, 0)) # remplir avec les variables
    pixels.show() # allumer la led en fonction de fill
    time.sleep(1)
    pixels.fill((255, 0, 0))
    pixels.show()
    time.sleep(1)
    pixels.fill((0, 0, 255))
    pixels.show()
    time.sleep(1)
    