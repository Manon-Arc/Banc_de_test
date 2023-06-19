from machine import Pin
import time

pin_microphone = Pin(2, Pin.IN)

while True:
    valeur_microphone = pin_microphone.value()  # Lire la valeur du capteur sonore
    if valeur_microphone == 0:
        print("Pas de son détecté")
    else:
        print("Son détecté")
    time.sleep(0.1)