import time 
from machine  import Pin
 
buzzer=Pin(23,Pin.OUT) # R猫gle la broche D23 de la carte ESP32 en mode sortie
 
while True:
    buzzer.value(1) #le buzzer sonne
    time.sleep(2) # Attendre 2s
    buzzer.value(0) #le buzzer arrête de sonner
    time.sleep(2) # Attendre 2s