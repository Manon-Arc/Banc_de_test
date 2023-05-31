from machine import Pin

button = Pin(14)
aimant=Pin(26,Pin.OUT) # Règle la broche D23 de la carte ESP32 en mode sortie

while True:
  if button.value():
    aimant.value(1)
    print("aimant activée")
  else: 
    aimant.value(0)
    print("aimant désactivée")