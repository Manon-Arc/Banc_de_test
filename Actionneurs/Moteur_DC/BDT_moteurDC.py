from machine import Pin

button = Pin(14, Pin.IN, Pin.PULL_UP)
motor=Pin(26,Pin.OUT) # Règle la broche D23 de la carte ESP32 en mode sortie

while True:
  if not button.value():
    motor.value(1) 
  else: 
    motor.value(0) 