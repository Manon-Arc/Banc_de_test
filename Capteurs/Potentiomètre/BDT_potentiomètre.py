from machine import ADC, Pin
import time

can = ADC(Pin(34))               # crée un objet ADC sur la broche 34
can.atten(ADC.ATTN_11DB)         # étendue totale : 3.3V
ADC.width(ADC.WIDTH_10BIT)       # change la résolution du convertisseur à 10bits

while True:
    pot = can.read()        # conversion analogique-numérique 0-1023
    print("CAN =", pot)     # affichage sur la console REPL de la valeur numérique
    time.sleep_ms(100)