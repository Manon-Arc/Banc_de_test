import esp_rgb_lcd_grove
import time

lcd = esp_rgb_lcd_grove.esp_afficheur_lcd()
lcd.color(0, 0, 0)

while True:
    # affichage
    lcd.clear()
    lcd.setCursor(2, 0)
    lcd.write('hello World')

    # couleur
    lcd.color(255, 0, 0)
    time.sleep_ms(500)
    lcd.color(0, 255, 0)
    time.sleep_ms(500)
    lcd.color(0, 0, 255)
    time.sleep_ms(500)