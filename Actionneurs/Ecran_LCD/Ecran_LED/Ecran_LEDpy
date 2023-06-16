from machine import Pin, I2C
import ssd1306
from time import sleep_ms

def ecran_led(scl, sda):
    i2c = I2C(0, scl=Pin(scl), sda=Pin(sda))
    sleep_ms(1000)
    screen_width = 128
    screen_length = 32
    oled = ssd1306.SSD1306_I2C(screen_width, screen_length, i2c)
    
    # Clear the screen
    oled.fill(0)
    
    # Draw the 3D box
    oled.rect(10, 10, 80, 20, 1)  # Outer rectangle
    oled.line(10, 10, 30, 30, 1)  # Top-left diagonal line
    oled.line(90, 10, 110, 30, 1)  # Top-right diagonal line
    oled.line(10, 30, 30, 50, 1)  # Bottom-left diagonal line
    oled.line(90, 30, 110, 50, 1)  # Bottom-right diagonal line
    
    oled.show()

# Example usage with SCL pin 5 and SDA pin 4
ecran_led(5, 4)
