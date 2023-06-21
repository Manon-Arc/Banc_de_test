from platform import machine

from aiohttp import Fingerprint
from machine import Pin, PWM, ADC, i2c
from time import sleep, sleep_ms, sleep_us, ticks_us
import tm1637
import esp_rgb_lcd_grove
import neopixel
import pypot.dynamixel as dynamixel
import serial

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from pypot.dynamixel.conversion import dynamixelModels
from pypot.dynamixel import DxlIO, Dxl320IO, get_available_ports
from pypot.utils import flushed_print as print
import sys
import ssd1306
from fingerprint3 import Fingerprint3
from MPU6050 import MPU6050
import lsm6dso


class TestClass:
    melody = []
    
    def sept_segments(PinIn, PinOut):
        try:
            tm= tm1637.TM1637(PinIn, PinOut)
            str = 'Hello World!'
            while True:
                tm.show('abcd')
                sleep(1)
                tm.number(1234)
                sleep(1)
                tm.numbers(12,34)
                sleep(1)
                tm.scroll(str)
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")
 
        
    def buzzer(PinIn, PinOut):
        try:
            buzzer=Pin(PinIn,PinOut)
            while True:
                buzzer.value(1) #le buzzer sonne
                sleep(2) # Attendre 2s
                buzzer.value(0) #le buzzer arrête de sonner
                sleep(2) # Attendre 2s
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def lcd(PinIn, PinOut):
        try:
            lcd = esp_rgb_lcd_grove.esp_rgb_lcd(PinIn, PinOut)
            lcd.clear()
            lcd.set_cursor(0,0)
            lcd.write("Hello World")
            lcd.set_cursor(0,1)
            lcd.write("Hello World")
            sleep(2)
            lcd.clear()
            lcd.set_cursor(0,0)
            lcd.write("Hello World")
            lcd.set_cursor(0,1)
            lcd.write("Hello World")
            sleep(2)
            lcd.clear()
        except KeyboardInterrupt:
            print("Program interrupted by user")
            
    
    def electroaimant(PinIn, PinOut):
        try:
            button = Pin(PinIn)
            aimant=Pin(PinOut,Pin.OUT)
            while True:
                if button.value():
                    aimant.value(1)
                    print("aimant activée")
                else: 
                    aimant.value(0)
                    print("aimant désactivée")
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def set_melody(new_melody):
        global melody
        melody = new_melody


    def haut_parleur(PinIn, PinOut):
        try:
            pwm = PWM(Pin(PinIn, PinOut))
            notes = {
                'do': 261,
                're': 293,
                'mi': 329,
                'fa': 349,
                'sol': 392,
                'la': 440,
                'si': 493,
                'do2': 523
            }
            melody2 = melody
            for i in melody2:
                duree = (i[1] * 200) - 50  # note en milliseconde
                pwm.freq(notes[i[0]])  # fréquence de la note
                pwm.duty_u16(32512)  # rapport cyclique 50%
                sleep_ms(int(duree))  
                pwm.duty_u16(0)  # rapport cyclique nul (silence)
                sleep_ms(50)
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def bouton_poussoir(PinIn, PinOut):
        try:
            bp_1 = Pin(PinIn, PinOut)
            while True:
                if bp_1.value() == 1:
                    print("bp pressed")
                else:
                    print("bp not pressed")
        except KeyboardInterrupt:
            print("Program interrupted by user")

    
    def led(PinIn):
        try:
            # Initialisation de la LED (avec paramètre de luminosité)
            pixels = neopixel.NeoPixel(PinIn, brightness = 0.2, auto_write = False, pixel_order = neopixel.GRB)

            while True:
                pixels.fill((0, 255, 0)) # remplir avec les variables
                pixels.show() # allumer la led en fonction de fill
                sleep(1)
                pixels.fill((255, 0, 0))
                pixels.show()
                sleep(1)
                pixels.fill((0, 0, 255))
                pixels.show()
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def brushless(pin):
        try:
            motor = PWM(Pin(Pin),freq=50)
            duty_cycle = 50 # Between 0 - 100 %
            while True:
                motor.duty(int((duty_cycle/100)*1024))
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def DC_motor(Pin):
        try:
            motor = PWM(Pin(Pin),freq=50)
            duty_cycle = 50 # Between 0 - 100 %
            while True:
                motor.duty(int((duty_cycle/100)*1024))
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def stepper_motor(Pin1, Pin2, Pin3, Pin4):
        try:
            pins = [Pin(Pin1, Pin.OUT), Pin(Pin2, Pin.OUT), Pin(Pin3, Pin.OUT), Pin(Pin4, Pin.OUT)]
            sequence = [[1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0], [0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1]]

            for _ in range(512):
                for step in sequence:
                    for i in range(4):
                        pins[i].value(step[i])
                    sleep_ms(2)  # délai entre chaque étape du moteur
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def potentiometre(Pin):
        try:
            can = ADC(Pin(Pin)) 
            can.atten(ADC.ATTN_11DB)         # étendue totale : 3.3V
            ADC.width(ADC.WIDTH_10BIT)       # change la résolution du convertisseur à 10bits

            while True:
                pot = can.read()        # conversion analogique-numérique 0-1023
                print("CAN =", pot)     # affichage sur la console REPL de la valeur numérique
                sleep_ms(100)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def ultrason(trig, echo):
        try:
            trigger = Pin(trig, Pin.OUT)
            echo = Pin(echo, Pin.IN)
            while True:
                trigger.value(1)
                sleep_us(10)
                trigger.value(0)
                while echo.value() == 0:
                    debut_impulsion = ticks_us()
                while echo.value() == 1:
                    fin_impulsion = ticks_us()
                distance = (fin_impulsion - debut_impulsion) / 58.0
                print(distance)
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def dynamixel(Pin):
        try:
            ser = serial.Serial("COM4", 1000000)
            if not ser.isOpen():
                ser.open()
            serial_port = "COM4"
            print(serial_port)

            motors = []
            # Wait for the motor to "reboot..."
            for _ in range(10):
                with Dxl320IO(serial_port, baudrate=1000000) as io:
                    sleep(1)
                    motors = (io.scan(range(20)))
                    if io.ping(1):
                        break
            else:
                print("Could not communicate with the motor...")
                print("Make sure one (and only one) is connected and try again")
                print("If the issue persists, use Dynamixel wizard to attempt a firmware recovery")
                sys.exit(1)

            print("Success!")
            print("Found motor(s): {}".format(motors))
        except KeyboardInterrupt:
            print("Program interrupted by user")

 
    def servo_moteur(pwm):
        try:
            p = Pin(13)
            servo = PWM(p,freq=50)

            while True :
                servo.duty(110)
                sleep(2)
                servo.duty(88)
                sleep(2)
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def IR(out):
        try:
            ir = Pin(Pin, out)
            while True:
                if ir.value() == 1:
                    print("IR detected")
                else:
                    print("IR not detected")
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def oled(pin1, pin2):
        try:
            i2c = machine.I2C(-1, machine.Pin(pin1), machine.Pin(pin2))
            DISPLAY_ADDR = 0x3C
            DISPLAY_WIDTH = 128
            DISPLAY_HEIGHT = 128
            display = ssd1306.SSD1306_I2C(DISPLAY_WIDTH, DISPLAY_HEIGHT, i2c, DISPLAY_ADDR)
            display.fill(0)
            display.show()
            display.text("HelloWorld ", 0, 0)
            display.text("Test", 0, 10)
            display.show()
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def fingerprint(self, pin1, pin2):
        try:
            i2c = self.I2C(scl=Pin(pin1), sda=Pin(pin2))
            fingerprint = Fingerprint3(i2c)
            fingerprint.initialize()
            while True:
                print("\nSelect an option:")
                print("1. Verify fingerprint")
                print("2. Enroll fingerprint")
                option = input("Enter your choice (1 or 2): ")
                if option == '1':
                    self.verify_fingerprint()
                elif option == '2':
                    self.time
        except KeyboardInterrupt:
            print("Program interrupted by user")



    def enroll_fingerprint():
        print("Place your finger on the sensor...")
        while not Fingerprint.read_image():
            sleep_ms(200)
        Fingerprint.enroll_start(1)
        print("Remove your finger...")
        while Fingerprint.read_image():
            sleep_ms(200)
        print("Place your finger again...")
        while not Fingerprint.read_image():
            sleep_ms(200)
        if Fingerprint.enroll1() == Fingerprint3.OK:
            print("First enrollment successful.")
            print("Remove your finger...")
            sleep(2)
            print("Place your finger again...")
            while not Fingerprint.read_image():
                sleep_ms(200)
            if Fingerprint.enroll2() == Fingerprint3.OK:
                print("Second enrollment successful.")
                print("Remove your finger...")
                sleep(2)
                print("Place your finger again...")
                while not Fingerprint.read_image():
                    sleep_ms(200)
                if Fingerprint.enroll3() == Fingerprint3.OK:
                    print("Third enrollment successful.")
                    Fingerprint.store()
                    print("Fingerprint enrolled.")
                else:
                    print("Enrollment failed.")
            else:
                print("Enrollment failed.")
        else:
            print("Enrollment failed.")


    def gyroscope_sensor(scl_pin, sda_pin):
        try:
            mpu = MPU6050(scl_pin, sda_pin)
            mpu.dmp_initialize()
            print("Gyroscope prêt")
            while True:
                accel_data = mpu.get_acceleration()
                gyro_data = mpu.get_rotation()
                print("Accélération [x, y, z]:", accel_data)
                print("Rotation [x, y, z]:", gyro_data)
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    
    def accelerometr(self, pin1, pin2):
        try:
            i2c = self.I2C(scl=Pin(pin1), sda=Pin(pin2)) 
            intertial_sensor = lsm6dso.LSM6DSO(i2c) # Instanciation du capteur
            intertial_sensor.scale_g('2000') # Moindre sensibilité pour les mesures angulaires
            intertial_sensor.scale_a('2g') # Sensibilité maximum pour les mesures d'accélérations
            while True:
                sleep_ms(100)
                # Mesures des accélérations selon les 3 axes 
                ax = intertial_sensor.ax()
                ay = intertial_sensor.ay()
                az = intertial_sensor.az()
                # Mesures des angles de rotation selon les 3 axes 
                gx = intertial_sensor.gx()
                gy = intertial_sensor.gy()
                gz = intertial_sensor.gz()
                print("ax : " + str(ax) + " mg")	
                print("ay : " + str(ay) + " mg")
                print("az : " + str(az) + " mg")
                print("Gx = " + str(gx/1000) + " °/s")
                print("Gy = " + str(gy/1000) + " °/s")
                print("Gz = " + str(gz/1000) + " °/s")
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def init_camera(self):
        i2c.writeto(self.camera_address, b'\x56\x00')
        sleep(1)
        i2c.writeto(self.camera_address, b'\x56\x36\x01\x00')
        sleep(1)
        i2c.writeto(self.camera_address, b'\x56\x31\x05\x00')
        sleep(1)


    def capture_photo(self):
        i2c.writeto(self.camera_address, b'\x56\x36\x01\x00')
        sleep(1)
        
        i2c.writeto(self.camera_address, b'\x56\x34\x01\x00')
        sleep(1)
        data = i2c.readfrom(self.camera_address, 0x32)
        with open('photo.jpg', 'wb') as file:
            file.write(data)
        print("Photo captured and saved!")

    def camera(self, pin1, pin2):
        i2c = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
        self.init_camera()
        self.capture_photo()
        camera_address = 0x30



        


                