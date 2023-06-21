from platform import machine
from aiohttp import Fingerprint
from machine import Pin, PWM, ADC, I2C, UART 
from time import sleep, sleep_ms, sleep_us, ticks_us
import tm1637
import esp_rgb_lcd_grove
import neopixel
import pypot.dynamixel as dynamixel
import serial
import Adafruit_DHT
import bme280
import sys
import ssd1306
import lsm6dso
import math
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from pypot.dynamixel.conversion import dynamixelModels
from pypot.dynamixel import DxlIO, Dxl320IO, get_available_ports
from pypot.utils import flushed_print as print
from fingerprint3 import Fingerprint3
from MPU6050 import MPU6050
from mpu9250 import MPU9250
from onewire import OneWire
from ds18x20 import DS18X20
from as7341 import AS7341


class TestClass:
    melody = []

    def sept_segments(PinIn, PinOut):
        """
        La fonction permet d'afficher un message sur un afficheur 7 segments
        """
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
        """
        La fonction permet de faire sonner un buzzer
        """
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
        """
        La fonction permet d'afficher un message sur l'écran LCD
        les paramètres PinIn et PinOut correspondent aux pins SCL et SDA
        la fonction renvoie le message "Hello World" sur l'écran LCD
        """
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
        """
        La fonction permet d'activer ou de désactiver un électroaimant
        les paramètres PinIn et PinOut correspondent aux pins de l'électroaimant
        """
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
        """
        La fonction permet de jouer une mélodie
        les paramètres PinIn et PinOut correspondent aux pins du haut-parleur
        """
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
        """
        La fonction permet de lire l'état d'un bouton poussoir
        """
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
        """
        La fonction permet d'allumer une LED
        """
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
        """
        La fonction permet de faire tourner un moteur brushless
        le paramètre Pin correspond à la pin du moteur 
        """
        try:
            motor = PWM(Pin(Pin),freq=50)
            duty_cycle = 50 # Between 0 - 100 %
            while True:
                motor.duty(int((duty_cycle/100)*1024))
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def DC_motor(Pin):
        """
        La fonction permet de faire tourner un moteur à courant continu
        le paramètre Pin correspond à la pin du moteur
        """
        try:
            motor = PWM(Pin(Pin),freq=50)
            duty_cycle = 50 # Between 0 - 100 %
            while True:
                motor.duty(int((duty_cycle/100)*1024))
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")

    def stepper_motor(Pin1, Pin2, Pin3, Pin4):
        """
        La fonction permet de faire tourner un moteur pas à pas
        les paramètres Pin1, Pin2, Pin3 et Pin4 correspondent aux pins du moteur
        la fonction renvoie la rotation du moteur
        """
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
        """
        La fonction permet de lire les données du potentiomètre
        le paramètre pin correspond à la pin du potentiomètre
        la fonction renvoie la valeur du potentiomètre
        """
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
        """
        la fonction permet de lire les données du capteur ultrason
        les paramètres trig et echo correspondent aux pins trig et echo du capteur
        la fonction renvoie la distance entre le capteur et l'objet détecté
        """
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
        """
        La fonction permet de lire les données du moteur dynamixel
        le paramètre pin correspond à la pin du moteur
        """
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
        """
        La fonction permet de faire tourner un servo moteur
        le paramètre pwm correspond à la pin du servo moteur
        """
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
        """
        La fonction permet de lire les données du capteur infrarouge
        le paramètre out correspond à la pin du capteur
        la fonction renvoie si un objet est détecté ou non
        """
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
        """
        La fonction permet d'afficher un message sur l'écran OLED
        les paramètres pin1 et pin2 correspondent aux pins SCL et SDA
        la fonction renvoie le message "HelloWorld" puis "Test" sur l'écran OLED
        la fonction fonctionne en I2C
        """
        try:
            I2C = machine.I2C(-1, machine.Pin(pin1), machine.Pin(pin2))
            DISPLAY_ADDR = 0x3C
            DISPLAY_WIDTH = 128
            DISPLAY_HEIGHT = 128
            display = ssd1306.SSD1306_I2C(DISPLAY_WIDTH, DISPLAY_HEIGHT, I2C, DISPLAY_ADDR)
            display.fill(0)
            display.show()
            display.text("HelloWorld ", 0, 0)
            display.text("Test", 0, 10)
            display.show()
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def fingerprint(self, pin1, pin2):
        """ 
        La fonction permet de lire les données du capteur d'empreintes digitales en I2C
        les paramètres pin1 et pin2 correspondent aux pins SCL et SDA
        la fonction renvoie si l'empreinte est reconnue ou non:
        - si l'empreinte est reconnue, le message "Fingerprint verified" s'affiche
        - si l'empreinte n'est pas reconnue, le message "Fingerprint not verified" s'affiche
        la fonction fonctionne en I2C
        """
        try:
            I2C = I2C(scl=Pin(pin1), sda=Pin(pin2))
            fingerprint = Fingerprint3(I2C)
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
        """
        La fonction permet d'enregistrer une empreinte digitale
        """
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
        """
        La fonction permet de lire les données du gyroscope en I2C
        les paramètres scl_pin et sda_pin correspondent aux pins SCL et SDA
        la fonction renvoie les données de l'accéléromètre et du gyroscope :
        - rotation selon les 3 axes
            - gx, gy, gz
        ...
        """
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

    
    def accelerometre(self, pin1, pin2):
        """
        La fonction permet de lire les données de l'accelelromètre en I2C
        les paramètres pin1 et pin2 correspondent aux pins SCL et SDA
        la fonction renvoie les données de l'accéléromètre et du gyroscope :
        - accélération selon les 3 axes
            - ax, ay, az
        ...
        """
        try:
            I2C = I2C(scl=Pin(pin1), sda=Pin(pin2)) 
            intertial_sensor = lsm6dso.LSM6DSO(I2C) # Instanciation du capteur
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
        """
        cette fonction permet d'initialiser la caméra
        """
        I2C.writeto(self.camera_address, b'\x56\x00')
        sleep(1)
        I2C.writeto(self.camera_address, b'\x56\x36\x01\x00')
        sleep(1)
        I2C.writeto(self.camera_address, b'\x56\x31\x05\x00')
        sleep(1)


    def capture_photo(self):
        """
        Cette fonction permet de capturer une photo avec la caméra
        """
        I2C.writeto(self.camera_address, b'\x56\x36\x01\x00')
        sleep(1)
        I2C.writeto(self.camera_address, b'\x56\x34\x01\x00')
        sleep(1)
        data = I2C.readfrom(self.camera_address, 0x32)
        with open('photo.jpg', 'wb') as file:
            file.write(data)
        print("Photo captured and saved!")

    def camera(self, pin1, pin2):
        """
        La fonction permet de lire les données de la caméra en I2C
        les paramètres pin1 et pin2 correspondent aux pins SCL et SDA
        la fonction renvoie une photo prise par la caméra
        """
        I2C = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
        self.init_camera()
        self.capture_photo()
        camera_address = 0x30

    def humidity_sensor(pin):
        """
        La fonction permet de lire les données du capteur d'humidité
        le paramètre pin correspond à la pin du capteur
        la fonction renvoie l'humidité et la température    
        """
        try:
            # capteur DHT11 ou DHT22
            sensor = Adafruit_DHT.DHT11
            humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)
            if humidity is not None and temperature is not None:
                print('Humidité : {0:.2f}%'.format(humidity))
                print('Température : {0:.2f}°C'.format(temperature))
            else:
                print('error')

        except Exception as e:
            print('Erreur:', str(e))

    
    def IMU(pin1, pin2):
        """
        La fonction permet de lire les données du capteur IMU en I2C
        le paramètre pin1 correspond à la pin SCL
        le paramètre pin2 correspond à la pin SDA
        la fonction renvoie les données de l'accéléromètre, du gyroscope et du magnétomètre
        """
        try:
            I2C = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
            devices = I2C.scan()
            print('I2C devices found:', devices)
            imu = MPU9250(I2C)
            while True:
                accel = imu.acceleration
                gyro = imu.gyro
                mag = imu.magnetic
                print('Accelerometer (m/s^2):', accel)
                print('Gyroscope (rad/s):', gyro)
                print('Magnetometer (uT):', mag)
                roll = math.atan2(accel[1], accel[2]) * 180 / math.pi
                pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180 / math.pi
                print('Roll:', roll)
                print('Pitch:', pitch)
                sleep(0.1)
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def joystick(vrx, vry, sw):
        """
        la fonction permet de lire les données du joystick
        les paramètres vrx, vry et sw correspondent aux pins du joystick
        vrx correspond à l'axe X
        vry correspond à l'axe Y
        sw correspond au bouton sous le joystick
        la fonction renvoie les valeurs des axes X et Y ainsi que l'état du bouton
        """
        try:
            vrx = ADC(Pin(vrx))
            vry = ADC(Pin(vry))
            sw = Pin(sw, Pin.IN, Pin.PULL_UP)
            while True:
                print("X : " + str(vrx.read()))
                print("Y : " + str(vry.read()))
                print("SW : " + str(sw.value()))
                sleep(1)
                if 780 >= vrx >= 750:
                    print("Haut")
                if 280 >= vrx >= 240:
                    print("Bas")
                if 780 >= vry >= 750:
                    print("Gauche")
                if 280 >= vry >= 240:
                    print("Droite")
                if vrx >= 1000:
                    print("test")
        except KeyboardInterrupt:
            print("Program interrupted by user")  


    def micro(data):
        """
        La fonction permet de lire les données du capteur sonore
        le paramètre data correspond à la pin du capteur
        la fonction renvoie si un son est détecté ou non
        """
        try:
            pin_microphone = Pin(data)
            while True:
                valeur_microphone = pin_microphone.value()  # Lire la valeur du capteur sonore
                if valeur_microphone == 0:
                    print("Pas de son détecté")
                else:
                    print("Son détecté")
                sleep(0.1)  # Attendre 100ms avant de recommencer la boucle 
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def pression(pin1, pin2):
        """
        La fonction permet de lire les données du capteur de pression en I2C
        le paramètre pin1 correspond à la pin SCL
        le paramètre pin2 correspond à la pin SDA
        la fonction renvoie la température, l'humidité et la pression
        """
        try:
            I2C = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
            devices = I2C.scan()
            print('I2C devices found:', devices)
            bme = bme280.BME280(I2C=I2C)

            while True:
                temperature = bme.temperature
                humidity = bme.humidity
                pressure = bme.pressure
                print('Temperature (C):', temperature)
                print('Humidity (%):', humidity)
                print('Pressure (hPa):', pressure)
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")
    

    def proximite_capacitif(data):
        try:
            sensor = Pin(data)
            while True:
                if sensor :
                    print("metaux présent")
                else:
                    print("nothing")
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def read_RFID_data(self, pin):
        """
        La fonction permet de lire les données du lecteur RFID
        le paramètre pin correspond à la pin du lecteur
        le lecteur fonctionne en UART
        la fonction renvoie l'identifiant RFID en hexa et en decimal
        """
        try:
            uart = UART(pin, 9600)
            uart.init(9600, bits=8, parity=None, stop=1, tx=25, rx=26)
            while True:
                if uart.any() > 0:
                    donnee_tag = uart.readline()
                    print("Donnees tag RFID :            ", donnee_tag)

                    rfid_hexa = ""
                    for i in range(5, 11):
                        rfid_hexa += chr(donnee_tag[i])
                    print("Identifiant RFID en hexa :    ", rfid_hexa)

                    rfid = str(int(rfid_hexa, 16))
                    while len(rfid) < 10:
                        rfid = "0" + rfid
                    print("Identifiant RFID en decimal : ", rfid)
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def sonde_thermique(pin):
        """
        La fonction permet de lire les données de la sonde thermique
        le paramètre pin correspond à la pin de la sonde
        le capteur fonctionne en OneWire
        la fonction renvoie la température
        """
        try:
            ow = OneWire(machine.Pin(pin))  
            ds = DS18X20(ow)
            devices = ds.scan()
            print('DS18B20 devices found:', devices)
            while True:
                ds.convert_temp()
                sleep_ms(750)
                temperatures = []
                for device in devices:
                    temp = ds.read_temp(device)
                    temperatures.append(temp)
                for i, temp in enumerate(temperatures):
                    print('Sensor', i + 1, 'Temperature:', temp, '°C')
                sleep(1)
        except KeyboardInterrupt:
            print("Program interrupted by user")


    def spectrometre(pin1, pin2):
        """
        La fonction permet de lire les données du spectromètre
        le paramètre pin1 correspond à la pin SCL
        le paramètre pin2 correspond à la pin SDA
        le spectrographe fonctionne en I2C
        la fonction renvoie les données spectrales
        """
        i2c = machine.I2C(scl=machine.Pin(pin1), sda=machine.Pin(pin2))
        devices = i2c.scan()
        print('I2C devices found:', devices)
        as7341 = AS7341(i2c)
        as7341.setup(mode=AS7341.MODE_1)
        while True:
            spectral_data = as7341.get_spectral_data()
            print('Spectral Data:', spectral_data)
            sleep(1)

    
    def bouton_poussoir(pin):
        """
        La fonction permet de lire l'état d'un bouton poussoir
        la fonction renvoie l'état du bouton
        """
        try:
            button = Pin(pin)
            while True:
                if button.value() == 1:
                    print("bp pressed")
                else:
                    print("bp not pressed")
        except KeyboardInterrupt:
            print("Program interrupted by user")