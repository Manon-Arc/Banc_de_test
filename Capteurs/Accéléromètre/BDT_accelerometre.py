from machine import I2C
import lsm6dso # Pilote de la centrale à inertie
from time import sleep_ms

i2c = I2C(1) 

# Pause d'une seconde pour laisser à l'I2C le temps de s'initialiser
sleep_ms(1000)

# Liste des adresses I2C des périphériques présents
print("Adresses I2C utilisées : " + str(i2c.scan()))

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