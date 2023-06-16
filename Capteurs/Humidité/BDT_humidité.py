import Adafruit_DHT

def humidity_sensor(pin):
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
