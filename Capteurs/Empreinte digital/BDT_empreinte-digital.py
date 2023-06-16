from pyfingerprint.pyfingerprint import PyFingerprint
import time

def fingerprint_sensor(tx_pin, rx_pin):
    try:
        f = PyFingerprint(tx=tx_pin, rx=rx_pin, baudrate=57600)
        while True:
            if f.read_image():
                f.convert_image(0x01)
                result = f.search_template()
                if result >= 0:
                    print('Empreinte digitale reconnue')
                else:
                    print('Empreinte digitale inconnue')
            time.sleep(1)
    except Exception as e:
        print('Erreur:', str(e))
