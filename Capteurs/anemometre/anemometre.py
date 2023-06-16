import RPi.GPIO as GPIO
import time

def anemometre(pin_anemo):
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pin_anemo, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    tour_count = 0
    last_state = GPIO.input(pin_anemo)
    
    def anemo_callback(channel):
        nonlocal tour_count, last_state
        current_state = GPIO.input(pin_anemo)
        
        if current_state != last_state:
            tour_count += 1
            last_state = current_state
    
    GPIO.add_event_detect(pin_anemo, GPIO.BOTH, callback=anemo_callback)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
