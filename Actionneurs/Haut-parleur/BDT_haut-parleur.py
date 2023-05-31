import machine
import time
broche_sortie = machine.Pin(4, machine.Pin.OUT)

notes = [65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, 110.00, 116.54, 123.47]
nombre_de_notes = 32
tempo = 150

melodie = [
    [4, 2, 2], [5, 2, 1], [7, 2, 3], [0, 3, 6],
    [2, 2, 2], [4, 2, 1], [5, 2, 8],
    [7, 2, 2], [9, 2, 1], [11, 2, 3], [5, 3, 6],
    [9, 2, 2], [11, 2, 1], [0, 3, 3], [2, 3, 3], [4, 3, 3],
    [4, 2, 2], [5, 2, 1], [7, 2, 3], [0, 3, 6],
    [2, 3, 2], [4, 3, 1], [5, 3, 8],
    [7, 2, 2], [7, 2, 1], [4, 3, 3], [2, 3, 2],
    [7, 2, 1], [5, 3, 3], [4, 3, 2], [2, 3, 1], [0, 3, 8]
]

pwm = machine.PWM(broche_sortie)
pwm.freq(0)

def song():
    for i in range(nombre_de_notes):
        frequence = round(notes[melodie[i][0]] * 2.0 * (melodie[i][1] - 1))
        pwm.freq(frequence)
        pwm.duty(512)  # Rapport cyclique à 50%
        time.sleep_ms(tempo * melodie[i][2] - 50)
        pwm.duty(0)  # Rapport cyclique à 0% (silence)
        time.sleep_ms(50)

    time.sleep_ms(2000)

while True:
    song()
    time.sleep(0.5)