from machine import Pin, UART

uart = UART(1, 9600)
uart.init(9600, bits=8, parity=None, stop=1, tx=25, rx=26 )

while True:

    if uart.any()>0:

        donnee_tag = uart.readline()
        print("Donnees tag RFID :            ", donnee_tag)

        # On isole et on décode les 5 octets de poids faible du champ "10 ASCII Data Characters"
        rfid_hexa = ""
        for i in range(5, 11):
            rfid_hexa += chr(donnee_tag[i])
        print("Identifiant RFID en hexa :    ",  rfid_hexa)

        # L'identifiant RFID indiqué sur le tag est obtenu par conversion hexadecimal décimale
        rfid = str(int(rfid_hexa, 16))
        while len(rfid)<10:
            rfid = "0" + rfid
        print("Identifiant RFID en decimal : ", rfid)    