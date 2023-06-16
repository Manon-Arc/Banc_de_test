from machine import ADC, Pin

adc = ADC(Pin(32))
adc.atten(ADC.ATTN_11DB)

def read_cny70():
    return adc.read()

def calculate_distance(sensor_value):
    voltage = sensor_value * 3.3 / 4095  # Convert sensor value to voltage
    distance = 17.63 / (voltage - 0.173)  # Calculate distance using a linear formula
    return distance

while True:
    cny70_value = read_cny70()
    distance = calculate_distance(cny70_value)
    print("CNY70 value:", cny70_value)
    print("Distance:", distance, "cm")