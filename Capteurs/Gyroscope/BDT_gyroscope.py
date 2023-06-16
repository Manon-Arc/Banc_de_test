from MPU6050 import MPU6050

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
            time.sleep(1)
    except Exception as e:
        print('Erreur:', str(e))
