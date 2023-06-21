class cam:
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
