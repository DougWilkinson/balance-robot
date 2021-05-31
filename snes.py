import machine,time

init = b'\x40\x00'
decrypt = b'\xF0\x55'
decrypt2 = b'\xFB\x00'
getdevice = b'\xFA'
startread = b'\x00'

class SNES:
    
    def write(self,msg=startread):
        self.snes.writeto(82, msg)

    def read(self):
        return self.snes.readfrom(82,6)

    def __init__(self, sdapin=12, sclpin=14):
        self.snes = machine.I2C(sda=machine.Pin(sdapin),scl=machine.Pin(sclpin))
        self.cx = .5
        self.cy = .5
        self.connected = False
        print("Initializing ...")
        self.write(init)
        time.sleep_ms(300)
        print("Disabling Decrypt ...")
        self.write(decrypt)
        time.sleep_ms(300)
        self.write(decrypt2)
        time.sleep_ms(300)
        print("Getting Device ID ...")
        self.write(getdevice)
        print("Device#: ")
        print(self.read())
        self.write()
        self.keys = self.read()

    def spin(self):
        self.write()
        keys = self.read()
        self.connected = True
        self.cx = .5
        self.cy = .5
        if keys == b'\xa0 \x10\x00\xff\xff':
           self.connected = False
           return
        if keys[5] & 1 == False:  # Up Key
            self.cy = 1
        if keys[5] & 2 == False:   # Left key
            self.cx = 0
        if keys[4] & 64 == False:  # Down Key
            self.cy = 0
        if keys[4] & 128 == False:   # Right key
            self.cx = 1
        #print("cx: ", self.cx, "cy: ", self.cy)
        return
