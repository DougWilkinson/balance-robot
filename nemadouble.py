import machine



class Stepper:
    """
    Handles  A4988 hardware driver for bipolar stepper motors
    """

    
    def __init__(self, dir_pin, step_pin, enable_pin):
        self.step_pin = machine.Pin(step_pin, machine.Pin.OUT)
        self.dir_pin = machine.Pin(dir_pin, machine.Pin.OUT)
        self.enable_pin = machine.Pin(enable_pin, machine.Pin.OUT)
        self.enable_pin.on()       
        self.dir = 0
        self.pulserate = 1
        self.count = 0
        self.speed = 0
        self.MAX_ACCEL = 100   #equivallent to 100 x (periodicity of set_speed) usteps/sec/sec
 

    def do_step(self):   # called by timer interrupt every 100us
        if self.dir == 0:
            return
        self.count = (self.count+1)%self.pulserate
        if self.count == 0:
            self.step_pin.on()
            pass
            self.step_pin.off()
            pass
            self.step_pin.on()
            pass
            self.step_pin.off()
        
    def set_speed(self, speed): #called periodically
        if (self.speed - speed) > self.MAX_ACCEL:
            self.speed -= self.MAX_ACCEL
        elif (self.speed - speed)< -self.MAX_ACCEL:
            self.speed+=self.MAX_ACCEL
        else:
            self.speed = speed
        # set direction
        if self.speed>0:
            self.dir = 1
            self.dir_pin.on()
            self.enable_pin.off()       
        elif self.speed<0:
            self.dir = -1
            self.dir_pin.off()
            self.enable_pin.off()       
        else:
            self.dir = 0
        if abs(self.speed)>0:
            self.pulserate = 3000//abs(self.speed)

    def set_off(self):
        self.enable_pin.on()

    def get_speed(self):
        return self.speed
       

                
            
        
            
    

