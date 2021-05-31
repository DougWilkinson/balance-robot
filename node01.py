# esp32 jjrobot ported from urobot

import machine,time
#import graphics
#from ssd1306 import SSD1306
import network

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect('xxxxxx','xxxxxx')

from mpu6050 import MPU6050
imu = MPU6050(2,False)

# set up stepper motors
from nemastepper import Stepper
motor1 = Stepper(32,33,27)
motor2 = Stepper(5,17,18)

motor1.MAX_ACCEL = 500
motor2.MAX_ACCEL = 500

#import oscserver
#import oscclient
import socket                                                                                

oscrx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
oscrx.setblocking(0)

addr = socket.getaddrinfo('192.168.1.147','2222')
addr = addr[0][-1]
oscrx.bind(addr)

def issr(t):
    global motor1, motor2
    motor1.do_step()
    motor2.do_step()
    
tim = machine.Timer(1)


# Complementary Filter A = rt/(rt + dt) where rt is response time, dt = period
def compf(fangle,accel,gyro,looptime,A):
    fangle = A * (fangle + gyro * looptime/1000000) + (1-A) * accel
    return fangle

#  graphic display of accel angle & filtered angle
#   - primarily used in development but also for initial setup
def align():
    start = time.ticks_us()
    cangle = 90.0
    while abs(cangle)>2.0:
        angle  = imu.pitch()
        cangle = compf(cangle, angle, imu.get_gy(), time.ticks_diff(time.ticks_us(),start),0.91) 
        start = time.ticks_us()
        #print("angle: ", angle," cangle: ", cangle)
        #graphics.line(lcd,32,26,angle,24,1)
        #graphics.line(lcd,96,26,cangle,24,1)
        #lcd.display()
        #graphics.line(lcd,32,26,angle,24,0)
        #graphics.line(lcd,96,26,cangle,24,0)
    #lcd.clear()
    print("angle: ", angle," cangle: ", cangle)
    print("Start balancing!.")
    #lcd.text("Start balancing!.",0,24,1)
    #lcd.text('zero:{:5.2f}'.format(cangle),0,32,1)
    #lcd.display()

cx = 0
cy = 0

def spin():
    global cx,cy
    try:
        data, caddr = oscrx.recvfrom(100)
        if data..startswith(b'/xy'):
            cx,cy = struct.unpack('>ff',data[8:])
            print(cx,cy)
        return True
    except:
        return True

MAX_VEL = 6000 # 2000 usteps/sec = 500steps/sec = 2.5rps = 150rpm
MAX_ANGLE = 10  # degrees of tilt for speed control

def constrain(val,minv,maxv):
    if val<minv:
        return minv
    elif val>maxv:
        return maxv
    else:
        return val

#stability PD controiller - input is target angle, output is acceleration
K = 6 # 7
Kp = 4.0
Kd = 0.5
def stability(target,current,rate):
    global K,Kp,Kd
    error = target - current
    output = Kp * error - Kd*rate
    return int(K*output)

#speed P controiller - input is target speed, output is inclination angle
KpS = 0.01
def speedcontrol(target,current):
    global KpS
    error = target - current
    output = KpS * error 
    return constrain(output,-MAX_ANGLE,+MAX_ANGLE)

tangle = 0
#main balance loop runs every 5ms
def balance():
    global tangle
    gangle = 0.0
    start = time.ticks_us()
    controlspeed = 0
    fspeed = 0
    while abs(gangle) < 45 and spin():  # give up if inclination angle >=45 degrees
        angle  = imu.pitch()
        rate   = imu.get_gy()
        gangle = compf(gangle, angle, rate, time.ticks_diff(time.ticks_us(),start),0.99)         
        start = time.ticks_us()
        # speed control
        actualspeed = (motor1.get_speed()+motor2.get_speed())/2
        fspeed = 0.95 * fspeed + 0.05 * actualspeed
        #cmd = radio.poll() # cmd[0] is turn speed, cmd[1] is fwd/rev speed
        #tangle = speedcontrol(800*cmd[1],fspeed)
        # stability control
        controlspeed += stability(tangle, gangle, rate)           
        controlspeed = constrain(controlspeed,-MAX_VEL,MAX_VEL)
        # set motor speed
        #motor1.set_speed(-controlspeed-int(300*cmd[0]))
        #motor2.set_speed(-controlspeed+int(300*cmd[0]))
        motor1.set_speed(-controlspeed)
        motor2.set_speed(-controlspeed)
        time.sleep_us(5000-time.ticks_diff(time.ticks_us(),start))
    # stop and turn off motors
    motor1.set_speed(0)
    motor2.set_speed(0)
    motor1.set_off()
    motor2.set_off()

# main program

def main():
    while True:
        align()
        #tim.init(period=1, mode=machine.Timer.PERIODIC, callback=issr) #start interrupt routine
        tim.init(freq=2000, mode=machine.Timer.PERIODIC, callback=issr) #start interrupt routine
        balance()
        tim.deinit() #stop interrupt routine

