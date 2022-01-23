from machine import Timer
import utime
import ustruct
import sys
import uctypes
from ulab import numpy as np
from ulab import scipy as sc
import math


##### REGISTER MAP #####
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

SELF_TEST_X_ACCEL = 0x0D
SELF_TEST_Y_ACCEL = 0x0E
SELF_TEST_Z_ACCEL = 0x0F

SELF_TEST_X_GYRO = 0x00
SELF_TEST_Y_GYRO = 0x01
SELF_TEST_Z_GYRO = 0X02

ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C

ACCEL_WHO_AM_I = 0x75
GYRO_WHO_AM_I = 0x75

########################

GYRO_cs = machine.Pin(21, machine.Pin.OUT)
ACCEL_cs = machine.Pin(17, machine.Pin.OUT)
timer = Timer()

# Initialize SPI
spi = machine.SPI(0,
                  baudrate=1000000,
                  polarity=1,
                  phase=1,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(18),
                  mosi=machine.Pin(19),
                  miso=machine.Pin(20))



def reg_write(spi, cs, reg, data):
    """
    Write 1 byte to the specified register.
    """
    
    # Construct message (set ~W bit low, MB bit low)
    msg = bytearray()
    msg.append(0x00 | reg)
    msg.append(data)
    
    # Send out SPI message
    cs.value(0)
    spi.write(msg)
    cs.value(1)



def reg_read(spi, cs, reg, nbytes=1):
    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """
    
    # Determine if multiple byte (MB) bit should be set
    if nbytes < 1:
        return bytearray()
    elif nbytes == 1:
        mb = 0
    else:
        mb = 1
    
    # Construct message (set ~W bit high)
    msg = bytearray()
    msg.append(0x80 | (mb << 6) | reg)
    
    # Send out SPI message and read
    cs.value(0)
    spi.write(msg)
    data = spi.read(nbytes)
    cs.value(1)
    
    return data


# from stackoverflow J.F. Sebastian
def _twos_comp(val, bits=16):
    """compute the 2's complement of int val with bits"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set
        val = val - (1 << bits)          # compute negative value
    return val


def ACCEL_set_2g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x00)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)


def ACCEL_set_4g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x08)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)


def ACCEL_set_8g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x10)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)


def ACCEL_set_16g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x18)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)


def GYRO_set_250dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x00)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    

def GYRO_set_500dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x10)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    
def GYRO_set_1000dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x08)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)


def GYRO_set_2000dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x18)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)



def ACCEL_data():
    
    #################### X axis ############################
    dataX = reg_read(spi, ACCEL_cs, ACCEL_XOUT_L, 1)
    dataX = dataX + reg_read(spi, ACCEL_cs, ACCEL_XOUT_H, 1)
    ax = int.from_bytes(dataX,'little')
    ax = _twos_comp(ax)
    
    #################### Y axis ############################
    dataY = reg_read(spi, ACCEL_cs, ACCEL_YOUT_L, 1)
    dataY = dataY + reg_read(spi, ACCEL_cs, ACCEL_YOUT_H, 1)
    ay = int.from_bytes(dataY,'little')
    ay = _twos_comp(ay)   
    
    #################### Z axis ############################
    dataZ = reg_read(spi, ACCEL_cs, ACCEL_ZOUT_L, 1)
    dataZ = dataZ + reg_read(spi, ACCEL_cs, ACCEL_ZOUT_H, 1)
    az = int.from_bytes(dataZ,'little')
    az = _twos_comp(az)
    
    return ax,ay,az


def GYRO_data():
    
    #################### X axis ############################
    dataX = reg_read(spi, GYRO_cs, GYRO_XOUT_L, 1)
    dataX = dataX + reg_read(spi, GYRO_cs, GYRO_XOUT_H, 1)
    wx = int.from_bytes(dataX,'little')
    wx = _twos_comp(wx)
    
    #################### Y axis ############################
    dataY = reg_read(spi, GYRO_cs, GYRO_YOUT_L, 1)
    dataY = dataY + reg_read(spi, GYRO_cs, GYRO_YOUT_H, 1)
    wy = int.from_bytes(dataY,'little')
    wy = _twos_comp(wy)
    
    #################### Z axis ############################
    dataZ = reg_read(spi, GYRO_cs, GYRO_ZOUT_L, 1)
    dataZ = dataZ + reg_read(spi, GYRO_cs, GYRO_ZOUT_H, 1)
    wz = int.from_bytes(dataZ,'little')
    wz = _twos_comp(wz)
    
    return -wx,-wy,-wz


def run():
    ax,ay,az = ACCEL_data()
    wx,wy,wz = GYRO_data()
    print((ax/8192),",",(ay/8192),",",(az/8192),",",((wx/32768)*250),",",((wy/32768)*250),",",((wz/32768)*250))
    


pitch = 0
roll = 0
yaw = 0

estQuaternion = np.array([[0],
                          [1],
                          [2],
                          [3]],dtype=np.float)

corrQuaternion = np.array([[0],
                           [0],
                           [0],
                           [0]],dtype=np.float)

estP = np.array([[1,0,0,0],
                 [0,1,0,0],
                 [0,0,1,0],
                 [0,0,0,1]],dtype=np.float)*50

corrP = np.array([[1,0,0,0],
                  [0,1,0,0],
                  [0,0,1,0],
                  [0,0,0,1]],dtype=np.float)

R = np.array([[0.00095,0,0],
              [0,0.00095,0],
              [0,0,0.00095]],dtype=np.float)

Qt = np.array([[0,0,0,0],
               [0,0,0,0],
               [0,0,0,0],
               [0,0,0,0]],dtype=np.float)

Ew = np.array([[0.0087,0,0],
               [0,0.0087,0],
               [0,0,0.0087]],dtype=np.float)

hqt = np.array([[0],
                [0],
                [0]],dtype=np.float)
Hqt = np.array([[0,0,0,0],
                [0,0,0,0],
                [0,0,0,0]],dtype=np.float)

F = np.array([[0,0,0,0],
              [0,0,0,0],
              [0,0,0,0],
              [0,0,0,0]],dtype=np.float)

KalmanGain = np.array([[0,0,0],
                       [0,0,0],
                       [0,0,0],
                       [0,0,0]],dtype=np.float)

Vt = np.array([[0],
               [0],
               [0]],dtype=np.float)

Zt = np.array([[0],
               [0],
               [0]],dtype=np.float)

axb = 0 #g
ayb = 0 #g
azb = 0 #g
axc = 0 #g
ayc = 0 #g
azc = 0 #g
wxb = 0 #deg/s
wyb = 0 #deg/s
wzb = 0 #deg/s
wxc = 0 #deg/s
wyc = 0 #deg/s
wzc = 0 #deg/s
topax = 0
topay = 0
topaz = 0
topwx = 0
topwy = 0
topwz = 0
deltaT = 0.01 #second
timeStep = 1 #unit
time = 0 #second
frequency = 100 #hertz

def initialize(ax,ay,az):#kullanýldý
    
    global estQuaternion
    
    estQuaternion = np.array([[((az+1)/2)**0.5],
                              [-(ay/((2*(az+1))**0.5))],
                              [(ax/((2*(az+1))**0.5))],
                              [0]],dtype=np.float)
    


def estQua():#kullanýldý
    global estQuaternion,corrQuaternion,wxc,wyc,wzc,deltaT
    
    ident = np.eye(4)
    omega = np.array([[0,-wxc,-wyc,-wzc],
                      [wxc,0,wzc,-wyc],
                      [wyc,-wzc,0,wxc],
                      [wzc,wyc,-wxc,0]],dtype=np.float)*(deltaT/2)
    step1 = ident + omega
    estQuaternion = np.dot(step1,estQuaternion)
    
def estCovarianceMatrix():#kullanýldý
    global F,corrP,Qt,estP
    
    Ftr = F.transpose()
    step1 = np.dot(corrP,Ftr)
    step2 = np.dot(F,step1)
    estP = step2 + Qt


def transitionMatrix():#kullanýldý
    global wxc,wyc,wzc,deltaT,F
    
    ident = np.eye(4)
    
    omega = np.array([[0,-wxc,-wyc,-wzc],
                      [wxc,0,wzc,-wyc],
                      [wyc,-wzc,0,wxc],
                      [wzc,wyc,-wxc,0]],dtype=np.float)*(deltaT/2)
    
    F =  ident + omega
    

def processNoiseCovarianceMatrix():#kullanýldý
    
    global estQuaternion,deltaT,Qt,Ew
    qw = estQuaternion[0][0]
    qx = estQuaternion[1][0]
    qy = estQuaternion[2][0]
    qz = estQuaternion[3][0]
    
    Wt = np.array([[-qx,-qy,-qz],
                   [qw,-qz,qy],
                   [qz,qw,-qx],
                   [-qy,qx,qw]],dtype=np.float)*(deltaT/2)

    Wtr = Wt.transpose()
    
    step1 = np.dot(Ew,Wtr)
    Qt = np.dot(Wt,step1)


def measModel():#kullanýldý
    global hqt,estQuaternion,Hqt
    qw = estQuaternion[0][0]
    qx = estQuaternion[1][0]
    qy = estQuaternion[2][0]
    qz = estQuaternion[3][0]
    hqt = np.array([[qx*qz-qw*qy],
                    [qw*qx+qy*qz],
                    [0.5-qx**2-qy**2]],dtype = np.float)*2
    
    Hqt = np.array([[-qy,qz,-qw,qx],
                    [qx,qw,qz,qy],
                    [0,-2*qx,2*qy,0]],dtype = np.float)*2
    
def kalmangain():#kullanýldý
    global Hqt,estP,R,KalmanGain
    
    Htr = Hqt.transpose()
    step1 = np.dot(estP,Htr)
    step2 = np.dot(Hqt,step1)
    S = step2+R
    step3 = np.linalg.inv(S)
    step4 = np.dot(Htr,step3)
    KalmanGain = np.dot(estP,step4)
    
def correctedQuaternion():#kullanýldý
    global estQuaternion,KalmanGain,Vt,corrQuaternion
    
    step1 = np.dot(KalmanGain,Vt)
    corrQuaternion = estQuaternion+step1

def correctedCovarianceMatrix():#kullanýldý
    global KalmanGain,corrP,estP,Hqt
    
    ident = np.eye(4) 
    step1 = np.dot(KalmanGain,Hqt)
    step2 = ident - step1
    corrP = np.dot(step2,estP)
    

def AHRS(timer):
    
    global pitch,roll,yaw,estQuaternion,corrQuaternion,estP,corrP,R,Qt,Ew,hqt,Hqt,F,KalmanGain,Vt,Zt,deltaT,timeStep,time,topax,topay,topaz,topwx,topwy,topwz,axb,ayb,azb,wxb,wyb,wzb,wxc,wyc,wzc,axc,ayc,azc
    
    ax,ay,az = ACCEL_data()
    ax = ax/8192
    ay = ay/8192
    az = az/8192
    wx,wy,wz = GYRO_data()
    wx = (wx/32768)*250
    wy = (wy/32768)*250
    wz = (wz/32768)*250
    
    if timeStep <= 100:
        print("Don't Touch")
        topax += ax
        topay += ay
        topaz += az
        topwx += wx
        topwy += wy
        topwz += wz
    
    elif timeStep == 101:
        axb = topax/100
        ayb = topay/100
        azb = (topaz/100)-1
        wxb = topwx/100
        wyb = topwy/100
        wzb = topwz/100
        wxc = wx - wxb
        wyc = wy - wyb
        wzc = wz - wzb
        axc = ax - axb
        ayc = ay - ayb
        azc = az - azb
        initialize(axc,ayc,azc)
    
    elif timeStep > 101:
        
        wxc = math.radians(wx - wxb)
        wyc = math.radians(wy - wyb)
        wzc = math.radians(wz - wzb)
        axc = ax - axb
        ayc = ay - ayb
        azc = az - azb
        norm = 1/((axc**2 + ayc**2 + azc**2)**0.5)
        axcn = axc*norm
        aycn = ayc*norm
        azcn = azc*norm
        
        measModel()
        Zt = np.array([[axcn],
                       [aycn],
                       [azcn]],dtype = np.float)
        
        Vt = Zt-hqt
        kalmangain()
        correctedQuaternion()
        correctedCovarianceMatrix()
        #####################
        cqw = corrQuaternion[0][0]
        cqx = corrQuaternion[1][0]
        cqy = corrQuaternion[2][0]
        cqz = corrQuaternion[3][0]
        norm = cqw*cqw + cqx*cqx + cqy*cqy + cqz*cqz
        cqwn = corrQuaternion[0][0]/norm
        cqxn = corrQuaternion[1][0]/norm
        cqyn = corrQuaternion[2][0]/norm
        cqzn = corrQuaternion[3][0]/norm
        if timeStep % 20 == 0:
            roll = math.atan2((2*(cqwn*cqxn+cqyn*cqzn)),(1 - cqxn**2 - cqyn**2))
            pitch = math.asin(2*(cqwn*cqyn - cqxn*cqzn))
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            print(roll,pitch)
        #####################
        transitionMatrix()
        processNoiseCovarianceMatrix()
        estQua()
        estCovarianceMatrix()
    
    timeStep += 1
    #time += 10

ACCEL_set_4g()
utime.sleep(1)
GYRO_set_250dps()
utime.sleep(1)
timer.init(freq=100, mode=Timer.PERIODIC, callback=AHRS)
ACCEL_cs.value(1)
GYRO_cs.value(1)