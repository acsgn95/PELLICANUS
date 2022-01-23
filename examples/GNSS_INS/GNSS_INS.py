from machine import Timer,UART,Pin
import machine
import utime
import ustruct
import sys
import uctypes
from ulab import numpy as np
from math import *
##############################
uart1 = UART(0,baudrate = 115200, tx = Pin(12),rx = Pin(13))

##### REGISTER MAP #####
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG2 = 0x1D
GYRO_CONFIG = 0x1B
GYRO_CONFIG2 = 0x1A

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

####ACCEL SET MAX RANGE########################

def ACCEL_set_2g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x00)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)


def ACCEL_set_4g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x01)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)


def ACCEL_set_8g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x10)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)


def ACCEL_set_16g():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG,0x11)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG,1)

#####ACCEL SET LOW PASS FÝLTER#############

def ACCEL_set_5hz():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG2,0x06)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)

def ACCEL_set_10hz():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG2,0x05)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)

def ACCEL_set_20hz():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG2,0x04)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)

def ACCEL_set_40hz():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG2,0x03)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)

def ACCEL_set_100hz():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG2,0x02)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)

def ACCEL_set_200hz():
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)
    reg_write(spi,ACCEL_cs,ACCEL_CONFIG2,0x01)
    reg = reg_read(spi,ACCEL_cs,ACCEL_CONFIG2,1)


######GYRO SET MAX RANGE###########

def GYRO_set_250dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x00)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    
def GYRO_set_500dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x01)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    
def GYRO_set_1000dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x10)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)


def GYRO_set_2000dps():
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)
    reg_write(spi,GYRO_cs,GYRO_CONFIG,0x11)
    reg = reg_read(spi,GYRO_cs,GYRO_CONFIG,1)

#######GYRO SET LOW PASS FÝLTER#############

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
    
    return wx,wy,wz

################################ MAHNETOMETER #############################

LIS3MDL_M_ADDRESS       = 0x1E
LIS3MDL_WHO_AM_I_M      = 0x0F
LIS3MDL_CTRL_REG1_M     = 0x20
LIS3MDL_CTRL_REG2_M     = 0x21
LIS3MDL_CTRL_REG3_M     = 0x22
LIS3MDL_CTRL_REG4_M     = 0x23
LIS3MDL_STATUS_REG_M    = 0x27
LIS3MDL_OUT_X_L_M       = 0x28
LIS3MDL_OUT_X_H_M       = 0x29
LIS3MDL_OUT_Y_L_M       = 0x2A
LIS3MDL_OUT_Y_H_M       = 0x2B
LIS3MDL_OUT_Z_L_M       = 0x2C
LIS3MDL_OUT_Z_H_M       = 0x2D
LIS3MDL_TEMP_OUT_L_M        = 0x2E
LIS3MDL_TEMP_OUT_H_M        = 0x2F
LIS3MDL_INT_CFG_M       = 0x30
LIS3MDL_INT_SRC_M       = 0x31
LIS3MDL_INT_THS_L_M     = 0x32
LIS3MDL_INT_THS_H_M     = 0x33
 
LIS3MDL_REG_CTL_1_TEMP_EN   = 0x80
LIS3MDL_REG_CTL_2_RESET     = 0x04
 
 
# mag_scale defines all possible FSR's of the magnetometer:
 
LIS3MDL_M_SCALE_4GS         = 0x20 # 00:  4Gs
LIS3MDL_M_SCALE_8GS     = 0x40 # 01:  8Gs
LIS3MDL_M_SCALE_12GS        = 0x60 # 10:  12Gs
LIS3MDL_M_SCALE_16GS        = 0x60 # 11:  16Gs

# mag_oder defines all possible output data rates of the magnetometer:
 
LIS3MDL_M_ODR_625       = 0x04 # 6.25 Hz 
LIS3MDL_M_ODR_125       = 0x08 # 12.5 Hz 
LIS3MDL_M_ODR_25        = 0x0C # 25 Hz 
LIS3MDL_M_ODR_5         = 0x10 # 50 
LIS3MDL_M_ODR_10        = 0x14 # 10 Hz
LIS3MDL_M_ODR_20        = 0x14 # 20 Hz
LIS3MDL_M_ODR_40        = 0x14 # 40 Hz
LIS3MDL_M_ODR_80        = 0x14 # 80 Hz

mRes = 4.0 / 32768.0   # 4G
SENSITIVITY_OF_MIN_SCALE = 27368.0 # (4 guass scale) * (6842 LSB/guass at 4 guass scale)
Scale = 16

i2c = machine.I2C(0,
                  scl=machine.Pin(13),
                  sda=machine.Pin(12),
                  freq=400000)

###############################################################################
# Functions

def reg_writei2c(i2c, addr, reg, data):

    """
    Write bytes to the specified register.
    """
    
    # Construct message
    msg = bytearray()
    msg.append(data)
    
    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)
    
def reg_readi2c(i2c, addr, reg, nbytes=1):

    """
    Read byte(s) from specified register. If nbytes > 1, read from consecutive
    registers.
    """
    
    # Check to make sure caller is asking for 1 or more bytes
    if nbytes < 1:
        return bytearray()
    
    # Request data from specified register(s) over I2C
    data = i2c.readfrom_mem(addr, reg, nbytes)
    
    return data

def _twos_comp(val, bits=16):
    """compute the 2's complement of int val with bits"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set
        val = val - (1 << bits)          # compute negative value
    return val 

###############################################################################

def initialise():
 
# initMag -- Sets up the magnetometer to begin reading.
 
    #User Register Reset Function
    reg_writei2c(i2c,LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG2_M, LIS3MDL_REG_CTL_2_RESET)
    #Temperature Sensor Enabled
    reg_writei2c(i2c,LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG1_M, LIS3MDL_REG_CTL_1_TEMP_EN )
    #Ultra High Performance Mode Selected for XY Axis
    reg_writei2c(i2c,LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG1_M, 0x60)
    #Ultra High Performance Mode Selected for Z Axis
    reg_writei2c(i2c,LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG4_M, 0x0C)
    #Output Data Rate of 80 Hz Selected
    reg_writei2c(i2c,LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG1_M, 0x1C)
    #Continous Conversion Mode,4 wire interface Selected 
    reg_writei2c(i2c,LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG3_M, 0x00)
    # 16 guass Full Scale 
    reg_writei2c(i2c,LIS3MDL_M_ADDRESS, LIS3MDL_CTRL_REG2_M, 0x60)
 
 
#Read the magnetometer output registers.
# This will read all six Magnetometer output registers.
 
# Reading the  Magnetometer X-Axis Values from the Register

def readMagx():
        Mag_l = reg_readi2c(i2c,LIS3MDL_M_ADDRESS,LIS3MDL_OUT_X_L_M)
        Mag_h = reg_readi2c(i2c,LIS3MDL_M_ADDRESS,LIS3MDL_OUT_X_H_M)
        Mag_total = (Mag_l[0] | Mag_h[0] <<8)
        
        return Mag_total  if Mag_total < 32768 else Mag_total - 65536
            
# Reading the  Magnetometer Y-Axis Values from the Register

def readMagy():
        Mag_l = reg_readi2c(i2c,LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Y_L_M)
        Mag_h = reg_readi2c(i2c,LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Y_H_M)
        Mag_total = (Mag_l[0] | Mag_h[0] <<8)
        return Mag_total  if Mag_total < 32768 else Mag_total - 65536
    
# Reading the  Magnetometer Z-Axis Values from the Register

def readMagz():
        Mag_l = reg_readi2c(i2c,LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Z_L_M)
        Mag_h = reg_readi2c(i2c,LIS3MDL_M_ADDRESS,LIS3MDL_OUT_Z_H_M)
        Mag_total = (Mag_l[0] | Mag_h[0] <<8)
        return Mag_total  if Mag_total < 32768 else Mag_total - 65536



################ GNSS ##########################################

gps_module = UART(0,baudrate = 9600, tx = Pin(28), rx = Pin(29))

#Used to Store NMEA Sentences

buff = bytearray(255)
latitude = None
longitude = None
height = None
groundspeed = None
gpsyaw = None
geoidalseperation = None
hemisLon = None
hemisLat = None
ggaFix = False
vtgFix = False
vVFix = False

def gnss(timer2):
    
    global latitude,longitude,height,groundspeed,gpsyaw,geoidalseperation,hemisLon,hemisLat,ggaFix,vtgFix
    
    latitude = None
    longitude = None
    height = None
    groundspeed = None
    gpsyaw = None
    geoidalseperation = None
    hemisLon = None
    hemisLat = None
    ggaFix = False
    vtgFix = False
    

    gps_module.readline()
    
    buff = str(gps_module.readline())
    
    data = buff.split(",")
    
    if data[0] == "b'$GNGGA" and len(data) == 15:

        if len(data[2]) != 0 and len(data[4]) != 0 and len(data[9]) != 0 and len(data[11]) != 0:
            latitude = data[2]
            latdeg = latitude[0:2]
            latdeg1 = latitude[2:]
            latdeg1 = float(latdeg1)/60
            latitude = int(latdeg) + latdeg1
            longitude = data[4]
            londeg = longitude[0:3]
            londeg1 = longitude[3:]
            londeg1 = float(londeg1)/60
            longitude = int(londeg) + londeg1

            height = float(data[9])
            #print(latdeg,latdeg1,londeg,londeg1)
            hemisLat = data[3]
            hemisLon = data[5]
            geoidalseperation = float(data[11])
            ggaFix = True

    if data[0] == "b'$GNVTG" and len(data) == 10:
        if len(data[7]) != 0 and len(data[1]) != 0:
            groundspeed = float(data[7])
            gpsyaw = float(data[1])
            vtgFix = True
    
    #print(latitude,hemisLat,longitude,hemisLon,height,geoidalseperation,groundspeed,gpsyaw)
    #print(data)

################## TIME STEP ###################

t = 0
deltaT = 0.02

#####################################

axb = 0
ayb = 0
azb = 0
ax = 0
ay = 0
az = 0
wx = 0
wy = 0
wz = 0
Magx = 0
Magy = 0
Magz = 0
step1magx = 0
step1magy = 0
step1magz = 0
bx = 0
by = 0
cmagx = 0
cmagy = 0
cmagz = 0

def IMU_ecompensation():

    global axb,ayb,azb,ax,ay,az,wx,wy,wz,Magx,Magy,Magz,step1magx,step1magy,step1magz,cmagx,cmagy,cmagz

    axb,ayb,azb = ACCEL_data()
    axb=axb/16384
    ayb=ayb/16384
    azb=azb/16384
    axb = axb + 0.008859
    ayb = ayb - 0.007039
    azb = azb - 0.005115

    ax = 0.998103*axb - 0.000179*ayb - 0.000331*azb
    ay = -0.000179*axb + 1.001615*ayb + 0.000106*azb
    az = -0.000331*axb + 0.000106*ayb + 0.998466*azb

    #NED conflict
    ay = -ay
    az = -az

    #GYRO
    wx,wy,wz = GYRO_data()
    wx = radians((wx/32768)*500)
    wy = radians((wy/32768)*500)
    wz = radians((-wz/32768)*500)

    #NED conflict
    wy = -wy

    # Mag Read
    Magx = ((readMagx())*(Scale/SENSITIVITY_OF_MIN_SCALE))*100
    Magy = ((readMagy())*(Scale/SENSITIVITY_OF_MIN_SCALE))*100
    Magz = ((readMagz())*(Scale/SENSITIVITY_OF_MIN_SCALE))*100

    #NED conflict
    Magz = -Magz
    Magy = -Magy

    step1magx = (Magx - (-55.256433))
    step1magy = (Magy - (-4.367858))
    step1magz = (Magz - (9.826686))

    cmagx = step1magx*1.301028 + step1magy*-0.046316 + step1magz*0.014783
    cmagy = step1magx*-0.046316 + step1magy*1.282879 + step1magz*-0.033587
    cmagz = step1magx*0.014783 + step1magy*-0.033587 + step1magz*1.348008

################## AHRS ###################

gyro_quaternion = np.array([[0],
                            [0],
                            [0],
                            [0]],dtype = np.float)

ma_quaternion = np.array([[0],
                          [0],
                          [0],
                          [0]],dtype = np.float)


omega = np.array([[0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0]],dtype=np.float)
pitch = 0
roll = 0
yaw = 0
pitch_rd = 0
roll_rd = 0
yaw_rd = 0
bx = 0
by = 0
qw = 0
qx = 0
qy = 0
qz = 0
dcm = np.array([[0,0,0],
                [0,0,0],
                [0,0,0]],dtype = np.float)

alfa = 0.1



def euler2qua():
    global pitch_rd,roll_rd,yaw_rd,qw,qx,qy,qz

    p = pitch_rd/2
    r = roll_rd/2
    y = yaw_rd/2

    qw = cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y)
    qx = sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y)
    qy = cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y)
    qz = cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y)



def qua2euler():

    global pitch_rd,roll_rd,yaw_rd,qw,qx,qy,qz

    roll_rd = atan2((2*(qw*qx + qy*qz)),(1-2*qx*qx - 2*qy*qy))
    pitch_rd = asin(2*(qw*qy - qx*qz))
    yaw_rd = atan2((2*(qw*qz + qx*qy)),(1- 2*qy*qy - 2*qz*qz))



def AHRS():

    global t,pitch,pitch_rd,roll,roll_rd,yaw,yaw_rd,ax,ay,az,wx,wy,wz,dcm,bx,by,cmagx,cmagy,cmagz,qw,qx,qy,qz,deltaT,alfa,gyro_quaternion,ma_quaternion,omega
    
    pitch_rd = atan(ax/((ay*ay + az*az)**0.5))
    #pitch_rd = -asin(-ax)
    roll_rd = atan2(-ay,-az)
    bx = cmagx*cos(pitch_rd)+cmagy*sin(pitch_rd)*sin(roll_rd)+cmagz*sin(pitch_rd)*cos(roll_rd)
    by = cmagy*cos(roll_rd) - cmagz*sin(roll_rd)
    yaw_rd = atan2(-by,bx) + 0.09843
    #print(degrees(pitch_rd),degrees(roll_rd),degrees(yaw_rd))
    d = deltaT/2

    if t<1:

        euler2qua()

    else:

        gyro_quaternion = np.array([[qw-d*wx*qx-d*wy*qy-d*wz*qz],
                                    [qx+d*wx*qw-d*wy*qz+d*wz*qy],
                                    [qy+d*wx*qz+d*wy*qw-d*wz*qx],
                                    [qz-d*wx*qy+d*wy*qx+d*wz*qw]],dtype=np.float)*(1-alfa)


        euler2qua()
        ma_quaternion = np.array([[qw],
                                  [qx],
                                  [qy],
                                  [qz]],dtype=np.float)*(alfa)

        quat = gyro_quaternion + ma_quaternion

        qw = quat[0][0]
        qx = quat[1][0]
        qy = quat[2][0]
        qz = quat[3][0]

        qnorm = qw*qw + qx*qx + qy*qy + qz*qz

        qw = qw/qnorm
        qx = qx/qnorm
        qy = qy/qnorm
        qz = qz/qnorm
        qua2euler()

        roll = degrees(roll_rd)
        pitch = degrees(pitch_rd)
        yaw = degrees(yaw_rd)

        dcm = np.array([[(qw*qw + qx*qx - qy*qy -qz*qz),(2*(qx*qy - qz*qw)),(2*(qx*qz + qy*qw))],
                        [(2*(qx*qy + qz*qw)),(qw*qw - qx*qx + qy*qy -qz*qz),(2*(qy*qz - qx*qw))],
                        [(2*(qx*qz - qy*qw)),(2*(qy*qz + qx*qw)),(qw*qw - qx*qx -qy*qy + qz*qz)]],dtype=np.float)



################### EKF ##################
gravity = np.array([[0],
                    [0],
                    [-1]],dtype = np.int8)

estState = np.array([[0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0]],dtype = np.float)

corState = np.array([[0],
                     [0],
                     [0],
                     [0],
                     [0],
                     [0]],dtype = np.float)

estP = np.eye(6,dtype = np.float)*50

corrP = np.eye(6,dtype = np.float)

F = np.array([[1,0,0,0,0,0],
              [0,1,0,0,0,0],
              [0,0,1,0,0,0],
              [0,0,0,1,0,0],
              [0,0,0,0,1,0],
              [0,0,0,0,0,1]],dtype = np.float)

Rgga = np.eye(3)*0.025
Rvtg = np.eye(2)*0.05
RvV = np.eye(1)*0.05
R = None
noise = np.array([[2.81e-13],
                  [2.81e-13],
                  [2.81e-13],
                  [7.03e-10],
                  [7.03e-10],
                  [7.03e-10]],dtype = np.float)

#Qt = np.dot(np.eye(6),noise)
Qt = np.eye(6)*0.1
Hgga = np.array([[1,0,0,0,0,0],
                 [0,1,0,0,0,0],
                 [0,0,1,0,0,0]],dtype = np.int8)

Hvtg = np.array([[0,0,0,1,0,0],
                 [0,0,0,0,1,0]],dtype = np.int8)

HvV = np.array([[0,0,0,0,0,1]],dtype = np.int8)

H = None

kG = np.eye(6,dtype=np.float)

Z = np.array([[0],
              [0],
              [0]],dtype = np.float)

Vt = np.array([[0],
               [0],
               [0]],dtype = np.float)

accnoise = 86e-4

posnoiseconst = accnoise*deltaT*deltaT
velnoiseconst = accnoise*deltaT

NEDpos = np.array([[0],
                   [0],
                   [0]],dtype = np.float)

NEDvel = np.array([[0],
                   [0],
                   [0]],dtype = np.float)

a = np.array([[0],
              [0],
              [0]],dtype = np.float)

mNEDX = 0
mNEDY = 0
mNEDZ = 0
mNVELX = 0
mNVELY = 0
vVelocity = 0
estimating = np.array([[0],
                       [0],
                       [0]],dtype = np.float)

def estimatingState():

    global ax,ay,az,a,dcm,deltaT,estState,corState,posnoiseconst,velnoiseconst,NEDpos,NEDvel

    a = np.array([[ax],
                  [ay],
                  [az]],dtype = np.float)
    
    pnoise = np.array([[posnoiseconst],
                       [posnoiseconst],
                       [posnoiseconst]],dtype = np.float)
    vnoise = np.array([[velnoiseconst],
                       [velnoiseconst],
                       [velnoiseconst]],dtype = np.float)
    
    pnoise = - np.dot(dcm,pnoise)
    vnoise = - np.dot(dcm,vnoise)

    NEDpos = np.array([[corState[0][0]],
                       [corState[1][0]],
                       [corState[2][0]]],dtype = np.float)

    NEDvel = np.array([[corState[3][0]],
                       [corState[4][0]],
                       [corState[5][0]]],dtype = np.float)
    
    NEDpos = NEDpos + NEDvel*deltaT #+ pnoise
    
    step1 = np.dot(dcm,a)
    step2 = (step1-gravity)*9.81*deltaT

    NEDvel = NEDvel + step2 #+ vnoise

    estState = np.array([[NEDpos[0][0]],
                         [NEDpos[1][0]],
                         [NEDpos[2][0]],
                         [NEDvel[0][0]],
                         [NEDvel[1][0]],
                         [NEDvel[2][0]]],dtype = np.float)

def estimatingCovarianceMatrix():

    global corrP,F,Qt,estP

    Ftr = F.transpose()

    step1 = np.dot(F,corrP)
    step2 = np.dot(step1,Ftr)
    estP = step2 + Qt

def kalmangain():

    global H,estP,R,kG

    Htr = H.transpose()

    step1 = np.dot(H,estP)
    step2 = np.dot(step1,Htr)
    step3 = step2 + R
    step4 = np.linalg.inv(step3)
    step5 = np.dot(estP,Htr)
    kG = np.dot(step5,step4)
    
def correctionState():
    
    global estState,kG,Vt,corState

    step1 = np.dot(kG,Vt)
    corState = estState - step1

def correctionP():

    global estP,kG,H,corrP

    step1 = np.dot(kG,H)
    step2 = np.dot(step1,estP)
    corrP = estP - step2


def EKF():

    global t,ax,ay,az,dcm,gravity,estState,corState,estP,corrP,F,R,Rgga,Rvtg,Qt,H,Hgga,Hvtg,kG,Z,Vt,ggaFix,vtgFix,mNEDX,mNEDY,mNEDZ,mNVELX,mNVELY,estimating,vVFix,HvV,RvV,vVelocity

    if ggaFix:
        
        R = Rgga
        H = Hgga
        Z = np.array([[mNEDX],
                      [mNEDY],
                      [mNEDZ]],dtype = np.float)
        estimating = np.array([[estState[0][0]],
                               [estState[1][0]],
                               [estState[2][0]]],dtype = np.float)
        
        Vt = estimating - Z
        kalmangain()
        correctionState()
        correctionP()

        ggaFix = False
        vVFix = True

    elif vtgFix:

        R = Rvtg
        H = Hvtg
        Z = np.array([[mNVELX],
                      [mNVELY]],dtype = np.float)
        estimating = np.array([[estState[3][0]],
                               [estState[4][0]]],dtype = np.float)
        
        Vt = estimating - Z
        kalmangain()
        correctionState()
        correctionP()

        vtgFix = False

    elif vVFix and ggaFix == False:
        R = RvV
        H = HvV
        Z = np.array([[vVelocity]],dtype = np.float)
        estimating = np.array([estState[5][0]],dtype = np.float)
        Vt = estimating-Z
        kalmangain()
        correctionState()
        correctionP()

        vVFix = False

    else:

        corState = estState
        corrP = estP


    estimatingState()
    estimatingCovarianceMatrix()

################# COORDINATE TRANSFORMATION ####################
#WGS 84 ellipsoid model
Rea = 6378137 #The semi major axis (meters)
Reb = 6356752 #The semi-minor axis (meters)
e = 0.08181919 #The first eccentricity
ee = e*e
en = 1-ee
eSqrt = 0.99330562
Rea_eSqrt = Rea*eSqrt
Me = None #Meridian Radius
Ne = None #Vertical Radius
#############################
ECEF = np.array([[0],
                 [0],
                 [0]],dtype = np.float)
Xecef = None
Yecef = None
Zecef = None
referanceNED = False
referanceLat = None
referanceLon = None
referanceECEF = np.array([[0],
                          [0],
                          [0]],dtype = np.float)
referanceECEFX = None
referanceECEFY = None
referanceECEFZ = None
NED = np.array([[0],
                [0],
                [0]],dtype = np.float)
Xned = None
Yned = None
Zned = None
Xvel = None
Yvel = None
Zvel = None
ecef_ned = np.array([[0,0,0],
                     [0,0,0],
                     [0,0,0]],dtype = np.float)
ned_ecef = np.array([[0,0,0],
                     [0,0,0],
                     [0,0,0]],dtype = np.float)

clongitude = 0
clatitude = 0
cheight = 0
h0 = 0
h1 = 0
pTime = 0
cTime = 0
def coordinateTransformation(timer):

    global t,ax,ay,az,ggaFix,vtgFix,corState,pitch,roll,yaw,clongitude,clatitude,cheight,mNEDX,mNEDY,mNEDZ,mNVELX,mNVELY,latitude,longitude,height,geoidalseperation,hemisLon,hemisLat,groundspeed,gpsyaw,Rea,Reb,ee,en,eSqrt,Rea_eSqrt,Me,Ne,ECEF,Xecef,Yecef,Zecef,NED,Xned,Yned,Zned,referanceNED,referanceLat,referanceLon,referanceECEF,referanceECEFX,referanceECEFY,referanceECEFZ,ned_ecef,ecef_ned,Xvel,Yvel,Zvel,h0,h1,pTime,cTime,vVelocity
    gnssFix = "No GNSS!!!!"
    isStart = "No Started"
    correct = "No Correct"
    IMU_ecompensation()
    AHRS()
    if t >= 50:

        if ggaFix:
            if hemisLat == "S":
                latitude = -latitude

            if hemisLon == "W":
                longitude = -longitude
         
            lat = radians(latitude)
            lon = radians(longitude)

            step1 = 1- ee*(sin(lat)**2)

            Me = Rea_eSqrt/(step1**1.5)
            Ne = Rea/(step1**0.5)

            Xecef = (Ne+height)*cos(lat)*cos(lon)
            Yecef = (Ne+height)*cos(lat)*sin(lon)
            Zecef = (Ne*eSqrt+height)*sin(lat)

            ECEF = np.array([[Xecef],
                             [Yecef],
                             [Zecef]],dtype = np.float)

            if referanceNED == False:

                referanceLat = lat
                referanceLon = lon
                referanceECEFX = Xecef
                referanceECEFY = Yecef
                referanceECEFZ = Zecef

                ecef_ned = np.array([[(-sin(lat)*cos(lon)),(-sin(lat)*sin(lon)),(cos(lat))],
                                     [(-sin(lon)),(cos(lon)),0],
                                     [(-cos(lat)*cos(lon)),(-cos(lat)*sin(lon)),(-sin(lat))]],dtype = np.float)

                ned_ecef = ecef_ned.transpose()

                referanceECEF = np.array([[referanceECEFX],
                                          [referanceECEFY],
                                          [referanceECEFZ]],dtype = np.float)
        
                referanceNED = True
            step2 = ECEF - referanceECEF
            NED = np.dot(ecef_ned,step2)
            mNEDX = NED[0][0]
            mNEDY = NED[1][0]
            mNEDZ = NED[2][0]
            gnssFix = "GNSS FIX!!!"
            correct = "Position!!"
            
            h1 = mNEDZ
            cTime = t

            if pTime == 0:
                h1 = h0
                pTime = cTime

            elif pTime != 0:
                
                epoch = (cTime-pTime)*0.02
                vVelocity = -(h1-h0)/epoch
                h1 = h0
                pTime = cTime

        elif vtgFix:
            
            gpsy = radians(gpsyaw)

            groundspeedmetersecond = (groundspeed*1000)/3600
            
            mNVELX = cos(gpsy)*groundspeedmetersecond
            mNVELY = sin(gpsy)*groundspeedmetersecond
            gnssFix = "GNSS FIX!!!"
            correct = "Velocity!!"

        if referanceNED:

            EKF()
        
            Xned = corState[0][0]
            Yned = corState[1][0]
            Zned = corState[2][0]
            Xvel = corState[3][0]
            Yvel = corState[4][0]
            Zvel = corState[5][0]
            corNED = np.array([[Xned],
                               [Yned],
                               [Zned]],dtype = np.float)

            corECEF = np.dot(ned_ecef,corNED) + ECEF
            #### Heikkinen Closed-Form Exact Solution == ECEF --> Geodetic Position#########
            
            Xe = corECEF[0][0]
            Ye = corECEF[1][0]
            Ze = corECEF[2][0]
            
            beta = (Xe*Xe + Ye*Ye)**0.5

            clong = atan2(Ye,Xe)
            clongitude = degrees(clong)

            Fh = 54*eSqrt*Rea*Rea*Ze*Ze
            Gh = beta*beta + eSqrt*Ze*Ze - ee*ee*Rea*Rea
            Ch = (ee*ee*Fh*beta*beta)/(Gh**3)
            Sh = (1+Ch+((Ch*Ch + 2*Ch)**0.5))**(1/3)
            Ph = Fh/(3*((Sh+(1/Sh)+1)**2)*Gh*Gh)
            Qh = (1+ 2*ee*ee*Ph)**0.5
            sh1 = (Rea*Rea/2)*(1 + 1/Qh)
            sh2 = (Ph*eSqrt*Ze*Ze)/(Qh*(1+Qh))
            sh3 = (Ph*beta*beta)/2
            sh4 = (Ph*ee*beta)/(1+Qh)

            Th = ((sh1-sh2-sh3)**0.5)-sh4
            Vh = (((beta-ee*Th)**2)+(eSqrt*Ze*Ze))**0.5
            Mh = (1+((ee*Rea)/Vh))*(Ze/beta)
            clat = atan(Mh)
            clatitude = degrees(clat)

            cheight = (1-((eSqrt*Rea)/Vh))*(((beta - ee*Th)**2+ Ze**2)**0.5)

            isStart = "Started"

    result = "{},{},{},{},{},{},{},{},{},{},{},{}".format(isStart,gnssFix,correct,clatitude,clongitude,cheight,pitch,roll,yaw,Xvel,Yvel,Zvel)
    print(result)

    t += 1
    

timer = Timer()
timer2 = Timer()




initialise()
utime.sleep(1)
ACCEL_set_2g()
utime.sleep(1)
GYRO_set_250dps()
utime.sleep(4)

reset = Pin(7,Pin.OUT)

reset.value(1)
utime.sleep(1)
gps_module.write("$PMTK101")
utime.sleep(1)

timer2.init(freq = 4, mode = Timer.PERIODIC, callback= gnss)
timer.init(freq = 50, mode = Timer.PERIODIC, callback= coordinateTransformation)

ACCEL_cs.value(1)
GYRO_cs.value(1)