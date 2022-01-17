from machine import Timer
import utime
import ustruct
import sys
import uctypes
from math import *


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

#####ACCEL SET LOW PASS F�LTER#############

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

#######GYRO SET LOW PASS F�LTER#############

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