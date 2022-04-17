import sys
from machine import Pin,I2C,Timer
import uctypes
import ustruct
import utime

i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=100000)

MMC5983MA_XOUT_0        = 0x00
MMC5983MA_XOUT_1        = 0x01
MMC5983MA_YOUT_0        = 0x02
MMC5983MA_YOUT_1        = 0x03
MMC5983MA_ZOUT_0        = 0x04
MMC5983MA_ZOUT_1        = 0x05
MMC5983MA_XYZOUT_2      = 0x06
MMC5983MA_TOUT          = 0x07
MMC5983MA_STATUS        = 0x08
MMC5983MA_CONTROL_0     = 0x09
MMC5983MA_CONTROL_1     = 0x0A
MMC5983MA_CONTROL_2     = 0x0B
MMC5983MA_CONTROL_3     = 0x0C
MMC5983MA_PRODUCT_ID    = 0x2F

MMC5983MA_ADDRESS       = 0x30

# Sample rates
MODR_ONESHOT =  0x00
MODR_1Hz     =  0x01
MODR_10Hz    =  0x02
MODR_20Hz    =  0x03
MODR_50Hz    =  0x04
MODR_100Hz   =  0x05
MODR_200Hz   =  0x06 # BW = 0x01 only
MODR_1000Hz  =  0x07 # BW = 0x11 only

#Bandwidths
MBW_100Hz = 0x00  # 8 ms measurement time
MBW_200Hz = 0x01  # 4 ms
MBW_400Hz = 0x02  # 2 ms
MBW_800Hz = 0x03  # 0.5 ms


# Set/Reset as a function of measurements
MSET_1     = 0x00 # Set/Reset each data measurement
MSET_25    = 0x01 # each 25 data measurements
MSET_75    = 0x02
MSET_100   = 0x03
MSET_250   = 0x04
MSET_500   = 0x05
MSET_1000  = 0x06
MSET_2000  = 0x07

def I2C_reg_write(i2c, addr, reg, data):
    """
    Write bytes to the specified register.
    """
    
    # Construct message
    msg = bytearray()
    msg.append(data)
    
    # Write out message to register
    i2c.writeto_mem(addr, reg, msg)



def I2Creg_read(i2c, addr, reg, nbytes=1):
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


timer = Timer()

def MAGNETO_data():
    
    XYZout2 = I2Creg_read(i2c,MMC5983MA_ADDRESS ,MMC5983MA_XYZOUT_2,1)
    
    Xout0 = I2Creg_read(i2c,MMC5983MA_ADDRESS ,MMC5983MA_XOUT_0,1)
    Xout1 = I2Creg_read(i2c,MMC5983MA_ADDRESS ,MMC5983MA_XOUT_1,1)
    Xout0 = int.from_bytes(Xout0,"little")<<8
    Xout1 = int.from_bytes(Xout1,"little")
    Xout1 = Xout0 | Xout1
    X_Magneto = (Xout1<<2) | (((XYZout2[0] & 0xC0) >> 6 ) & 0x3)
    
    Yout0 = I2Creg_read(i2c,MMC5983MA_ADDRESS ,MMC5983MA_YOUT_0,1)
    Yout1 = I2Creg_read(i2c,MMC5983MA_ADDRESS ,MMC5983MA_YOUT_1,1)
    Yout0 = int.from_bytes(Yout0,"little")<<8
    Yout1 = int.from_bytes(Yout1,"little")
    Yout1 = Yout0 | Yout1
    Y_Magneto = (Yout1<<2) | (((XYZout2[0] & 0x30) >> 4 ) & 0x3)
    
    Zout0 = I2Creg_read(i2c,MMC5983MA_ADDRESS ,MMC5983MA_ZOUT_0,1)
    Zout1 = I2Creg_read(i2c,MMC5983MA_ADDRESS ,MMC5983MA_ZOUT_1,1)
    Zout0 = int.from_bytes(Zout0,"little")<<8
    Zout1 = int.from_bytes(Zout1,"little")
    Zout1 = Zout0 | Zout1
    Z_Magneto = (Zout1<<2) | (((XYZout2[0] & 0x03) >> 2 ) & 0x3)
    
    
    print(((X_Magneto-131072)/16384)*100,((Y_Magneto-131072)/16384)*100,((Z_Magneto-131072)/16384)*100)


#reset
I2C_reg_write(i2c,MMC5983MA_ADDRESS ,MMC5983MA_CONTROL_0, 0x10)

utime.sleep(1)

#set
I2C_reg_write(i2c,MMC5983MA_ADDRESS ,MMC5983MA_CONTROL_0, 0x08)

utime.sleep(1)

#init
I2C_reg_write(i2c,MMC5983MA_ADDRESS, MMC5983MA_CONTROL_0, 0x20 | 0x04)

utime.sleep(1)

I2C_reg_write(i2c,MMC5983MA_ADDRESS, MMC5983MA_CONTROL_1, MBW_100Hz)

utime.sleep(1)

I2C_reg_write(i2c,MMC5983MA_ADDRESS, MMC5983MA_CONTROL_2, 0x08 | MODR_100Hz)

utime.sleep(1)
######




while True:
    
    temp = I2Creg_read(i2c,MMC5983MA_ADDRESS, MMC5983MA_STATUS)
    I2C_reg_write(i2c,MMC5983MA_ADDRESS ,MMC5983MA_STATUS,(temp[0] & 0x01))
    
    MAGNETO_data()
    
    utime.sleep(0.01)
    
  





