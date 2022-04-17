import sys
from machine import Pin,I2C,Timer,SPI,UART
import uctypes
import ustruct
import utime

##### REGISTER MAP #####
##### imu ####

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

#### magnetometer ####

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

#########################################

#### PIN CONFIGURATION ####

#### imu conf ####
IMU_cs = Pin(17, machine.Pin.OUT)

# Initialize SPI

spi = SPI(0,
          baudrate=1000000,
          polarity=1,
          phase=1,
          bits=8,
          firstbit=SPI.MSB,
          sck=Pin(18),
          mosi=Pin(19),
          miso=Pin(20))

##################

#### mag conf ####

i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=100000)

#### gnss conf ####

gps_module = UART(0,baudrate = 9600, tx = Pin(28), rx = Pin(29))

###########################################

#### SPI FUNCTIONS ####

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

###########################################

#### I2C FUNCTIONS ####

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

###########################################

########### IMU FUNCTIONS #################

####ACCEL SET MAX RANGE########################

def ACCEL_set_2g():
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)
    reg_write(spi,IMU_cs,ACCEL_CONFIG,0x00)
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)


def ACCEL_set_4g():
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)
    reg_write(spi,IMU_cs,ACCEL_CONFIG,0x08)
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)


def ACCEL_set_8g():
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)
    reg_write(spi,IMU_cs,ACCEL_CONFIG,0x10)
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)


def ACCEL_set_16g():
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)
    reg_write(spi,IMU_cs,ACCEL_CONFIG,0x18)
    reg = reg_read(spi,IMU_cs,ACCEL_CONFIG,1)


######GYRO SET MAX RANGE###########

def GYRO_set_250dps():
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)
    reg_write(spi,IMU_cs,GYRO_CONFIG,0x00)
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)
    
def GYRO_set_500dps():
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)
    reg_write(spi,IMU_cs,GYRO_CONFIG,0x08)
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)
    
def GYRO_set_1000dps():
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)
    reg_write(spi,IMU_cs,GYRO_CONFIG,0x10)
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)

def GYRO_set_2000dps():
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)
    reg_write(spi,IMU_cs,GYRO_CONFIG,0x18)
    reg = reg_read(spi,IMU_cs,GYRO_CONFIG,1)



def ACCEL_data():
    
    #################### X axis ############################
    dataX = reg_read(spi, IMU_cs, ACCEL_XOUT_L, 1)
    dataX = dataX + reg_read(spi, IMU_cs, ACCEL_XOUT_H, 1)
    ax = int.from_bytes(dataX,'little')
    ax = _twos_comp(ax)
    
    #################### Y axis ############################
    dataY = reg_read(spi, IMU_cs, ACCEL_YOUT_L, 1)
    dataY = dataY + reg_read(spi, IMU_cs, ACCEL_YOUT_H, 1)
    ay = int.from_bytes(dataY,'little')
    ay = _twos_comp(ay)   
    
    #################### Z axis ############################
    dataZ = reg_read(spi, IMU_cs, ACCEL_ZOUT_L, 1)
    dataZ = dataZ + reg_read(spi, IMU_cs, ACCEL_ZOUT_H, 1)
    az = int.from_bytes(dataZ,'little')
    az = _twos_comp(az)
    
    return ax,ay,az


def GYRO_data():
    
    #################### X axis ############################
    dataX = reg_read(spi, IMU_cs, GYRO_XOUT_L, 1)
    dataX = dataX + reg_read(spi, IMU_cs, GYRO_XOUT_H, 1)
    wx = int.from_bytes(dataX,'little')
    wx = _twos_comp(wx)
    
    #################### Y axis ############################
    dataY = reg_read(spi, IMU_cs, GYRO_YOUT_L, 1)
    dataY = dataY + reg_read(spi, IMU_cs, GYRO_YOUT_H, 1)
    wy = int.from_bytes(dataY,'little')
    wy = _twos_comp(wy)
    
    #################### Z axis ############################
    dataZ = reg_read(spi, IMU_cs, GYRO_ZOUT_L, 1)
    dataZ = dataZ + reg_read(spi, IMU_cs, GYRO_ZOUT_H, 1)
    wz = int.from_bytes(dataZ,'little')
    wz = _twos_comp(wz)
    
    return wx,wy,wz

###########################################

########### MAG FUNCTIONS #################

def MAGNETO_init():
    
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
    
    
    return ((X_Magneto-131072)/16384)*100, ((Y_Magneto-131072)/16384)*100, ((Z_Magneto-131072)/16384)*100


def MAGNETO_clear():
    
    temp = I2Creg_read(i2c,MMC5983MA_ADDRESS, MMC5983MA_STATUS)
    I2C_reg_write(i2c,MMC5983MA_ADDRESS ,MMC5983MA_STATUS,(temp[0] & 0x01))
    
###########################################

########### GNSS FUNCTIONS #################

gnssTimer = Timer()
isGnssData = False
latitude = None
longitude = None
height = None
groundspeed = None
gpsyaw = None
geoidalseperation = None
hemisLon = None
hemisLat = None

#Used to Store NMEA Sentences
buff = bytearray(255)


def gps(gnssTimer):
    
    global gpsdata,isGnssData,latitude,longitude,height,groundspeed,gpsyaw,geoidalseperation,hemisLon,hemisLat
    
    latitude = None
    longitude = None
    height = None
    groundspeed = None
    gpsyaw = None
    geoidalseperation = None
    hemisLon = None
    hemisLat = None
    
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
    if data[0] == "b'$GNVTG" and len(data) == 10:
        if len(data[7]) != 0 and len(data[1]) != 0:
            groundspeed = float(data[7])
            gpsyaw = float(data[1])
    
    
    isGnssData = True
    

###########################################

########## LED #############

led1 = Pin(1,Pin.OUT)

###########################################

########## GENERAL TIMER FUNCTION #########
    
timer = Timer()
counter = 0
ledcounter = 0

def allData(timer):
    
    global isGnssData,counter,ledcounter,latitude,longitude,height,groundspeed,gpsyaw,geoidalseperation,hemisLon,hemisLat

    ##### LED WARNING ####
    if (ledcounter % 25 == 0) and (ledcounter % 50 != 0):
        led1.high()
        
    elif (ledcounter % 50 == 0):
        led1.low()
        
    #####################
        
    ax,ay,az = ACCEL_data()
    
    wx,wy,wz = GYRO_data()
    
    MAGNETO_clear()
    
    mx,my,mz = MAGNETO_data()
    
    #### NED CONFLICT and NORM #####
    ax = -ax/4096
    ay = -ay/4096
    az = -az/4096
    wx = -wx/65.5
    wy = wy/65.5
    wz = -wz/65.5
    my = -my
    
    print(ax,ay,az,wx,wy,wz,mx,my,mz,latitude,longitude,height,groundspeed,gpsyaw,geoidalseperation,hemisLon,hemisLat)
    
    if isGnssData == True:
        
        isGnssData = False
        latitude = None
        longitude = None
        height = None
        groundspeed = None
        gpsyaw = None
        geoidalseperation = None
        hemisLon = None
        hemisLat = None
    
    counter += 1
    ledcounter += 1
    

###########################################

######### INITIALIZATIONS ########

ACCEL_set_8g()
utime.sleep(1)
GYRO_set_500dps()
utime.sleep(1)

MAGNETO_init()

reset = Pin(7,Pin.OUT)

reset.value(1)
utime.sleep(1)
gps_module.write("$PMTK101")


####### TIMER BLOCK ##########

gnssTimer.init(freq = 4, mode = Timer.PERIODIC, callback= gps)
timer.init(freq = 100, mode = Timer.PERIODIC, callback=allData)

IMU_cs.value(1)