from micropython import const
import utime
import ustruct
import machine

# list of commands in hex for MS5637
c_reset = const(0x1E) # reset command
r_c1 = const(0xA2) # read PROM C1 command
r_c2 = const(0xA4) # read PROM C2 command
r_c3 = const(0xA6) # read PROM C3 command
r_c4 = const(0xA8) # read PROM C4 command
r_c5 = const(0xAA) # read PROM C5 command
r_c6 = const(0xAC) # read PROM C6 command
r_adc = const(0x00) # read ADC command
r_d1 = const(0x44) # convert D1 (OSR=1024)
r_d2 = const(0x54) # convert D2 (OSR=1024)


# set i2c clock to 100KHz
# TE MS5637 i2c adress = 0x76
i2c = machine.I2C(1, sda=machine.Pin(26), scl=machine.Pin(27), freq=100000)
slave_addr = 0x76  

# reset device to make sure PROM loaded
data = bytearray([c_reset])
i2c.writeto(slave_addr, data)

# scan i2c bus for active addresses
def scan_I2C():
    devices = i2c.scan()
    return devices


def read_c1(): #read PROM value C1
    data = bytearray([r_c1])
    i2c.writeto(slave_addr, data)
    raw_c = i2c.readfrom(slave_addr, 2) #raw C is 2 bytes
    value = int.from_bytes(raw_c, "big") # use builtin to convert to integer
    return value


def read_c2(): #read PROM value C2
    data = bytearray([r_c2])
    i2c.writeto(slave_addr, data)
    raw_c = i2c.readfrom(slave_addr, 2) #raw C is 2 bytes
    value = int.from_bytes(raw_c, "big") # use builtin to convert to unsigned integer
    return value

def read_c3(): #read PROM value C3
    data = bytearray([r_c3])
    i2c.writeto(slave_addr, data)
    raw_c = i2c.readfrom(slave_addr, 2) #raw C is 2 bytes
    value = int.from_bytes(raw_c, "big") # use builtin to convert to unsigned integer
    return value

def read_c4(): #read PROM value C4
    data = bytearray([r_c4])
    i2c.writeto(slave_addr, data)
    raw_c = i2c.readfrom(slave_addr, 2) #raw C is 2 bytes
    value = int.from_bytes(raw_c, "big") # use builtin to convert to unsigned integer
    return value

def read_c5(): #read PROM value C5
    data = bytearray([r_c5])
    i2c.writeto(slave_addr, data)
    raw_c = i2c.readfrom(slave_addr, 2) #raw C is 2 bytes
    value = int.from_bytes(raw_c, "big") # use builtin to convert to unsigned integer
    return value

def read_c6(): #read PROM value C6
    data = bytearray([r_c6])
    i2c.writeto(slave_addr, data)
    raw_c = i2c.readfrom(slave_addr, 2) #raw C is 2 bytes
    value = int.from_bytes(raw_c, "big") # use builtin to convert to unsigned integer
    return value
  
# read PROM calibration data  
C1 = read_c1()
C2 = read_c2()
C3 = read_c3()
C4 = read_c4()
C5 = read_c5()
C6 = read_c6()

"""print ('PROM C1 = ', C1)
print ('PROM C2 = ', C2)
print ('PROM C3 = ', C3)
print ('PROM C4 = ', C4)
print ('PROM C5 = ', C5)
print ('PROM C6 = ', C6)"""

# start D1 conversion - pressure (24 bit unsigned)
def start_d1():
    #print ('start D1 ')
    data = bytearray([r_d1])
    i2c.writeto(slave_addr, data)

# start D2 conversion - temperature (24 bit unsigned)  
def start_d2():
    #print ('start D2 ')
    data = bytearray([r_d2])
    i2c.writeto(slave_addr, data) 

#read ADC
def read_adc(): #read ADC 24 bits unsigned
    data = bytearray([r_adc])
    i2c.writeto(slave_addr, data)
    adc = i2c.readfrom(slave_addr, 3) #ADC is 3 bytes
    value = int.from_bytes(adc, "big") # use builtin to convert to integer
    return value

#print ('i2c scan addresses found: ',scan_I2C())

Temp = None
Pres = None

def calcAirPressure():

    global Temp,Pres
    
    start_d1() # start D1 conversion
    utime.sleep(1.0) # short delay during conversion
    raw_d1 = read_adc()
    start_d2() # start D2 conversion
    utime.sleep(1.0)
    raw_d2 = read_adc()
    dT = raw_d2 - (C5 * 256) # difference between actual and ref temp
    Temp = (2000 + (dT * (C6/8388608)))/100 #actual temperature
    OFF = (C2*131072) + (C4*dT/64) # offset at actual temperature
    SENS = (C1*65536) + (C3*dT/128) # pressure offset at actual temperature
    Pres = (raw_d1*SENS/2097152 - OFF)/3276800 # barometric pressure
    print(Temp,Pres)
    utime.sleep(1.0)