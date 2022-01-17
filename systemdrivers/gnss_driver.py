from machine import UART,Pin,Timer
import time

"""4 Hz"""
gps_module = UART(0,baudrate = 9600, tx = Pin(28), rx = Pin(29))

timer = Timer()

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
def gps(timer):
    
    global latitude,longitude,height,groundspeed,gpsyaw,geoidalseperation,hemisLon,hemisLat
    
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
    
    print(latitude,hemisLat,longitude,hemisLon,height,geoidalseperation,groundspeed,gpsyaw)
    #print(data)

reset = Pin(7,Pin.OUT)

reset.value(1)
time.sleep(1)
gps_module.write("$PMTK101")