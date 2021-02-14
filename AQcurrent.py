#! /usr/bin/env python3

'''
This python3 program measures the AirQuality by measuring temperature, humidity
pressure and altitude using a Waveshare BCM280 sensor and an SDS011 sensor for 
PM2.5 and PM10. It indicates the AQ using an RGB LED. It send the data to a
ThingSpeak channel.
'''
# Import all needed modules.
import serial, time, board, digitalio, busio, time, adafruit_bme280, thingspeak
from gpiozero import LED

# Functions.
def changeLED(pmtwofive1, pmten1):
    '''Changes the LED, according to AQ.'''
    if pmtwofive1 <= 12 and pmten1 <= 50:        # if the AQ is good, it lights 
        red.off()                                #  green
        green.on()
        blue.off()
    elif pmtwofive1 > 12 and pmtwofive1 < 35 or pmten1 > 50 and pmten1 < 100:
        red.on()                                # if AQ is moderate, it light
        green.on()                              #  yellow
        blue.off()
    elif pmtwofive1 >= 35 or pmten1 >= 100:   # if AQ gets really bad, it lights
        red.on()                              #  red
        green.off()
        blue.off()

def readBME280():
    '''Reads the BME280 sensor and assigns global variables.'''
    global temp
    temp = int(bme280.temperature)
    global humidity
    humidity = int(bme280.relative_humidity)
    global pressure
    pressure = int(bme280.pressure)
    global altitude
    altitude = int(bme280.altitude)

def readSDS011():
    '''Read the SDS011 sensors and assigns value to global PM variables.'''
    data = []
    for index in range(0, 10):
        datum = ser.read()
        data.append(datum)
        
    global pmtwofive    
    pmtwofive = int.from_bytes(b''.join(data[2:4]), byteorder='little') / 10
    global pmten
    pmten = int.from_bytes(b''.join(data[4:6]), byteorder='little') / 10

def measure(channel):
    # Read the BME280 sensor.
    readBME280()
    # Reads SDH011.
    readSDS011()
    changeLED(pmtwofive, pmten)
    # write
    response = channel.update({'field1': temp,
        'field2': humidity,
        'field3': pressure,
        'field4': altitude,
        'field5': pmtwofive,
        'field6': pmten})
 
# ThingSpeak channels and keys.
channel_id = CHANNELID 
api_key  = 'THINGSPEAKAPIKEY'

# Variables.
temp = None
humidity = None
pressure = None
altitude = None
pmtwofive = None
pmten = None

# Assigns LEDs.
red = LED(17)       # set the GPIO17 as an output pin for RED
green = LED(27)     # set the GPIO27 as an output pin for GREEN
blue = LED(22)      # set the GPIO22 as an output pin for BLUE

# Initialize the USB SDH011
ser = serial.Serial('/dev/ttyUSB0')

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# change this to match the location's pressure (hPa) at sea level
bme280.sea_level_pressure = 1013.25
   
if __name__ == "__main__":
    channel = thingspeak.Channel(id=channel_id, api_key=api_key)
    while True:
        try:
            measure(channel)
            # free account has an api limit of 15sec
            time.sleep(60)
        except:
            time.sleep(600)
            continue
