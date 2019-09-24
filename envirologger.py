#!/usr/bin/python3
"""
Names: Jason Pilbrough and Richard Masson
Student Number: PLBJAS001, MSSRIC004
Prac: ProjectA
Date: 24/09/19
"""

# import Relevant Librares
import RPi.GPIO as GPIO
import time
import spidev
import smbus

#GLOBAL VARIABLES

#BUTTONS
global BUTTON1; BUTTON1 = 31
global BUTTON2; BUTTON2 = 33
global BUTTON3; BUTTON3 = 35
global BUTTON4; BUTTON4 = 37

#I2C
global I2C_BUS; I2C_BUS = 1 # 1 indicates /dev/i2c-1  
global RTC_ADDRESS; RTC_ADDRESS = 0x6f
global RTC;
global RTC_SEC; RTC_SEC = 0x0 #address of seconds register on RTC
global RTC_MIN; RTC_MIN = 0x1 #address of minutes register on RTC
global RTC_HOUR; RTC_HOUR = 0x2 #address of hours register on RTC

#SPI
global SPI_BUS; SPI_BUS = 0 # 0 or 1
global SPI_DEVICE; SPI_DEVICE = 0 #chip select pin (0 or 1)
global SPI_MODE; SPI_MODE = 0
global SPI_MAX_SPEED; SPI_MAX_SPEED=488000

#ADC
global RESOLUTION; RESOLUTION = 10 #10 bit ADC
global VREF; VREF = 3.3 #reference voltage: 3.3V


#CONFIG GPIO PINS (ONLY RUNS ONCE)
def config():
    GPIO.setmode(GPIO.BOARD)
    #CONFIG OUTPUT PINS
    
    #CONFIG INPUT PINS
    GPIO.setup(BUTTON1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #config with pull down resistor
    GPIO.add_event_detect(BUTTON1, GPIO.RISING, callback=startStopMonitoring, bouncetime=200)  # add rising edge detection with debouncing on a channel

    GPIO.setup(BUTTON2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # config with pull down resistor
    GPIO.add_event_detect(BUTTON2, GPIO.RISING, callback=dismissAlarm, bouncetime=200)  # add rising edge detection with debouncing on a channel

    GPIO.setup(BUTTON3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # config with pull down resistor                                                                                                             
    GPIO.add_event_detect(BUTTON3, GPIO.RISING, callback=resetSysTime, bouncetime=200)  # add rising edge detection with debouncing on a channel 

    GPIO.setup(BUTTON4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # config with pull down resistor                                                                                                             
    GPIO.add_event_detect(BUTTON4, GPIO.RISING, callback=changeReadInteval, bouncetime=200)  # add rising edge detection with debouncing on a channel 

    #CONFIG I2C FOR RTC
    global RTC; RTC = smbus.SMBus(I2C_BUS)
    RTC.write_byte_data(RTC_ADDRESS, RTC_SEC, 0b10000000) # enable oscillator in SEC register

def readADC(channel):
    spi = spidev.SpiDev() #Enable SPI                                                                                                        
    spi.open(SPI_BUS, SPI_DEVICE) #Open connection to a specific bus and device (CS pin)                                                                                                                  
    spi.max_speed_hz = SPI_MAX_SPEED
    spi.mode = SPI_MODE
    config_bits = [0b1, (8+channel)<<4,0b0] #byte0=1 to start, byte2=selecting ADC channel,  byte3=0 ?
    reply = spi.xfer(config_bits)

    adc = 0 #build reply int from byte array
    for x in reply:
        adc = (adc << 8) + x

    voltage = round(adc*VREF/(2**RESOLUTION),3);
    
    #print(round(voltage,2),"V")
    spi.close() #close connection
    return voltage

def readTime():
    sec = RTC.read_byte_data(RTC_ADDRESS, RTC_SEC) - 128 #first bit is oscilator bit, must remove to get seconds value
    minute = RTC.read_byte_data(RTC_ADDRESS, RTC_MIN)
    hour = RTC.read_byte_data(RTC_ADDRESS, RTC_HOUR)
    return f"{hour:02}:{minute:02}:{sec:02}"

def displayInfo(time, humidity, light):
    print(f"RTC time: {time}, Humidity: {humidity}, Light: {light}")
    

def startStopMonitoring(pos):
    print("startStopMonitoring")

def dismissAlarm(pos):
    print("dismissAlarm")

def resetSysTime(pos):
    print("restSysTime")

def changeReadInteval(pos):
    print("changeReadInteval")

def main():
    humidityReading = readADC(0);
    lightReading = readADC(1);
    timeReading = readTime();
    displayInfo(timeReading, humidityReading, lightReading);
    #print("Humidity: ",humidityReading,"Light: ",lightReading)
    time.sleep(1)
    
# Only run the functions if 
if __name__ == "__main__":
    # Make sure the GPIO is stopped correctly
    try:
        config()
        while True:
            main()
    except KeyboardInterrupt:
        print("Exiting gracefully")
        # Turn off your GPIOs here
        GPIO.cleanup()
    except e:
        GPIO.cleanup()
        print("Some other error occurred")
        print(e.message)
