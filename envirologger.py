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

#GLOBAL VARIABLES

#BUTTONS
global BUTTON1; BUTTON1 = 31
global BUTTON2; BUTTON2 = 33
global BUTTON3; BUTTON3 = 35
global BUTTON4; BUTTON4 = 37

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
    #GPIO.setup(CHANNEL_LED1, GPIO.OUT, initial=GPIO.LOW)
    #CONFIG INPUT PINS
    GPIO.setup(BUTTON1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #config with pull down resistor
    GPIO.add_event_detect(BUTTON1, GPIO.RISING, callback=startStopMonitoring, bouncetime=200)  # add rising edge detection with debouncing on a channel

    GPIO.setup(BUTTON2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # config with pull down resistor
    GPIO.add_event_detect(BUTTON2, GPIO.RISING, callback=dismissAlarm, bouncetime=200)  # add rising edge detection with debouncing on a channel

    GPIO.setup(BUTTON3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # config with pull down resistor                                                                                                             
    GPIO.add_event_detect(BUTTON3, GPIO.RISING, callback=resetSysTime, bouncetime=200)  # add rising edge detection with debouncing on a channel 

    GPIO.setup(BUTTON4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # config with pull down resistor                                                                                                             
    GPIO.add_event_detect(BUTTON4, GPIO.RISING, callback=changeReadInteval, bouncetime=200)  # add rising edge detection with debouncing on a channel 


def readADC():
    spi = spidev.SpiDev() #Enable SPI                                                                                                        
    spi.open(SPI_BUS, SPI_DEVICE) #Open connection to a specific bus and device (CS pin)                                                                                                                  
    spi.max_speed_hz = SPI_MAX_SPEED
    spi.mode = SPI_MODE
    config_bits = [0b1, 0b10000000,0b0] #byte0=1 to start, byte2=selecting ADC channel,  byte3=0 ?
    reply = spi.xfer(config_bits)

    adc = 0 #build reply int from byte array
    for x in reply:
        adc = (adc << 8) + x

    voltage = adc*VREF/(2**RESOLUTION);
    
    print(round(voltage,2),"V")
    spi.close() #close connection   

def startStopMonitoring(pos):
    print("startStopMonitoring")

def dismissAlarm(pos):
    print("dismissAlarm")

def resetSysTime(pos):
    print("restSysTime")

def changeReadInteval(pos):
    print("changeReadInteval")

def main():
    readADC()
    time.sleep(0.2)
    
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
