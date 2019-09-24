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
from datetime import datetime
import spidev
import smbus

#GLOBAL VARIABLES

#BUTTONS
global BUTTON1; BUTTON1 = 31
global BUTTON2; BUTTON2 = 33
global BUTTON3; BUTTON3 = 35
global BUTTON4; BUTTON4 = 37

#PWM
global PWM
global PWM_OUT; PWM_OUT = 32 #pin used for PWM
global PWM_FREQ; PWM_FREQ = 1 #frequency of LED flash

#ALARM
global ALARM_TRIGGERED; ALARM_TRIGGERED = False # indicates whether an alarm has been triggered

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
global RESOLUTION; ADC_RESOLUTION = 10 #10 bit ADC
global VREF; VREF = 3.3 #reference voltage: 3.3V

#TIMING
global READ_INTERVAL; READ_INTERVAL = 1 # can be 1s, 2s or 5s -> updated by changeReadInterval()
global IS_MONITORING; IS_MONITORING = True # -> updated by startStopMonitoring() 


#TEMPERATURE SENSOR
global TEMP_0; TEMP_0 = 0.5   #voltage of temperature sensor at 0 degrees celcius
global TEMP_COEFFICIENT; TEMP_COEFFICIENT = 0.01   #temperature sensor coefficent

#TIME
global SYS_TIME_REF; SYS_TIME_REF = 0 #time that system time was last reset



#CONFIG GPIO PINS (ONLY RUNS ONCE)
def config():
    GPIO.setmode(GPIO.BOARD)
    
    #CONFIG OUTPUT PINS AND PWM
    GPIO.setup(PWM_OUT, GPIO.OUT) #setup PWM output pin
    global PWM; PWM = GPIO.PWM(PWM_OUT, PWM_FREQ)
    
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
    RTC.write_byte_data(RTC_ADDRESS, RTC_SEC, 0b10000000) # enable ST (start oscillator) bit

    #SYS TIME CONFIG
    global SYS_TIME_REF; SYS_TIME_REF = time.time()
    print(SYS_TIME_REF);


def readADC(channel): #Channels: 1 = pot (humidity), 2 = LDR, 3 = temperature
    spi = spidev.SpiDev() #Enable SPI                                                                                                        
    spi.open(SPI_BUS, SPI_DEVICE) #Open connection to a specific bus and device (CS pin)                                                                                                                  
    spi.max_speed_hz = SPI_MAX_SPEED
    spi.mode = SPI_MODE
    config_bits = [0b1, (8+channel)<<4,0b0] #byte0=1 to start, byte2=selecting ADC channel,  byte3=0 ?
    reply = spi.xfer(config_bits)

    adc = 0 #build reply int from byte array
    for x in reply:
        adc = (adc << 8) + x
    
    spi.close() #close connection
    return adc

def readRTC():
    sec_byte = RTC.read_byte_data(RTC_ADDRESS, RTC_SEC) - 128 #first bit is oscilator bit, must remove to get seconds value
    sec = int(((sec_byte&0b01110000)>>4)*10) + int(sec_byte&(0b00001111)) # convert to right format
    minute_byte = RTC.read_byte_data(RTC_ADDRESS, RTC_MIN)
    minute = int(((minute_byte&0b01110000)>>4)*10) + int(minute_byte&(0b00001111)) # convert to right format
    hour_byte = RTC.read_byte_data(RTC_ADDRESS, RTC_HOUR)
    hour = int(((hour_byte&0b00010000)>>4)*10) + int(hour_byte&(0b00001111)) # convert to right format
    #return time.strptime(f"{hour:02}:{minute:02}:{sec:02}","%H:%M:%S")
    return f"{hour:02}:{minute:02}:{sec:02}"

    
def triggerAlarm():
    global PWM; global ALARM_TRIGGERED
    if(ALARM_TRIGGERED):
        return

    PWM.ChangeFrequency(PWM_FREQ) #Bug in PWM that require freq to be reset on pwm restart (Do not remove)
    PWM.start(50)
    ALARM_TRIGGERED = True

def displayInfo(RTCtime,systime, humidity,temp, light, DACout):
    global ALARM_TRIGGERED;
    alarmStr = ""
    if(ALARM_TRIGGERED):
        alarmStr = "(*)"
    print(f"{RTCtime:10} {systime:10} {humidity:8}V {temp:8} C {light:8} {DACout:8}V {alarmStr:5}")
    

def startStopMonitoring(pos):
    global IS_MONITORING
    IS_MONITORING = not(IS_MONITORING)

def dismissAlarm(pos):
    global ALARM_TRIGGERED
    PWM.stop()
    ALARM_TRIGGERED = False

def resetSysTime(pos):
    RTC.write_byte_data(RTC_ADDRESS, RTC_SEC, 0b10000000)  #ensure enable clock bit is set
    RTC.write_byte_data(RTC_ADDRESS, RTC_MIN, 0b00000000)
    RTC.write_byte_data(RTC_ADDRESS, RTC_HOUR, 0b00000000)

    
#TOGGLE READ INTERVAL BETWEEN 1s, 2s, and 5s
def changeReadInteval(pos): 
    global READ_INTERVAL
    if(READ_INTERVAL==1):
        READ_INTERVAL=2
    elif(READ_INTERVAL==2):
        READ_INTERVAL=5
    elif(READ_INTERVAL==5):
        READ_INTERVAL=1

def main():
    global IS_MONITORING
    if(IS_MONITORING): #only read values if currently monitoring
        humidityReading = readADC(0);
        humidityVoltage = round(humidityReading*VREF/(2**ADC_RESOLUTION),1); #convert humidity reading to voltage 
        lightReading = readADC(1);
        tempReading = readADC(2);
        tempVoltage = round(tempReading*VREF/(2**ADC_RESOLUTION),3); #convert temperature reading to voltage
        ambientTemp = round((tempVoltage-TEMP_0)/(TEMP_COEFFICIENT),3)
        #print(tempReading, tempVoltage, ambientTemp)
        RTCtimeReading = readRTC();
        #systime = time.mktime(RTCtimeReading) - SYS_TIME_REF #calculate current system time based on current RTC time and last reset of sys time
        systime = "00:00:00"
        DACout = round((lightReading/(2**ADC_RESOLUTION)) *  humidityVoltage,3)
        if(DACout<0.65 or DACout>2.65):
            triggerAlarm()
        displayInfo(RTCtimeReading,systime, humidityVoltage, ambientTemp, lightReading, DACout);
    time.sleep(READ_INTERVAL)

    
# Only run the functions if 
if __name__ == "__main__":
    # Make sure the GPIO is stopped correctly
    try:
        config()
        print(f"{'RTC time':<10} {'sys time':<10} {'Humidity':>8} {'Temp':>8} {'Light':>8} {'DAC out':>8} {'Alarm':>5}")
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
