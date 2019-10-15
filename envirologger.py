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
import blynklib

#GLOBAL VARIABLES

#BUTTONS
global BUTTON1; BUTTON1 = 31                       # start/stop monitoring
global BUTTON2; BUTTON2 = 33                       # dismiss  alarm
global BUTTON3; BUTTON3 = 35                       # reset system time
global BUTTON4; BUTTON4 = 37                       # change read interval

#PWM
global PWM                                         # stores PWM reference
global PWM_OUT; PWM_OUT = 32                       # GPIO pin used for PWM
global PWM_FREQ; PWM_FREQ = 1                      # frequency of LED flash

#ALARM
global ALARM_TRIGGERED; ALARM_TRIGGERED = False    # indicates whether an alarm has been triggered
global LAST_ALARM_TIME; LAST_ALARM_TIME = 0        # stores time in millis of last alarm trigger

#I2C
global I2C_BUS; I2C_BUS = 1                        # 1 indicates /dev/i2c-1  
global RTC_ADDRESS; RTC_ADDRESS = 0x6f             # address of RTC
global RTC;                                        # stores RTC reference
global RTC_SEC; RTC_SEC = 0x0                      # address of seconds register on RTC
global RTC_MIN; RTC_MIN = 0x1                      # address of minutes register on RTC
global RTC_HOUR; RTC_HOUR = 0x2                    # address of hours register on RTC

#SPI
global SPI_BUS; SPI_BUS = 0                        # 0 or 1 - only SPI0 is used in this app
global SPI_DEVICE_ADC; SPI_DEVICE_ADC = 0          # chip select of ADC (0 or 1)
global SPI_DEVICE_DAC; SPI_DEVICE_DAC = 1          # chip select of DAC (0 or 1)
global SPI_MODE; SPI_MODE = 0                      # always mode 0
global SPI_MAX_SPEED; SPI_MAX_SPEED=488000         # spi clock speed

#ADC AND DAC
global VREF; VREF = 3.3                            # reference voltage: 3.3V
global ADC_RESOLUTION; ADC_RESOLUTION = 10         # 10 bit ADC
global DAC_RESOLUTION; DAC_RESOLUTION = 10         # 10 bit DAC 

#SAMPLING
global READ_INTERVAL; READ_INTERVAL = 1            # can be 1s, 2s or 5s -> updated by changeReadInterval()
global IS_MONITORING; IS_MONITORING = True         # -> updated by startStopMonitoring() 


#TEMPERATURE SENSOR
global TEMP_0; TEMP_0 = 0.5                        #voltage of temperature sensor at 0 degrees celcius
global TEMP_COEFFICIENT; TEMP_COEFFICIENT = 0.01   #temperature sensor coefficent

#SYSTEM TIME
global SYS_TIME_REF; SYS_TIME_REF = 0              #time in millis of last system reset

# BLYNK VIRTUAL PINS
global virtTemp;virtTemp = 1
global virtHum; virtHum = 2
global virtLight; virtLight = 3
global virtTime; virtTime = 4
global virtAlarm; virtAlarm = 7
global virtVolt; virtVolt = 5
global virtSlide; virtSlide = 6
global terminal; terminal = 0;


#INITIALISE BLYNK COMMUNICATION CHANNEL
BLYNK_AUTH = 'mXqD_hyad0IgtjuXt0r7We_y7JaTxIEh'
blynk = blynklib.Blynk(BLYNK_AUTH)

#CONFIG GPIO PINS (ONLY RUNS ONCE)
def config():
    GPIO.setmode(GPIO.BOARD)
    
    
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
    secondsReg = RTC.read_byte_data(RTC_ADDRESS, RTC_SEC) #read the value in seconds register currenly
    RTC.write_byte_data(RTC_ADDRESS, RTC_SEC, 0b10000000|secondsReg) # enable ST (start oscillator) bit without changing seconds value

    #CONFIG SYS TIME AT BOOT 
    global SYS_TIME_REF; SYS_TIME_REF = time.time()
    blynk.run() #needs to run once before clear command can be run
    blynk.virtual_write(terminal, "clr") #clear blynk terminal on boot
    blynk.virtual_write(virtSlide, 1) #set readInterval in Blynk app to 1 by default

    #CONFIG OUTPUT PINS AND PWM                                                                                                                                                                            
    GPIO.setup(PWM_OUT, GPIO.OUT) #setup PWM output pin                                                                                                                                                    
    global PWM; PWM = GPIO.PWM(PWM_OUT, PWM_FREQ)
    GPIO.output(PWM_OUT, GPIO.LOW) #set alarm pin initially low
    
    
def readADC(channel): #Channels: 1 = pot (humidity), 2 = LDR, 3 = temperature
    spi = spidev.SpiDev() #Enable
    spi.open(SPI_BUS, SPI_DEVICE_ADC) #Open connection to a specific bus and device (CS pin)                                                                                                                  
    spi.max_speed_hz = SPI_MAX_SPEED
    spi.mode = SPI_MODE
    config_bits = [0b1, (8+channel)<<4,0b0] #byte0=1 to start, byte2=selecting ADC channel,  byte3=0 ?
    reply = spi.xfer(config_bits)

    adc = 0 #build reply int from byte array
    for x in reply:
        adc = (adc << 8) + x
    
    spi.close() #close connection
    return adc

def writeDAC(value):
    DAC_code = int((value / VREF * (2**DAC_RESOLUTION)))
    byte1 = 0b0011 << 4 | DAC_code >> 6
    byte2 = (DAC_code << 2) % 256 

    spi = spidev.SpiDev() #Enable
    spi.open(SPI_BUS, SPI_DEVICE_DAC) #Open connection to a specific bus and device (CS pin)
    spi.max_speed_hz = SPI_MAX_SPEED
    spi.mode = SPI_MODE

    to_send = [byte1 , byte2] #define what to send
    spi.xfer(to_send)

    # Close the SPI connection
    spi.close()

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
    strToPrint = f"{RTCtime:10} {systime:10} {humidity:8}V {temp:8} C {light:8} {DACout:8}V {alarmStr:>6}"
    print(strToPrint)
    toBlynk = str(RTCtime) +" " +str(systime) +" " +str(humidity) +"V " +str(temp) +"C " +str(light) +"  " +str(DACout) + " " + alarmStr +"\n"
    blynk.virtual_write(terminal, toBlynk)

def startStopMonitoring(pos):
    global IS_MONITORING
    IS_MONITORING = not(IS_MONITORING)

def dismissAlarm(pos):
    global ALARM_TRIGGERED
    PWM.stop()
    blynk.virtual_write(virtAlarm, 0)
    ALARM_TRIGGERED = False
    print("Alarm Dismissed")

def resetSysTime(pos):
    global SYS_TIME_REF; SYS_TIME_REF = time.time() #update reference used to calculate sys time from RTC time
    blynk.virtual_write(terminal, "clr")
    toBlynk = "RTC Time Sys Time Hum Temp Li  Vout Alarm\n"
    blynk.virtual_write(terminal, toBlynk)
    valueToWrite = READ_INTERVAL
    if(valueToWrite==5):
        valueToWrite=3

    blynk.virtual_write(virtSlide, valueToWrite)
    
def setRTCtime(hour, min, sec):
    secBits = 0b10000000 | sec % 10 | (sec // 10)<<4
    minBits = min % 10 | (min // 10)<<4
    hourBits = hour % 10 | (hour // 10)<<4
    RTC.write_byte_data(RTC_ADDRESS, RTC_SEC, secBits)
    RTC.write_byte_data(RTC_ADDRESS, RTC_MIN, minBits)                                                                                                                                                 
    RTC.write_byte_data(RTC_ADDRESS, RTC_HOUR, hourBits)
    
#TOGGLE READ INTERVAL BETWEEN 1s, 2s, and 5s
def changeReadInteval(pos): 
    global READ_INTERVAL
    #READ_INTERVAL = (READ_INTERVAL +1)%5
    if(READ_INTERVAL==1):
        READ_INTERVAL=2
    elif(READ_INTERVAL==2):
        READ_INTERVAL=5
    elif(READ_INTERVAL==5):
        READ_INTERVAL=1

    valueToWrite = READ_INTERVAL
    if(valueToWrite==5):
        valueToWrite=3
        
    blynk.virtual_write(virtSlide, valueToWrite)
		
@blynk.handle_event('write V6')
def read_virtual_pin_handler(pin, value):
    global READ_INTERVAL
    temp = int( value[0])
    if(temp==3):
        temp = 5

    READ_INTERVAL = int(temp)

def main():
    global IS_MONITORING
    if(IS_MONITORING): #only read values if currently monitoring
        blynk.run() #initialise blynk monitor
        humidityReading = readADC(0);
        humidityVoltage = round(humidityReading*VREF/(2**ADC_RESOLUTION),1); #convert humidity reading to voltage 
        blynk.virtual_write(virtHum, humidityVoltage)
        lightReading = readADC(1);
        blynk.virtual_write(virtLight, lightReading)
        tempReading = readADC(2);
        tempVoltage = round(tempReading*VREF/(2**ADC_RESOLUTION),3); #convert temperature reading to voltage
        ambientTemp = round((tempVoltage-TEMP_0)/(TEMP_COEFFICIENT))
        blynk.virtual_write(virtTemp, ambientTemp)
        
        #Calculate systime by subtracting system time ref from RTC time
        RTCtimeReading = readRTC();
        time.strptime(RTCtimeReading, "%H:%M:%S")
        timeDiff = time.time()-SYS_TIME_REF
        
        systime = time.strftime("%H:%M:%S", time.gmtime(timeDiff))
        blynk.virtual_write(virtTime, systime)

        DACout = round((lightReading/(2**DAC_RESOLUTION)) *  humidityVoltage,3) 
        blynk.virtual_write(virtVolt, DACout)
        writeDAC(DACout)

        global LAST_ALARM_TIME
        if((DACout<0.65 or DACout>2.65) and ((time.time() - LAST_ALARM_TIME) > 180)): # DAC value outside range and no alarm in last 180 seconds
            triggerAlarm()
            blynk.virtual_write(virtAlarm, 255)
            LAST_ALARM_TIME = time.time() # update time of last alarm to current time
            
        displayInfo(RTCtimeReading,systime, humidityVoltage, ambientTemp, lightReading, DACout);

    time.sleep(READ_INTERVAL)

    
# Only run the functions if 
if __name__ == "__main__":
    # Make sure the GPIO is stopped correctly
    try:
        config()
        #setRTCtime(1,53,00)

        #write headings
        print(f"{'RTC Time':<10} {'Sys Time':<10} {'Humidity':>9} {'Temp':>10} {'Light':>8} {'DAC out':>9} {'Alarm':>6}")
        toBlynk = "RTC Time Sys Time Hum Temp Li  Vout Alarm\n"
        #toBlynk = "RTC Time  Sys Time  Hum  Temp Light  Vout  Alarm\n"
        blynk.virtual_write(terminal, toBlynk)
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
