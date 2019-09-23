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

#GLOBAL VARIABLES
global BUTTON1; BUTTON1 = 31
global BUTTON2; BUTTON2 = 33
global BUTTON3; BUTTON3 = 35
global BUTTON4; BUTTON4 = 37


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


def startStopMonitoring():
    return

def dismissAlarm():
    return

def resetSysTime():
    return

def changeReadInteval():
    return

def main():
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
