#!/usr/bin/env python3
"""
Buttons to start recoding.
"""

# GPIO pin for detecting button input
# Feel free to change this
PIN_SCAN = 17


import RPi.GPIO as GPIO
import signal
import sys



def main():
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_SCAN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    while True:
        print ('Waiting for Arduino to Start Velodyne')
        btnInp1 = GPIO.input(PIN_SCAN)

      
        if not btnInp1:
            print("Start Recording")
            break   

if __name__ == "__main__":
    main()
