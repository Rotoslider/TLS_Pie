#!/usr/bin/env python3
"""
Waits for a button press, then terminates.
"""

# GPIO pin for detecting button input
# Feel free to change this
PIN_KILL = 27


import RPi.GPIO as GPIO
import signal
import sys

def catchTerm(signal, frame):
    # Called when SIGTERM is detected
    sys.exit(0)

def main():
    signal.signal(signal.SIGTERM, catchTerm)
    signal.signal(signal.SIGINT, catchTerm)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_KILL, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    while True:
        btnInp = GPIO.input(PIN_KILL)
        if not btnInp:
            print("Button press detected STOPPED PCAP RECORD")
            break

if __name__ == "__main__":
    main()
