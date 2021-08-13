import numpy as np
import RPi.GPIO as GPIO
import time

ledPin = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(ledPin, GPIO.OUT)

for i in range(10):
    GPIO.output(ledPin, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(ledPin, GPIO.LOW)
    time.sleep(2)
    
GPIO.cleanup()
print('all done!')