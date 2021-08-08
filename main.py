from dynamixel_multi import Dy_motor
import RPi.GPIO as GPIO
import time

try:
    
    command = Dy_motor()

    command.wheel_mode()
        
except KeyboardInterrupt:
    flag = False
    
finally:
    GPIO.cleanup()