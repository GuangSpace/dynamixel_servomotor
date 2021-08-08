
"""

Operation type  : Joint mode , ID = 1
description     : This is the first version script of servo motor, you can modify it or redistribute it.
                  We use Raspberry pi to be our main controller. We set the direction pin == GPIO18,
                  baudrare == 9600.
                  
                  To control the Dynamixel, the code like:
                      
                      def TxDByte(data):
                          serial.wrtie(data)
                          
                      direction_port = direction_TX
                      
                      TxDByte("FF")
                      TxDByte("FF")
                      TxDByte("ID")
                      TxDByte("LENGTH")
                      TxDByte("INSTRUCTION")
                      TxDByte("Parameter0")
                      TxDByte("ParameterN")
                      
                      direction_port = direction_RX
                  
                  =============================================================
                  
                  if __name__ == "__main__":
                      Test().led() //loop(servo motor led light will turn on and then turn off)
                      or
                      Test().angle_change //loop(servo motor angle will from 180° turn to 360°)
                      
"""

import RPi.GPIO as GPIO
import serial
import time

class Test():
    
    def __init__(self):
        
        self.ser = serial.Serial("/dev/ttyAMA0" ,baudrate=9600 ,timeout=3.0)

        self.direction_pin = 18                                                
        self.tx = GPIO.HIGH                                                    
        self.rx = GPIO.LOW
        
        self.transmit_delaytime = 1
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT) 

    def direction(self, d):                                                    
        GPIO.output(self.direction_pin, d)
        
    def led(self):
        try:
            flag = True
            while flag:
                self.direction(self.tx)
                self.ser.write(bytearray.fromhex("ff ff 01 04 03 19 01 dd"))   #led on
                time.sleep(self.transmit_delaytime)
                self.direction(self.rx)

                self.direction(self.tx)
                self.ser.write(bytearray.fromhex("ff ff 01 04 03 19 00 de"))   #led off
                time.sleep(self.transmit_delaytime)
                self.direction(self.rx)
                
        except KeyboardInterrupt:
            flag = False
        
        finally:
            GPIO.cleanup()
    
    def angle_change(self):
        try:
            flag = True
            while flag:
                self.direction(self.tx)
                self.ser.write(bytearray.fromhex("FF FF 01 05 03 1E 00 08 D0")) #180°
                time.sleep(self.transmit_delaytime)
                self.direction(self.rx)

                self.direction(self.tx)
                self.ser.write(bytearray.fromhex("FF FF 01 05 03 1E FF 0F CA")) #360°
                time.sleep(self.transmit_delaytime)
                self.direction(self.rx)
        
        except KeyboardInterrupt:
            flag = False
            
        finally:
            GPIO.cleanup()
        
if __name__ == "__main__":
    pass












