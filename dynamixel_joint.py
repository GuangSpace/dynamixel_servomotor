'''

Operation type  : Joint mode , ID = 1
description     : The joint mode can be used to multi-joints robot 
                  since the robots can be controlled with specific angles.

                  =============================================================

                  from dynamixel_joint import Dy_motor
                  import RPi.GPIO as GPIO
                    
                  try:
                      command = Dy_motor()
                      command.goal_position(180)
                      
                  except KeyboardInterrupt:
                      print("KeyboardInterrupt")
                        
                  finally:
                      GPIO.cleanup()
    
'''

import serial
import time
import RPi.GPIO as GPIO
import math

class Initialize():                                                            # define the reapberry pi and Dynamixel motor
    
    def raspi_set(self, dev_name = "/dev/ttyAMA0" ,baudrate=9600):

        self.ser = serial.Serial(dev_name ,baudrate ,timeout=3.0)    
        
        self.direction_pin = 18                                                # GPIO pin 18
        self.tx = GPIO.HIGH                                                    # HIGH for TX mode
        self.rx = GPIO.LOW                                                     # LOW for RX mode

        self.transmit_delaytime = 0.01                                         
        self.rotate_delaytime = 1
    
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)   
        
    def define_register(self):
        
        self.MX_id_reg = 3                                                     
        self.MX_led_reg = 25
        self.MX_goalposition_reg = 30
        self.MX_movingspeed_reg = 32
        self.MX_present_position_reg = 36
        self.MX_temperature_reg = 43
        
        self.MX_write = 3                                                      
        self.MX_read = 2                                                       
        self.MX_length_of_data_1 = 1                                           
        self.MX_length_of_data_2 = 2                                           
        self.MX_led_on = 1                                                     
        self.MX_led_off = 0                                                    
        
class Operation():                                                             
    
    def ID_hex(self, arg):
        
        arg = "0"+ str(arg)
        
        return arg
    
    def dec2hex(self, number):                                                 

        hex_number = hex(number)
        hex_list = list(hex_number[2:])  
        arg = hex_list[0] + hex_list[1]

        return arg

    def hex2int(self, arg):                                                    
    
        arg = list(hex(arg))
    
        flag = True
    
        while flag:
        
            if len(arg) < 4:
            
                arg.insert(2,"0")
            
            elif len(arg) == 4:
            
                flag = False
    
    
        total = arg[2] + arg[3]
    
        return total
    
    def str2list(self, cs_arg):                                                
        
        cs_arg = list(cs_arg[2:])

        flag = True
        
        while flag :
    
            if len(cs_arg) == 2:
    
                flag = False

            elif len(cs_arg) < 2:
        
                cs_arg.insert(0,"0")
                
        cs_list = []
        cs_list.append(cs_arg[0]+cs_arg[1])
        
        return cs_list[0]
    
    def dec_conversion(self, speed):

        hex_number = hex(speed)
        hex_list = list(hex_number[2:])
        
        flag = True
        
        while flag :
    
            if len(hex_list) == 4:
    
                flag = False

            elif len(hex_list) < 4:
        
                hex_list.insert(0,"0")

        speed_L = hex_list[2]+hex_list[3]
        speed_H = hex_list[0]+hex_list[1]
        
        return speed_L , speed_H
    
    def angle_conversion(self, angle):
        
        float_number = float(angle / 0.0879120879120879)                                        
        check_number = round(float_number, 3)
        dec = math.ceil(check_number)
        hex_angle = hex(dec)
        hex_list = list(hex_angle[2:])
        
        flag = True
        
        while flag :
    
            if len(hex_list) == 4:
    
                flag = False

            elif len(hex_list) < 4:
        
                hex_list.insert(0,"0")
        
        angle_L = hex_list[2]+hex_list[3]
        angle_H = hex_list[0]+hex_list[1] 
        
        return angle_L , angle_H                                               

    def status_position(self, byte):
        
        final = self.hex2int(byte[5]) + self.hex2int(byte[4])
        
        print("angle = ",round(int(final,16)*0.0879120879120879))

    def status_ID(self, byte):
        
        if byte[3] == 0:
            print("ID = " , byte[4])
            
        else:
            print("error")

    def status_movingspeed(self, byte):                                        
        
        final = self.hex2int(byte[5]) + self.hex2int(byte[4])
        
        print("\nmoving speed = ",int(final,16))

    def status_temperature(self, byte):
        
        if byte[3] == 0:
            
            print("No error")
            print("temperature = " , byte[4])
            
        else:
            print("error")
            
class Dy_motor(Initialize,Operation):
    
    def __init__(self, ID=1):
                
        self.id = ID                                                           # default ID = 1
        self.string = ["ff","ff"]                                              # header byte
        
        self.raspi_set()
        self.define_register()
                
    def direction(self, d):                                                    # set direction of data.  rx or tx
        
        GPIO.output(self.direction_pin, d)
        
    def goal_position(self, angle):                                                                               
                                                                               
        del self.string[2:]                                                    
        length = 5
                
        self.string.append(self.ID_hex(self.id))                               
        self.string.append("0"+ str(length))                                   
        self.string.append("0"+ str(self.MX_write))                            
        self.string.append(self.dec2hex(self.MX_goalposition_reg))             

        (angle_L,angle_H) = self.angle_conversion(angle)
        self.string.insert(6,angle_L)                                           
        self.string.insert(7,angle_H)                                          

        checksum = hex(255-((self.id + length + self.MX_write + self.MX_goalposition_reg + int(angle_L,16) + int(angle_H,16))%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        print("Instruction Packet = " , self.string)                           
        
        tab = ' '
        self.output = tab.join(self.string)                                    

        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.rotate_delaytime)
        self.direction(self.rx)                                                
    
    def read_position(self):                                                   

        del self.string[2:]                                                                            
        length = 4

        self.string.append(self.ID_hex(self.id))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_read))                             
        self.string.append(self.dec2hex(self.MX_present_position_reg))
        self.string.append("0"+ str(self.MX_length_of_data_2))                 

        checksum = hex(255-((self.id + length + self.MX_read + self.MX_present_position_reg + self.MX_length_of_data_2)%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        print("Instruction Packet = " , self.string)
        
        tab = ' '
        self.output = tab.join(self.string)
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)
        reply = self.ser.read(10)                                              
        print("\nStatus Packet = " , reply)
        self.status_position(reply)        
 
    def moving_speed(self, speed):                                             
              
        del self.string[2:]
        length = 5
                
        self.string.append(self.ID_hex(self.id))                               
        self.string.append("0"+ str(length))                                   
        self.string.append("0"+ str(self.MX_write))                            
        self.string.append(self.dec2hex(self.MX_movingspeed_reg))              
        
        (speed_L,speed_H) = self.dec_conversion(speed)
        self.string.insert(6,speed_L)                                          
        self.string.insert(7,speed_H)                                          
                
        checksum = hex(255-((self.id + length + self.MX_write + self.MX_movingspeed_reg + int(speed_L,16) + int(speed_H,16))%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))                            
        
        print("Instruction Packet = " , self.string)
        
        tab = ' '
        self.output = tab.join(self.string)                                    
        
        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.rotate_delaytime) 
        self.direction(self.rx)  
        
    def read_movingspeed(self):
              
        del self.string[2:]
        length = 4
                
        self.string.append(self.ID_hex(self.id))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_read))
        self.string.append(self.dec2hex(self.MX_movingspeed_reg))
        self.string.append("0"+ str(self.MX_length_of_data_2))
        
        checksum = hex(255-((self.id + length + self.MX_read + self.MX_movingspeed_reg + self.MX_length_of_data_2)%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        print("Instruction Packet = " , self.string)
        
        tab = ' '
        self.output = tab.join(self.string)
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)
        reply = self.ser.read(10)
        print("\nStatus Packet = " , reply)
        self.status_movingspeed(reply) 
    
    def set_ID(self, new_id):
        
        del self.string[2:]
        length = 4
        
        self.string.append(self.ID_hex(self.id))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_write))
        self.string.append("0"+ str(self.MX_id_reg))
        self.string.append(self.ID_hex(new_id))  
        
        checksum = hex(255-((self.id + length + self.MX_write + self.MX_id_reg + new_id)%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        self.id = new_id
        print("new ID = " , self.id)
        print("Instruction Packet = " , self.string)

        tab = ' '
        self.output = tab.join(self.string)      
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)
    
    def read_ID(self):

        del self.string[2:]
        length = 4

        self.string.append(self.ID_hex(self.id))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_read))
        self.string.append("0"+ str(self.MX_id_reg))
        self.string.append("0"+ str(self.MX_length_of_data_1))
        
        checksum = hex(255-((self.id + length + self.MX_read + self.MX_id_reg + self.MX_length_of_data_1)%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        print("Instruction Packet = " , self.string)
        
        tab = ' '
        self.output = tab.join(self.string) 
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)
        reply = self.ser.read(10)
        print("\nStatus Packet = " , reply)
        self.status_ID(reply)

    def read_temperature(self):

        del self.string[2:]
        length = 4

        self.string.append(self.ID_hex(self.id))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_read))
        self.string.append(self.dec2hex(self.MX_temperature_reg))
        self.string.append("0"+ str(self.MX_length_of_data_1))     

        checksum = hex(255-((self.id + length + self.MX_read + self.MX_temperature_reg + self.MX_length_of_data_1)%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        print("Instruction Packet = " , self.string)
        
        tab = ' '
        self.output = tab.join(self.string)
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)
        reply = self.ser.read(10)
        print("\nStatus Packet = " , reply)
        self.status_temperature(reply)

    def led_on(self):
        
        del self.string[2:]
        length = 4
        
        self.string.append(self.ID_hex(self.id))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_write))
        self.string.append(self.dec2hex(self.MX_led_reg))
        self.string.append("0"+ str(self.MX_led_on))
        
        checksum = hex(255-((self.id + length + self.MX_write + self.MX_led_reg + self.MX_led_on)%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        print("Instruction Packet = " , self.string)
        
        tab = ' '
        self.output = tab.join(self.string)
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)

    def led_off(self):
        
        del self.string[2:]
        length = 4
        
        self.string.append(self.ID_hex(self.id))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_write))
        self.string.append(self.dec2hex(self.MX_led_reg))
        self.string.append("0"+ str(self.MX_led_off))
        
        checksum = hex(255-((self.id + length + self.MX_write + self.MX_led_reg + self.MX_led_off)%256))
        print("\nchecksum = ",checksum)
        self.string.append(self.str2list(checksum))
        
        print("Instruction Packet = " , self.string)
        
        tab = ' '
        self.output = tab.join(self.string)
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)
        