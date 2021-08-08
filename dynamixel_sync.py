'''

Operation type  : ID1 = 1 , ID2 = 2  
description     : 1."sync write" simultaneous contorl two motor to goal positon  
                  2."bulk read" simultaneous read two motor position

                  =============================================================

                  from dynamixel_sync import Dy_motor
                  import RPi.GPIO as GPIO
                    
                  try:
                      command = Dy_motor()
                     
                      command.moving_speed(100,100)  // ID1,ID2 CCW == 100
                      command.moving_speed(100,1124) // ID1 CCW == 100 , ID2 CW == 100
                                                       
                      
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

        self.transmit_delaytime = 0.05                                         # wait until data can be loaded
        self.rotate_delaytime = 1
        self.multi_transmit_delaytime = 0.014
    
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.direction_pin, GPIO.OUT)   
        
    def define_register(self):
        
        self.MX_id_reg = 3          
        self.MX_cw_reg = 6           
        self.MX_ccw_reg = 8 
        self.MX_multi_offset = 20                                           
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

        self.MX_broadcast_ID = 254  # 0xfe                                            
        self.MX_length_write = 10  # 0x0a
        self.MX_length_read = 9
        self.MX_inst_write = 131  # 0x83
        self.MX_inst_read = 146  # 0x92 
        self.MX_read_P1 = 0
        
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
    
    def dec_conversion(self, val):

        hex_number = hex(val)
        hex_list = list(hex_number[2:])
        
        flag = True
        
        while flag :
    
            if len(hex_list) == 4:
    
                flag = False

            elif len(hex_list) < 4:
        
                hex_list.insert(0,"0")

        bytes_L = hex_list[2]+hex_list[3]
        bytes_H = hex_list[0]+hex_list[1]
        
        return bytes_L , bytes_H
    
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

    def bulk_position(self, byte):
        
        final_1 = self.hex2int(byte[6]) + self.hex2int(byte[5])
        final_2 = self.hex2int(byte[14]) + self.hex2int(byte[13])
        
        return int(final_1,16) , int(final_2,16)
        
    def status_ID(self, byte):
        
        if byte[3] == 0:
            print("ID = " , byte[4])
            
        else:
            print("error")

    def status_movingspeed(self, byte):                                        
        
        final = self.hex2int(byte[5]) + self.hex2int(byte[4])
        
        print("\nmoving speed = ",int(final,16))

    def choose_specificposition(self,angle):
                
        if self.mode == 1:
            return angle
        
        elif self.mode == 2:
            
            if angle == 0:
                return 12290
            
            else:
                float_number = float(angle / 0.0879120879120879)                                        
                check_number = round(float_number, 3)
                dec = math.ceil(check_number)
                return (dec + 12288)

    def choose_goalposition(self,angle1,angle2):
        
        def angle2dec(angle):
            
            float_number = float(angle / 0.0879120879120879)                                        
            check_number = round(float_number, 3)
            dec = math.ceil(check_number)
            
            return dec
        
        if self.mode == 1:
            return angle1 , angle2
                    
        elif self.mode == 2:
            
            dec_1 = angle2dec(angle1)
            dec_2 = angle2dec(angle2)   
            
            (angle_1,angle_2) = self.read_position()
            
            return (dec_1 + angle_1) , (dec_2 + angle_2)
        
class Dy_motor(Initialize,Operation):
    
    def __init__(self):
                
        self.id1 = 1 
        self.id2 = 2                                                     
        self.string = ["ff","ff"]
        self.mode = 2   
        # wheel_mode(join_mode) = 1 , multi_mode = 2                                  
                                             
        self.raspi_set()
        self.define_register()
        
    def direction(self, d):                                                    
        
        GPIO.output(self.direction_pin, d)      

    def cw_reg(self,cw):
        
        del self.string[2:]

        self.string.append(self.dec2hex(self.MX_broadcast_ID)) 
        self.string.append("0"+ (list(hex(self.MX_length_write)[2:])[0]))
        self.string.append(self.dec2hex(self.MX_inst_write))         
        self.string.append("0"+ str(self.MX_cw_reg))   
        self.string.append("0"+ str(self.MX_length_of_data_2))
        self.string.append(self.ID_hex(self.id1))
        
        (cw1_L,cw1_H) = self.dec_conversion(cw)
        self.string.insert(8,cw1_L)                                          
        self.string.insert(9,cw1_H)   
        
        self.string.append(self.ID_hex(self.id2))

        (cw2_L,cw2_H) = self.dec_conversion(cw)
        self.string.insert(11,cw2_L)                                          
        self.string.insert(12,cw2_H) 
        
        checksum = hex(255-((self.MX_broadcast_ID + self.MX_length_write + self.MX_inst_write + self.MX_cw_reg 
                              + self.MX_length_of_data_2 + self.id1
          + self.id2 + int(cw1_L,16) + int(cw1_H,16) + int(cw2_L,16) + int(cw2_H,16))%256))
        
        self.string.append(self.str2list(checksum))
                
        tab = ' '
        self.output = tab.join(self.string)                                 
        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.transmit_delaytime) 
        self.direction(self.rx)

    def ccw_reg(self,ccw):
        
        del self.string[2:]

        self.string.append(self.dec2hex(self.MX_broadcast_ID)) 
        self.string.append("0"+ (list(hex(self.MX_length_write)[2:])[0]))
        self.string.append(self.dec2hex(self.MX_inst_write))         
        self.string.append("0"+ str(self.MX_ccw_reg))   
        self.string.append("0"+ str(self.MX_length_of_data_2))
        self.string.append(self.ID_hex(self.id1))
        
        (ccw1_L,ccw1_H) = self.dec_conversion(ccw)
        self.string.insert(8,ccw1_L)                                          
        self.string.insert(9,ccw1_H)   
        
        self.string.append(self.ID_hex(self.id2))

        (ccw2_L,ccw2_H) = self.dec_conversion(ccw)
        self.string.insert(11,ccw2_L)                                          
        self.string.insert(12,ccw2_H) 
        
        checksum = hex(255-((self.MX_broadcast_ID + self.MX_length_write + self.MX_inst_write + self.MX_ccw_reg 
                              + self.MX_length_of_data_2 + self.id1
          + self.id2 + int(ccw1_L,16) + int(ccw1_H,16) + int(ccw2_L,16) + int(ccw2_H,16))%256))
        
        self.string.append(self.str2list(checksum))
                
        tab = ' '
        self.output = tab.join(self.string)                                 
        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.transmit_delaytime) 
        self.direction(self.rx)
                        
    def wheel_mode(self):
        
        self.mode = 1
        self.cw_reg(0)
        time.sleep(self.transmit_delaytime)
        self.ccw_reg(0)
        print("wheel mode")
        
    def join_mode(self):
        
        self.mode = 1
        self.cw_reg(0)
        time.sleep(self.transmit_delaytime)
        self.ccw_reg(4095)
        print("join mode")
        
    def multi_mode(self):

        self.mode = 2
        self.cw_reg(4095)
        time.sleep(self.rotate_delaytime)
        self.ccw_reg(4095)
        print("multi mode")
        # time.sleep(self.transmit_delaytime)
        # self.multi_offset(self.id1)
        # time.sleep(self.transmit_delaytime)
        # self.multi_offset(self.id2)
        
    def multi_offset(self,ID):
        
        del self.string[2:]
        length = 5
        
        val = 12288
        
        self.string.append(self.ID_hex(ID))
        self.string.append("0"+ str(length))
        self.string.append("0"+ str(self.MX_write))
        self.string.append(self.dec2hex(self.MX_multi_offset))
        
        (byte_L,byte_H) = self.dec_conversion(val)
        self.string.insert(6,byte_L)                                          
        self.string.insert(7,byte_H) 

        checksum = hex(255-((ID + length + self.MX_write + self.MX_multi_offset + int(byte_L,16) + int(byte_H,16))%256))
        self.string.append(self.str2list(checksum))
                         
        tab = ' '
        self.output = tab.join(self.string)                                    
        
        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.transmit_delaytime)
        self.direction(self.rx)
        print("multi turn offest = 12288")
        
    def goal_position(self, angle1, angle2):                                                             
                
        (angle1,angle2) = self.choose_goalposition(angle1,angle2)
        
        del self.string[2:]                                                    
                
        self.string.append(self.dec2hex(self.MX_broadcast_ID))                               
        self.string.append("0"+ (list(hex(self.MX_length_write)[2:])[0]))                                   
        self.string.append(self.dec2hex(self.MX_inst_write))                            
        self.string.append(self.dec2hex(self.MX_goalposition_reg))            
        self.string.append("0"+ str(self.MX_length_of_data_2))
        self.string.append(self.ID_hex(self.id1))
        
        if self.mode == 1:
            (angle1_L,angle1_H) = self.angle_conversion(angle1)
            self.string.insert(8,angle1_L)                                           
            self.string.insert(9,angle1_H)                                          
        
        elif self.mode == 2:
            (angle1_L,angle1_H) = self.dec_conversion(angle1)
            self.string.insert(8,angle1_L)                                          
            self.string.insert(9,angle1_H)
            
        self.string.append(self.ID_hex(self.id2)) 
        
        if self.mode == 1:
            (angle2_L,angle2_H) = self.angle_conversion(angle2)
            self.string.insert(11,angle2_L)                                           
            self.string.insert(12,angle2_H)                                          
        
        elif self.mode == 2:
            (angle2_L,angle2_H) = self.dec_conversion(angle2)
            self.string.insert(11,angle2_L)                                          
            self.string.insert(12,angle2_H)
                
        checksum = hex(255-((self.MX_broadcast_ID + self.MX_length_write + self.MX_inst_write + self.MX_goalposition_reg 
                              + self.MX_length_of_data_2 + self.id1
          + self.id2 + int(angle1_L,16) + int(angle1_H,16) + int(angle2_L,16) + int(angle2_H,16))%256))
        
        
        self.string.append(self.str2list(checksum))
                
        tab = ' '
        self.output = tab.join(self.string)                                 
        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.rotate_delaytime) 
        self.direction(self.rx)   
        
    def specific_position(self, angle1, angle2):

        angle1 = self.choose_specificposition(angle1)
        angle2 = self.choose_specificposition(angle2)
        
        del self.string[2:]                                                    
                
        self.string.append(self.dec2hex(self.MX_broadcast_ID))                               
        self.string.append("0"+ (list(hex(self.MX_length_write)[2:])[0]))                                   
        self.string.append(self.dec2hex(self.MX_inst_write))                            
        self.string.append(self.dec2hex(self.MX_goalposition_reg))            
        self.string.append("0"+ str(self.MX_length_of_data_2))
        self.string.append(self.ID_hex(self.id1))
        
        if self.mode == 1:
            (angle1_L,angle1_H) = self.angle_conversion(angle1)
            self.string.insert(8,angle1_L)                                           
            self.string.insert(9,angle1_H)                                          
        
        elif self.mode == 2:
            (angle1_L,angle1_H) = self.dec_conversion(angle1)
            self.string.insert(8,angle1_L)                                          
            self.string.insert(9,angle1_H)
            
        self.string.append(self.ID_hex(self.id2)) 
        
        if self.mode == 1:
            (angle2_L,angle2_H) = self.angle_conversion(angle2)
            self.string.insert(11,angle2_L)                                           
            self.string.insert(12,angle2_H)                                          
        
        elif self.mode == 2:
            (angle2_L,angle2_H) = self.dec_conversion(angle2)
            self.string.insert(11,angle2_L)                                          
            self.string.insert(12,angle2_H)
                
        checksum = hex(255-((self.MX_broadcast_ID + self.MX_length_write + self.MX_inst_write + self.MX_goalposition_reg 
                              + self.MX_length_of_data_2 + self.id1
          + self.id2 + int(angle1_L,16) + int(angle1_H,16) + int(angle2_L,16) + int(angle2_H,16))%256))
        
        
        self.string.append(self.str2list(checksum))
                
        tab = ' '
        self.output = tab.join(self.string)                                 
        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.rotate_delaytime) 
        self.direction(self.rx)  
        
    def read_position(self):                                                             
                
        del self.string[2:]                                                    
                
        self.string.append(self.dec2hex(self.MX_broadcast_ID))                               
        self.string.append("0"+ str(self.MX_length_read))                                   
        self.string.append(self.dec2hex(self.MX_inst_read)) 
        self.string.append("0"+ str(self.MX_read_P1)) 
        self.string.append("0"+ str(self.MX_length_of_data_2))
        self.string.append(self.ID_hex(self.id1))
        self.string.append(self.dec2hex(self.MX_present_position_reg))
        self.string.append("0"+ str(self.MX_length_of_data_2))
        self.string.append(self.ID_hex(self.id2))
        self.string.append(self.dec2hex(self.MX_present_position_reg))

        checksum = hex(255-((self.MX_broadcast_ID + self.MX_length_read + self.MX_inst_read + self.MX_read_P1 + self.MX_length_of_data_2
                            + self.id1 + self.MX_present_position_reg + self.MX_length_of_data_2 + self.id2 + self.MX_present_position_reg)%256))
        
        self.string.append(self.str2list(checksum))
                
        tab = ' '
        self.output = tab.join(self.string)
        
        self.direction(self.tx)
        self.ser.write(bytearray.fromhex(self.output))
        time.sleep(self.multi_transmit_delaytime)
        self.direction(self.rx)
        reply = self.ser.read(20)
        (angle_1,angle_2) = self.bulk_position(reply)
        print(angle_1 , angle_2)
        return angle_1 , angle_2
          
    def moving_speed(self,speed1,speed2):                                             
              
        del self.string[2:]
        
        self.string.append(self.dec2hex(self.MX_broadcast_ID))
        self.string.append("0"+ (list(hex(self.MX_length_write)[2:])[0])) 
        self.string.append(self.dec2hex(self.MX_inst_write))
        self.string.append(self.dec2hex(self.MX_movingspeed_reg))
        self.string.append("0"+ str(self.MX_length_of_data_2))
        self.string.append(self.ID_hex(self.id1))     
        
        (speed1_L,speed1_H) = self.dec_conversion(speed1)
        self.string.insert(8,speed1_L)                                          
        self.string.insert(9,speed1_H)                                          

        self.string.append(self.ID_hex(self.id2))         
        (speed2_L,speed2_H) = self.dec_conversion(speed2)
        self.string.insert(11,speed2_L)                                          
        self.string.insert(12,speed2_H)
        
        checksum = hex(255-((self.MX_broadcast_ID + self.MX_length_write + self.MX_inst_write + self.MX_movingspeed_reg 
                              + self.MX_length_of_data_2 + self.id1
          + self.id2 + int(speed1_L,16) + int(speed1_H,16) + int(speed2_L,16) + int(speed2_H,16))%256))
        
        self.string.append(self.str2list(checksum))
                
        tab = ' '
        self.output = tab.join(self.string)                                    
        print(self.output)
        self.direction(self.tx)                                                
        self.ser.write(bytearray.fromhex(self.output))                         
        time.sleep(self.transmit_delaytime) 
        self.direction(self.rx)      
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        