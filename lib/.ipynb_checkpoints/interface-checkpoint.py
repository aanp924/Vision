
import smbus
import time

EXP_I2CADDR = 0x2B
"""
Using RaspbotV2 lib
Hardware Interfce Layer - Controlling Expansion board via I2C
"""
class Visionbot():

    def get_i2c_device(self, address, i2c_bus):
        self._addr = address
        if i2c_bus is None:
            return smbus.SMBus(1)
        else:
            return smbus.SMBus(i2c_bus)

    def __init__(self):
        # Create I2C device.
        self._device = self.get_i2c_device(EXP_I2CADDR, 1)

    def write_u8(self, reg, data):
        try:
            self._device.write_byte_data(self._addr, reg, data)
        except:
            print ('write_u8 I2C error')

    def write_reg(self, reg):
        try:
            self._device.write_byte(self._addr, reg)
        except:
            print ('write_u8 I2C error')

    def write_array(self, reg, data):
        try:
            # self._device.write_block_data(self._addr, reg, data)
            self._device.write_i2c_block_data(self._addr, reg, data)
        except:
            print ('write_array I2C error')

    def read_data_byte(self):
        try:
            buf = self._device.write_byte(self._addr)
            return buf
        except:
            print ('read_u8 I2C error')

    def read_data_array(self,reg,len):
        try:
            buf = self._device.read_i2c_block_data(self._addr,reg,len)
            return buf
        except:
            print ('read_u8 I2C error')

    def Ctrl_Drive(self, motor_id, motor_dir,motor_speed):
        try:
            if(motor_dir !=1)and(motor_dir != 0):  
                motor_dir = 0
            if(motor_speed>255):
                motor_speed = 255
            elif(motor_speed<0):
                motor_speed = 0

            reg = 0x01
            data = [motor_id, motor_dir, motor_speed]
            self.write_array(reg, data)
        except:
            print ('Ctrl_Drive I2C error')


    def Ctrl_Motor(self, motor_id, motor_speed):
        try:

            if(motor_speed>255):
                motor_speed = 255
            if(motor_speed<-255):
                motor_speed = -255
            if(motor_speed < 0 and motor_speed >= -255): 
                motor_dir = 1
            else:motor_dir = 0
            reg = 0x01
            data = [motor_id, motor_dir, abs(motor_speed)]
            self.write_array(reg, data)
        except:
            print ('Ctrl_Motor I2C error')


    def Ctrl_Ultra_Sensor(self, state):
        try:
            reg = 0x07
            data = [state]
            if state < 0:
                state = 0
            elif state > 1:
                state = 1
            self.write_array(reg, data)
        except:
            print ('Ctrl_UltraSonic Sensor I2C error')
    
    def Ctrl_Buzzer(self, state):
        try:
            reg = 0x06
            data = [state]
            if state < 0:
                state = 0
            elif state > 1:
                state = 1
            self.write_array(reg, data)
        except:
            print ('Ctrl_Buzzer I2C error')

def test():
    vBot = Visionbot()
   
    vBot.Ctrl_Ultra_Sensor(1)#open
    time.sleep(1) 
    diss_H = vBot.read_data_array(0x1b,1)[0]
    diss_L = vBot.read_data_array(0x1a,1)[0]
    dis = diss_H<< 8 | diss_L 
    print(dis+"mm") 
    vBot.Ctrl_Ultra_Sensor(0)#close
     
    vBot.Ctrl_Drive(0,0,150) #L1
    vBot.Ctrl_Drive(1,0,150) #L2
    vBot.Ctrl_Drive(2,0,150) #R1
    vBot.Ctrl_Drive(3,0,150) #R2
    time.sleep(1)
    vBot.Ctrl_Drive(0,1,50) #L1
    time.sleep(1)
    vBot.Ctrl_Drive(0,0,0) #L1
    vBot.Ctrl_Drive(1,0,0) #L2
    vBot.Ctrl_Drive(2,0,0) #R1
