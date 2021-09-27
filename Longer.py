import serial
import numpy as np
import base64
import time
import os
import struct
from binascii import unhexlify
import codecs
import threading
import time
import sys
import glob

##################
byte = 'byte'
STRGLO=""
BOOL=True

flag = int('0xe9',16)
Addr = int('0x01',16)
leng_WJ = int('0x06',16)
leng_RJ = int('0x02',16)
leng_WID = int('0x04',16)
leng_RID = int('0x03',16)

W = int('0x57',16)
J = int('0x4a',16)
R = int('0x52',16)
I = int('0x49',16)
D = int('0x44',16)

PH_0 = int('0x00',16) # Placeholder 0x00

CW = int('0x01',16)
CCW = int('0x00',16)
ON = int('0x01',16)
OFF = int('0x00',16)

YZII25_48 = 186 # max. ml/min in 100rpm
YZII25_35 = 88 # min. ml/min in 100rpm

def flow2speed(tube, flow): # flow rate is for example 230ml/min 
    max_speed = 100 # rpm
    # this func only for head YZII25
    # datasheet from https://www.longerpump.com.cn/index.php/PeristalticPumpTechnique/show/22.htmlhttps://www.longerpump.com.cn/index.php/PeristalticPumpTechnique/show/22.html
    k = tube / max_speed
    speed = flow / k
    return speed

def speed2flow(tube, speed): # speed rmp
    max_speed = 100 # rpm
    # this func only for head YZII25
    k = tube / max_speed
    flow = k * speed
    return flow

def get_serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    serial_ports = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            serial_ports.append(port)
        except (OSError, serial.SerialException):
            pass
    return serial_ports


def OpenPort(portx, bps=1200, timeout=0):  # try to open local serial port with specific name
    ret = False
    try:
        # open serial port and return
        ser = serial.Serial(portx, bps, timeout=timeout,
                            parity=serial.PARITY_EVEN,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            xonxoff=True,
                            dsrdtr=False)
        # return boolen to show wether opened
        if (ser.is_open):
            ret = True
            threading.Thread(target=ReadData, args=(ser,)).start()
    except Exception as error:
        print("can't open the port", error)
    return ser, ret


def OpenPort_S():  # loop local serial ports and open them, return a list contains all the opened ports
    Ports_list_opened = []
    ports_list = get_serial_ports()
    print(f'Find {len(ports_list)} ports, try to open these ports')
    i = 0
    for port in ports_list:
        ser, ret = OpenPort(port, bps=1200, timeout=1)
        Ports_list_opened.append(ser)
        print(f'Find Port: {ser.name} now is open: {ret}')
        i = i + 1
    return Ports_list_opened


def ReadData(ser, wait_time=5):  # not so useful here. may have to run in threading
    global StrGlo, Bool
    # dead loop but can run in threading
    t = 0  # set a time_var to read data after sending
    while BOOL:
        if ser.in_waiting:
            STRGLO = ser.read(ser.in_waiting).decode("hex")
            print(STRGLO)
            time.sleep(1)
            t = t + 1
            if t > (wait_time - 1):
                return STRGLO
                break


def DColsePort(ser):  # kill the port
    global BOOL
    BOOL = False
    ser.close()

class PUMP:
    
    def __init__(self, serial, Addr, Speed, CW, ON):

        self.serial = serial  # serial class
        self.Speed = Speed  # int
        self.Addr = Addr  # int
        self.CW = CW
        self.ON = ON

    def PDU_FCX(self, PDU, Speed):
        fcx = 0
        if Speed == None:
            PDU_ = PDU[:]
            for byte in PDU_:
                fcx = fcx ^ byte
            return hex(fcx)

        if Speed < 256:
            PDU_ = PDU[:]
            PDU_.insert(5, Speed)
            for byte in PDU_:
                fcx = fcx ^ byte
            return hex(fcx)

        else:
            SPD_1 = '0' + hex(Speed)[2]
            SPD_2 = hex(Speed)[3:5]
            # print(f'SPD_1: {SPD_1}  SPD_2: {SPD_2}')

            SPD_1 = int(SPD_1, 16)
            SPD_2 = int(SPD_2, 16)
            PDU.copy().pop(5)
            PDU.pop(4)
            for byte in PDU:
                fcx = fcx ^ byte
            fcx = fcx ^ SPD_1 ^ SPD_2
            return hex(fcx)

    def get_speed_hex(self, Speed):

        if Speed > 255:
            SPD_1 = '0x0' + hex(Speed)[2]
            SPD_2 = '0x' + hex(Speed)[3:5]
            hex_spd = [SPD_1, SPD_2]

        elif Speed == 232:
            hex_spd = [hex(Speed), hex(PH_0).replace('x', 'x0')]
        elif Speed < 16:
            hex_spd = [hex(Speed).replace('x', 'x0')]
        else:
            hex_spd = hex(Speed)
        return hex_spd

    def get_full_command(self, *PDU, FCX, Speed, hex_spd):  # reading the command and get CRC-Byte

        bf_btye = ['0xe9']  # E9 as command_flag. No need to change but only works for LONGER PUMP
        if Speed == None and hex_spd == None:
            for byte in PDU:
                if len(hex(byte)) < 4:
                    hex_byte = hex(byte)
                    hex_byte = hex_byte.replace('0x', '0x0')
                    bf_btye.append(hex_byte)

                else:
                    hex_byte = hex(byte)
                    bf_btye.append(hex_byte)

            if len(FCX) < 4:  # special situation for CRC only in 1 Byte
                FCX = FCX.replace('0x', '0x0')

            bf_btye.append(FCX)

            command_str = str(bf_btye)
            command_str = command_str.replace('[', '').replace(']', '')
            command_str = command_str.replace('0x', '').replace(', ', '').replace("''", '').replace("'", '')
            return command_str

        if Speed > 255:  # speed_hex will be 3 byte after set more than 255, these 3 byte have to devided in to 2byte + 2byte
            for byte in PDU:
                if len(hex(byte)) < 4:
                    hex_byte = hex(byte)
                    hex_byte = hex_byte.replace('0x', '0x0')
                    bf_btye.append(hex_byte)

                else:
                    hex_byte = hex(byte)
                    bf_btye.append(hex_byte)

            if len(FCX) < 4:
                FCX = FCX.replace('0x', '0x0')

            bf_btye.append(FCX)
            bf_btye.insert(5, hex_spd)

            command_str = str(bf_btye)
            command_str = command_str.replace('[', '').replace(']', '')
            command_str = command_str.replace('0x', '').replace(', ', '').replace("''", '').replace("'", '')

        else:  # normal situation here
            for byte in PDU:
                if len(hex(byte)) < 4:
                    hex_byte = hex(byte)
                    hex_byte = hex_byte.replace('0x', '0x0')
                    bf_btye.append(hex_byte)

                else:
                    hex_byte = hex(byte)
                    bf_btye.append(hex_byte)

            if len(FCX) < 4:
                FCX = FCX.replace('0x', '0x0')

            bf_btye.append(FCX)
            bf_btye.insert(6, hex_spd)

            command_str = str(bf_btye)
            command_str = command_str.replace('[', '').replace(']', '')
            command_str = command_str.replace('0x', '').replace(', ', '').replace("''", '').replace("'", '')
        return command_str

    def Get(self, par):
        Addr = self.Addr
        if par == 'Status': # send read_status command and try to read response
            PDU = [Addr, leng_RJ, R, J, PH_0]
            FCX = self.PDU_FCX(PDU, Speed=None)
            cmd = self.get_full_command(*PDU, FCX=FCX, Speed=None, hex_spd = None)
            self.serial.write(unhexlify(cmd))
            time.sleep(0.5)
            data = str(self.serial.read_all())
            
            print(
            f'Direction: {self.CW}, Speed: {self.Speed}, Switch: {self.ON}'
            )
            
            if len(data) < 10:
                raise Warning('wrong address OR occupied by other users')
            return data

        elif par == 'Address':
            for i in range(10): # scan the address from 1 to 10
                PDU = [i + 1, leng_RID, R, I, D]
                FCX = self.PDU_FCX(PDU, Speed=None)
                cmd = self.get_full_command(*PDU, FCX=FCX, Speed=None, hex_spd=None)
                self.serial.write(unhexlify(cmd))
                time.sleep(0.5)
                data = str(self.serial.read_all())
                # print(data)
                if len(data) > 3: # if get response and then stop
                    break
                time.sleep(0.5)
            for i in range(10):  # for loop for doublecheck BECAUSE it can happen that no response at all
                self.serial.write(unhexlify(cmd))
                time.sleep(0.5)
                data = str(self.serial.read_all())
                if len(data) > 3:
                    print(f'the pump address is {data[8:10]} through serial {self.serial.name}')
                    break
                    return data[8:10]
        else:
            raise Warning('invalid command')
            return False

    def Change_Addr(self, new_addr):
        old_addr = self.Addr
        self.Addr = new_addr

        PDU = [old_addr, leng_WID, W, I, D, new_addr]
        FCX = self.PDU_FCX(PDU, Speed=None)
        cmd = self.get_full_command(*PDU, FCX=FCX, Speed=None, hex_spd=None)
        self.serial.write(unhexlify(cmd))
        print(f'pump address is setting from {old_addr} to {new_addr} through {self.serial.name}')
        return new_addr
        
    def Stop(self): # stop running condition
        Addr = self.Addr
        CW = self.CW
        Speed = int(self.Speed) * 10
        serial = self.serial
        
        PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
        hex_spd = self.get_speed_hex(Speed)
        FCX = self.PDU_FCX(PDU, Speed)
        cmd = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
        self.serial.write(unhexlify(cmd))
        
    def Set_speed(self, Speed, ON, CW, Duration):
        self.Speed = Speed
        self.ON = ON
        self.CW = CW
        
        Speed = int(Speed * 10)
        Addr = self.Addr
        OFF = int('0x00',16) # dont know why func cant read this hex_code from begnning so assign again here
        if Speed > 999 or Speed < 1:
            raise Warning('invalid Speed, valid Speed from 0.1 -> 99.9 rpm')
            return 0
        if ON == True and CW == True:
            PDU = [Addr, leng_WJ, W, J, PH_0, ON, CW]
        elif ON == False and CW == False:
            PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CCW]
        elif ON == True and CW == False:
            PDU = [Addr, leng_WJ, W, J, PH_0, ON, CCW]
        else:
            PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
        
        hex_spd = self.get_speed_hex(Speed)
        FCX = self.PDU_FCX(PDU, Speed)
        cmd = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
        
        if str(Duration) == 'Keep':
            #print(Duration == 'Keep')
            self.serial.write(unhexlify(cmd))
            # to do -> may set a timer here. or can run in another threading
            print(f'pump Nr. {Addr} is running through serial port {self.serial.name}')
            
        else: # PDU[-1] to control direction PDU[-2] to control switch.
              # setting different conditions manuelly to confirm the it ll work
            if PDU[-1] == True and PDU[-2] == False: 
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                ON = int('0x01',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, ON, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                
            elif PDU[-1] == True and PDU[-2] == True:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                OFF = int('0x00',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                
            elif PDU[-1] == False and PDU[-2] == False:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                ON = int('0x01',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, ON, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                
            elif PDU[-1] == False and PDU[-2] == True:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                OFF = int('0x00',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
            else:
                return 0
            
    def Set_flow(self, Tube, Flow, ON, CW, Duration):
        Speed_ = flow2speed(Tube, Flow)
        self.Speed = Speed_
        Speed = int(Speed_ * 10)
        print(Speed)
        Addr = self.Addr
        self.ON = ON
        self.CW = CW
        OFF = int('0x00',16) # dont know why func cant read this hex_code from begnning so assign again here
        if Flow > YZII25_48 or Flow < 1:
            raise Warning(f'invalid Flow setting, valid Speed from 1 -> {YZII25_48} ml/min')
            return 0

        if ON == True and CW == True:
            PDU = [Addr, leng_WJ, W, J, PH_0, ON, CW]
        elif ON == False and CW == False:
            PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CCW]
        elif ON == True and CW == False:
            PDU = [Addr, leng_WJ, W, J, PH_0, ON, CCW]
        else:
            PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]

        hex_spd = self.get_speed_hex(Speed)
        FCX = self.PDU_FCX(PDU, Speed)
        cmd = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)

        if Duration == 'Keep':
            self.serial.write(unhexlify(cmd))
            # to do -> timer
            print(f'pump Nr. {Addr} is running through serial port {self.serial.name}')
            return self.Speed
        else:
            Duration = int(Duration)
            print(Duration)
            if PDU[-1] == True and PDU[-2] == False:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                ON = int('0x01',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, ON, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                return self.Speed

            elif PDU[-1] == True and PDU[-2] == True:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                OFF = int('0x00',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                return self.Speed

            elif PDU[-1] == False and PDU[-2] == False:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                ON = int('0x01',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, ON, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                return self.Speed

            elif PDU[-1] == False and PDU[-2] == True:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                OFF = int('0x00',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_)) 
                return self.Speed
            else:
                return 0
            
    def Set_volume(self, Tube, Volume, CW, Speed):

        Addr = self.Addr
        self.ON = ON
        self.CW = CW
        self.Speed = Speed
        Flow = speed2flow(tube, speed)
        Duration = Volume / Flow
        Speed = int(Speed * 10)
        print(f'Flow rate now is setting to {Flow} ml/min')
        OFF = int('0x00',16) # dont know why func cant read this hex_code from begnning so assign again here
        if Speed > 999 or Speed < 1:
            raise Warning('invalid Speed, valid Speed from 0.1 -> 99.9 rpm')
            return 0

        if CW == True:
            PDU = [Addr, leng_WJ, W, J, PH_0, ON, CW]
        else:
            PDU = [Addr, leng_WJ, W, J, PH_0, ON, CCW]
   
        # forget why i add these 3 lines here. (better dont delete)
        hex_spd = self.get_speed_hex(Speed)
        FCX = self.PDU_FCX(PDU, Speed)
        cmd = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)

        if Duration < 0:
            #print(f'pump Nr. {Addr} is running through serial port {self.serial.name}')
            raise Warning ('unvalid parameter')
        else:
            if PDU[-1] == True:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                OFF = int('0x00',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                return self.Speed

            elif PDU[-1] == False:
                self.serial.write(unhexlify(cmd))
                time.sleep(Duration)
                OFF = int('0x00',16)
                PDU = [Addr, leng_WJ, W, J, PH_0, OFF, CW]
                FCX = self.PDU_FCX(PDU, Speed)
                cmd_ = self.get_full_command(*PDU, FCX=FCX, Speed=Speed, hex_spd=hex_spd)
                self.serial.write(unhexlify(cmd_))
                return self.Speed
            else:
                return 0