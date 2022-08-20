import serial
import serial.tools.list_ports
import time
from enum import Enum
from numpy import float32

REV_COMPLETED_FLAG = 0x01
SET_MIRROR_SPEED = 0x02
START_COMMAND = 0x03
STOP_COMMAND = 0x04
ERROR_MOTOR = 0x05
ERROR_CHECKSUM = 0x06
SUCCESS_COMMAND = 0x07
ERROR_UNKNOWN_COMMAND = 0x08
ERROR_COMMAND_ALREADY_EFFECTIVE = 0x09
ERROR_NO_VELOCITY = 0x0A
ERROR_TIMEOUT = 0x0B
SET_OFFSET_ZERO = 0x0C

WHOAMI = 0xFF
ID = 0xFE

class Motor:

    class Status(Enum):
        REVOLUTION_COMPLETED = 1
        MOTOR_BLOCKED = 2
        ERROR_READING_STATUS = 3


    def __init__(self, port=None):
        self.__default_read_timeout = 1.0 # seconds
        self.__baudrate = 115200
        if port is None:
            self.__port = self.find_port_and_connect()
            
        else:
            self.__port = port
            self.serial = serial.Serial(port, self.__baudrate, timeout=self.__default_read_timeout)
            time.sleep(2) # wait for Arduino to reboot
    
    def find_port_and_connect(self):
        peripherials = serial.tools.list_ports.comports(include_links=False)
        for peripherial in peripherials:
            if peripherial.description == "USB2.0-Serial":
                port = "/dev/" + peripherial.name
                self.serial = serial.Serial(port, self.__baudrate, timeout=self.__default_read_timeout)
                time.sleep(2) # wait for Arduino to reboot
                self.serial.write(WHOAMI.to_bytes(1, 'little'))
                res = self.serial.read(1)
                if res == ID.to_bytes(1, 'little'): # motor Arduino found !
                    return port
                else:
                    self.serial.close()
        raise Exception("Motor driver not found")

    """set the motor speed in rad/s"""
    def set_motor_speed(self, speed):
        self.__send_command_float32(SET_MIRROR_SPEED, speed)
    
    """set the motor zero position in rad"""
    def set_motor_zero(self, zero_pos):
        self.__send_command_float32(SET_OFFSET_ZERO, zero_pos)
    
    def __send_command_float32(self, adress, value):
        value_bytes = float32(value).tobytes()
        checksum = sum(list(value_bytes)).to_bytes(2, 'little')[0]

        self.serial.write(adress.to_bytes(1, 'big'))
        for b in value_bytes:
            self.serial.write(b.to_bytes(1, 'little'))
        self.serial.write(checksum.to_bytes(1, 'little'))

        res = self.serial.read(1)
        if not res == SUCCESS_COMMAND.to_bytes(1, 'big'):
            raise Exception("Error sending command, error code: " + str(res))

    def start(self):
        self.serial.write(START_COMMAND.to_bytes(1, 'big'))
        self.serial.timeout = 10.0 # seconds
        res = self.serial.read(1)
        if res != SUCCESS_COMMAND.to_bytes(1, 'big'):
            raise Exception("Error starting motor, error code: " + str(res))
        self.serial.timeout = self.__default_read_timeout
        

    def stop(self):
        self.serial.write(STOP_COMMAND.to_bytes(1, 'big'))
        res =  self.serial.read(1)
        if res != SUCCESS_COMMAND.to_bytes(1, 'big'):
            raise Exception("Error stopping motor, error code: " + str(res))
    
    def wait_for_incoming_message(self):
        self.serial.timeout = None
        ret = self.serial.read(1)
        if(ret == REV_COMPLETED_FLAG.to_bytes(1, 'big')):
            return self.Status.REVOLUTION_COMPLETED
        elif(ret == ERROR_MOTOR.to_bytes(1, 'big')):
            return self.Status.MOTOR_BLOCKED
        else:
            return self.Status.ERROR_READING_STATUS
