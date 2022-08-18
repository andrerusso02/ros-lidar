import serial
import serial.tools.list_ports
import time
from enum import Enum

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

        integer  = int(speed)
        decimal = int((speed - integer)*0x100)
        checksum = (integer + decimal).to_bytes(1, 'little') # checksum -> less significant byte
        self.serial.write(SET_MIRROR_SPEED.to_bytes(1, 'big'))
        self.serial.write(integer.to_bytes(1, 'big'))
        self.serial.write(decimal.to_bytes(1, 'big'))
        self.serial.write(checksum)

        res = self.serial.read(1)
        if res == SUCCESS_COMMAND.to_bytes(1, 'big'):
            return integer + (decimal / 0x100)
        else:
            return -1

    def start(self):
        self.serial.write(START_COMMAND.to_bytes(1, 'big'))
        self.serial.timeout = 10.0 # seconds
        res = self.serial.read(1)
        self.serial.timeout = self.__default_read_timeout
        return res

    def stop(self):
        self.serial.write(STOP_COMMAND.to_bytes(1, 'big'))
        return self.serial.read(1)
    
    def wait_for_incoming_message(self):
        self.serial.timeout = None
        ret = self.serial.read(1)
        if(ret == REV_COMPLETED_FLAG.to_bytes(1, 'big')):
            return self.Status.REVOLUTION_COMPLETED
        elif(ret == ERROR_MOTOR.to_bytes(1, 'big')):
            return self.Status.MOTOR_BLOCKED
        else:
            return self.Status.ERROR_READING_STATUS


if __name__ == '__main__':

    motor  = Motor()
    print(motor.set_motor_speed(12.0))
    print(motor.start())
    time.sleep(5)
    print(motor.stop())