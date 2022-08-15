import serial
import serial.tools.list_ports
import time

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

    def __init__(self, baudrate, port='/dev/ttyUSB0', id=0xFE, connexion_mode='id'):
        self.__default_read_timeout = 1.0 # seconds
        self.__baudrate = baudrate
        if connexion_mode == 'serial':
            self.__serial = serial.Serial(port, baudrate, timeout=self.__default_read_timeout)
        else:
            self.find_port_and_connect(id)
    
    def find_port_and_connect(self, id):
        peripherials = serial.tools.list_ports.comports(include_links=False)
        for peripherial in peripherials:
            if peripherial.description == "USB2.0-Serial":
                port = "/dev/" + peripherial.name
                self.__serial = serial.Serial(port, self.__baudrate, timeout=self.__default_read_timeout)
                time.sleep(2)
                self.__serial.write(WHOAMI.to_bytes(1, 'little'))
                res = self.__serial.read(1)
                if res == id.to_bytes(1, 'little'): # motor Arduino found !
                    return
                else:
                    self.__serial.close()
        raise NoPortMatchingIdError

    """set the motor speed in rad/s"""
    def set_motor_speed(self, speed):
        integer  = int(speed)
        decimal = int((speed - integer)*0x100)
        checksum = (integer + decimal).to_bytes(1, 'little') # checksum -> less significant byte
        self.__serial.write(SET_MIRROR_SPEED.to_bytes(1, 'big'))
        self.__serial.write(integer.to_bytes(1, 'big'))
        self.__serial.write(decimal.to_bytes(1, 'big'))
        self.__serial.write(checksum)

        res = self.__serial.read(1)
        if res == SUCCESS_COMMAND.to_bytes(1, 'big'):
            return integer + (decimal / 0x100)
        else:
            return -1

    def start(self):
        self.__serial.write(START_COMMAND.to_bytes(1, 'big'))
        self.__serial.timeout = 5.0 # seconds
        res = self.__serial.read(1)
        self.__serial.timeout = self.__default_read_timeout
        return res

    def stop(self):
        self.__serial.write(STOP_COMMAND.to_bytes(1, 'big'))
        return self.__serial.read(1)


class NoPortMatchingIdError(Exception):
    pass