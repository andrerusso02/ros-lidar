from curses import baudrate
import serial

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

class lidar:

    def __init__(self, port, baudrate):
        self.__default_read_timeout = 1.0 # seconds
        self.__serial = serial.Serial(port, baudrate, timeout=self.__default_read_timeout)
    
    """set the motor speed in rad/s""" #todo prendre en compte les décimales
    def set_motor_speed(self, speed):
        integer  = int(speed)
        decimal = int((speed - integer)*0x100)
        checksum = (integer + decimal).to_bytes(1, 'little') # checksum -> less significant byte
        self.__serial.write(SET_MIRROR_SPEED.to_bytes(1, 'big'))
        self.__serial.write(integer.to_bytes(1, 'big'))
        self.__serial.write(decimal.to_bytes(1, 'big'))
        self.__serial.write(checksum)

        res = self.__serial.read(1)
        print(res)
        if res == SUCCESS_COMMAND:
            return integer + (decimal / 0x100)
        else:
            return -1

    def start_motor(self):
        self.__serial.write(START_COMMAND.to_bytes(1, 'big'))
        self.__serial.timeout = 5.0 # seconds
        res = self.__serial.read(1)
        self.__serial.timeout = self.__default_read_timeout
        self.__serial.close()
        return res

    def stop_motor(self):
        self.__serial.write(STOP_COMMAND.to_bytes(1, 'big'))
        return self.__serial.read(1)


if __name__ == '__main__':
    lidar  = lidar('/dev/ttyUSB0', 115200)
    # print(lidar.set_motor_speed(2.0))
    print(lidar.start_motor())
