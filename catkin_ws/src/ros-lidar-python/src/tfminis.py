import time
import serial

class TFminiS:

    def __init__(self, port, baudrate):
        self.__serial = serial.Serial(port, baudrate)
    
    def read_distance(self):

        ret = bytes(7)

        start_1 = self.__serial.read(1)
        start_2 = self.__serial.read(1)
        
        while not(start_1 == b'\x59' and start_2 == b'\x59'):
            start_1 = start_2
            start_2 = self.__serial.read(1)
        
        ret = self.__serial.read(7)

        #print([int(i) for i in ret])

        checksum = 2 * 0x59
        for i in range(0, 6):
            checksum += int(ret[i])
        
        if ret[6] == checksum.to_bytes(2, 'little')[0]:
            return int(ret[0]) + int(ret[1])*256 
        else:
            return -1

if __name__ == '__main__':
    tfmini = TFminiS('/dev/ttyUSB0', 115200)
    time.sleep(2)
    cnt_success = 0
    cnt_fail = 0
    last = time.time()
    while(True):
        
        if tfmini.read_distance() == -1:
            cnt_fail += 1
        else:
            cnt_success += 1
        now = time.time()
        freq = 1.0/(now -last)
        print('success: {}, fail: {}, freq: {}'.format(cnt_success, cnt_fail, freq))
        last = time.time()
        
