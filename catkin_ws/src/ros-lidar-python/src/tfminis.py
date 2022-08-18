import time
import serial
import serial.tools.list_ports

class TFminiS:

    def __init__(self, port=None):
        self.__baudrate = 115200
        self.__port = self.__find_port() if port is None else port
        self.__timeout = 1.0 # seconds
        self.__serial = serial.Serial(self.__port, self.__baudrate, timeout=self.__timeout)

    def __find_port(self):
        peripherials = serial.tools.list_ports.comports(include_links=False)
        for peripherial in peripherials:
            if peripherial.description == "USB Serial":
                return "/dev/" + peripherial.name
        raise Exception("TF-Mini-S not found")
    
    def clear_buffer(self):
        self.__serial.flushInput()
    
    def read_distance(self):

        ret = bytes(7)

        start_1 = self.__serial.read(1)
        start_2 = self.__serial.read(1)
        
        while not(start_1 == b'\x59' and start_2 == b'\x59'):
            start_1 = start_2
            start_2 = self.__serial.read(1)
        
        ret = self.__serial.read(7)

        if start_1 == b'' and start_2 == b'' and ret.count(b'') == 7:
            raise Exception("No data received from TF-Mini-S")

        checksum = 2 * 0x59
        for i in range(0, 6):
            checksum += int(ret[i])
        
        if ret[6] == checksum.to_bytes(2, 'little')[0]:
            return int(ret[0]) + int(ret[1])*256 
        else:
            return -1


if __name__ == '__main__':
    tfmini = TFminiS()
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