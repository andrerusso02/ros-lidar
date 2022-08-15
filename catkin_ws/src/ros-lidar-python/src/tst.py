import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)  # open serial port
ser.write(b'hello')
ser.close()  # close port