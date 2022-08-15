import time
from lidar import Lidar

if __name__ == '__main__':
    lidar  = Lidar('/dev/ttyUSB0', 115200)
    time.sleep(2)
    print(lidar.set_motor_speed(12.0))
    print(lidar.start_motor())
    time.sleep(10)
    print(lidar.stop_motor())
