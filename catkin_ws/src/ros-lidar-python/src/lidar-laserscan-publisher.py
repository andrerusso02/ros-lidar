import time
from motor import Motor

if __name__ == '__main__':

    motor  = Motor(115200)
    time.sleep(2)
    print(motor.set_motor_speed(12.0))
    print(motor.start())
    time.sleep(5)
    print(motor.stop())

    # ports = serial.tools.list_ports.comports(include_links=False)
    # for port in ports:
    #     print(port)
    #     print(port.name)
    #     print(port.description)
    #     print(port.hwid)
    #     print(port.vid)
    #     print(port.pid)
    #     print(port.serial_number)
    #     print(port.location)
    #     print(port.manufacturer)
    #     print(port.product)
    #     print(port.interface)
    #     print()


