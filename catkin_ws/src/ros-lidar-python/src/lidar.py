from tfminis import TFminiS
from motor import Motor
import threading
from queue import Queue

class Lidar:
    def __init__(self, port_tfmini=None, port_motor=None):
        self.__tfmini = TFminiS(port=port_tfmini)
        self.__motor = Motor(port=port_motor)
        self.__distances = Queue()
        self.__thread_store_distance = threading.Thread(target=self.__thread_store_distance_function)
    
    def start(self, speed):
        print("Motor: set speed: " + str(self.__motor.set_motor_speed(speed)))
        print("Motor: start: " + str(self.__motor.start()))

        self.__skip_revolution(2) # todo remove if possible
        
        self.__thread_store_distance.start()

    def __thread_store_distance_function(self):
        while True:
            self.__distances.put(self.__tfmini.read_distance())
    
    def get_distances_set(self): # waitq for revolution completed
        status = self.__motor.wait_for_incoming_message()
        if status == Motor.Status.REVOLUTION_COMPLETED:
            distances  = []
            s = self.__distances.qsize()
            for i in range(0, s):
                distances.append(self.__distances.get())
                self.__distances.task_done()
            return distances
        elif status == Motor.Status.MOTOR_BLOCKED:
            print("motor blocked")
            self.__restart()
            return self.get_distances_set()

    def __restart(self):
        self.__motor.start()
        while self.__motor.wait_for_incoming_message() != Motor.Status.REVOLUTION_COMPLETED:
            self.__motor.start()
            pass
        self.__skip_revolution(1) # todo remove if possible
        self.__distances = Queue()
    
    def __skip_revolution(self, nb_to_skip):
        skip = 0
        while skip < nb_to_skip:
            while self.__motor.wait_for_incoming_message() != Motor.Status.REVOLUTION_COMPLETED:
                pass
            skip += 1
        self.__tfmini.clear_buffer()
        



            

