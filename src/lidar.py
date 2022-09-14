from tfminis import TFminiS
from motor import Motor
import threading
from queue import Queue
import rospy

class Lidar:
    def __init__(self, port_tfmini=None, port_motor=None):
        self.__distance_mirror = 0.02 # todo find more accurate value
        self.__tfmini = TFminiS(port=port_tfmini)
        self.__motor = Motor(port=port_motor)
        self.__stop_event = threading.Event()
        self.__measures_queue = Queue()
        self.__thread_store_measures = threading.Thread(target=self.__thread_store_measures_function)
    
    def start(self, speed, zero_pos):
        self.__motor.set_motor_speed(speed)
        self.__motor.set_motor_zero(zero_pos)
        self.__motor.start()
        self.__skip_revolution(2) # todo remove if possible
        self.__thread_store_measures.start()

    def __thread_store_measures_function(self):
        while not self.__stop_event.is_set():
            self.__measures_queue.put(self.__tfmini.read_measure())
    
    def get_measures_set(self): # wait for revolution completed
        status = self.__motor.wait_for_incoming_message()
        if status == Motor.Status.REVOLUTION_COMPLETED:
            measures  = []
            s = self.__measures_queue.qsize()
            for i in range(0, s):
                measure = self.__measures_queue.get()
                measure.distance = measure.distance / 100.0 - self.__distance_mirror
                measures.append(measure)
                self.__measures_queue.task_done()
            return measures
        elif status == Motor.Status.MOTOR_BLOCKED:
            rospy.logwarn("motor blocked")
            self.__restart()
            return self.get_measures_set()

    def __restart(self):
        self.__motor.start()
        while self.__motor.wait_for_incoming_message() != Motor.Status.REVOLUTION_COMPLETED:
            self.__motor.start()
            pass
        self.__skip_revolution(1) # todo remove if possible
        self.__measures_queue = Queue()
    
    def __skip_revolution(self, nb_to_skip):
        skip = 0
        while skip < nb_to_skip:
            while self.__motor.wait_for_incoming_message() != Motor.Status.REVOLUTION_COMPLETED:
                pass
            skip += 1
        self.__tfmini.serial.flushInput()
    
    def stop(self):

        self.__stop_event.set()
        self.__thread_store_measures.join()

        self.__motor.stop()

        self.__motor.serial.cancel_read()

        self.__tfmini.serial.close()
        self.__motor.serial.close()

        rospy.loginfo("Threads stopped and serial ports closed properly") 
        



            

