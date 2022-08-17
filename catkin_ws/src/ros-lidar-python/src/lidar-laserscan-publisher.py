from lidar import Lidar
import time

if __name__ == '__main__':

    lidar = Lidar()
    lidar.start(3.14159*4)
    print("started")
    t_last = time.time()
    while True:
        l = len(lidar.get_distances_set())
        t = time.time()
        print(str(l) + '\t' + str(t-t_last))
        t_last = t


