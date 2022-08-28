#!/usr/bin/env python3

import sys
from lidar import Lidar
import time
from sensor_msgs.msg import LaserScan
import rospy
import serial
from rosgraph import is_master_online

# scan : distances and intensities for a complete revolution
def build_laserscan_msg(scan, duration):
    t = time.time()
    scan.reverse() # LaserScan wants measurements to be counter-clockwise
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "lidar" #todo vérifier que c'est bien ça
    msg.angle_min = 0.0
    msg.angle_max = 3.14159265359*2
    msg.angle_increment = 3.14159265359*2/len(scan)
    msg.time_increment = duration.to_sec()/len(scan)
    msg.scan_time = duration.to_sec()
    msg.range_min = 0.08
    msg.range_max = 12.0
    msg.ranges = [measure.distance for measure in scan]
    msg.intensities = [measure.intensity for measure in scan]
    return msg


if __name__ == '__main__':

    if not is_master_online():
        raise Exception("ROS master not found")

    # get speed from command line (rad/s)
    if len(rospy.myargv()) != 3:
        speed = 3.14159*3 # default
        zero_pos = 1.68
    else:
        speed = float(rospy.myargv()[1])
        zero_pos = float(rospy.myargv()[1])
    
    # get lidar sensor port from parameter server
    port_lidar_sensor = rospy.get_param('~port_lidar_sensor', None)
    # get motor port from parameter server
    port_motor = rospy.get_param('~port_motor', None)



    lidar = Lidar(port_lidar_sensor, port_motor)
    lidar.start(speed, zero_pos)

    print("LiDAR started")

    rospy.init_node('lidar_laserscan_publisher', anonymous=True)

    pub_laserscan = rospy.Publisher('lidar_laserscan', LaserScan, queue_size=10)

    rospy.on_shutdown(lidar.stop)

    t_last = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            distances = lidar.get_measures_set()
        except serial.serialutil.SerialException:
            if(rospy.is_shutdown()):
                break
        t_now = rospy.Time.now()
        msg = build_laserscan_msg(distances, t_now - t_last)
        pub_laserscan.publish(msg)
        t_last = t_now
        if(rospy.is_shutdown()):
            print("shutdown")


