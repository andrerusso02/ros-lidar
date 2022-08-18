#!/usr/bin/env python3

from lidar import Lidar
import time
from sensor_msgs.msg import LaserScan
import rospy

def build_laserscan_msg(distances, duration):
    t = time.time()
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "lidar" #todo vérifier que c'est bien ça
    msg.angle_min = 0.0
    msg.angle_max = 3.14159265359*2
    msg.angle_increment = 3.14159265359*2/len(distances)
    msg.time_increment = duration.to_sec()/len(distances)
    msg.scan_time = duration.to_sec()
    msg.range_min = 0.08
    msg.range_max = 12.0
    msg.ranges = distances
    # msg.intensities
    return msg


if __name__ == '__main__':

    lidar = Lidar()
    lidar.start(3.14159*3)
    print("started")

    rospy.init_node('lidar_laserscan_publisher', anonymous=True)

    pub_laserscan = rospy.Publisher('lidar_laserscan', LaserScan, queue_size=10)

    t_last = rospy.Time.now()

    while not rospy.is_shutdown():
        distances = lidar.get_distances_set()
        t_now = rospy.Time.now()
        msg = build_laserscan_msg(distances, t_now - t_last)
        pub_laserscan.publish(msg)
        t_last = t_now


