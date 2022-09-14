#!/usr/bin/env python3

from lidar import Lidar
import time
from sensor_msgs.msg import LaserScan
import rospy
import serial
from rosgraph import is_master_online

# scan : distances and intensities for a complete revolution
def build_laserscan_msg(scan, duration, frame_id):
    t = time.time()
    scan.reverse() # LaserScan wants measurements to be counter-clockwise
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id #todo vérifier que c'est bien ça
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

    namespace = ""
    if len(rospy.myargv()) != 1:
        if rospy.myargv()[1][0]!="/":
            namespace = "/"
        namespace += rospy.myargv()[1]

    rospy.init_node('lidar_laserscan_publisher', anonymous=True)

    rospy.loginfo("LidarLaserscanPublisher node started with namespace %s", namespace)

    # get params from parameter server
    try:
        find_ports_auto = rospy.get_param(namespace + "/find_ports_auto", False) # if true, no need to specify ports

        port_lidar_sensor = "auto"
        port_motor = "auto"
        if not find_ports_auto:
            port_lidar_sensor = rospy.get_param(namespace + "/port_lidar_sensor") # rad/s
            port_motor = rospy.get_param(namespace + "/port_motor") # rad : offset from hall sensor
        
        speed = rospy.get_param(namespace + "/speed")
        zero_pos = rospy.get_param(namespace + "/zero_position")
        lidar_frame_id = rospy.get_param(namespace + "/frame_id")

    except Exception as e:
        rospy.logerr("Parameters not found, aborting..." + str(e))
        time.sleep(1.0) # wait for log to be published
        rospy.signal_shutdown("Parameters not found")
        exit()

    rospy.loginfo("Started lidar_laserscan_publisher with parameters : speed=" + str(speed) + " zero_pos=" + str(zero_pos) + " port_lidar_sensor=" + str(port_lidar_sensor) + " port_motor=" + str(port_motor) + " lidar_frame_id=" + lidar_frame_id)

    lidar = None
    if find_ports_auto:
        lidar = Lidar()
    else:
        lidar = Lidar(port_lidar_sensor, port_motor)
    
    lidar.start(speed, zero_pos)

    pub_laserscan = rospy.Publisher('lidar_laserscan', LaserScan, queue_size=10)

    rospy.on_shutdown(lidar.stop)

    t_last = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            measures = lidar.get_measures_set()
        except serial.serialutil.SerialException:
            if(rospy.is_shutdown()):
                break
        t_now = rospy.Time.now()
        msg = build_laserscan_msg(measures, t_now - t_last, lidar_frame_id)
        pub_laserscan.publish(msg)
        t_last = t_now
        if(rospy.is_shutdown()):
            rospy.loginfo("Shutdown lidar_laserscan_publisher")


