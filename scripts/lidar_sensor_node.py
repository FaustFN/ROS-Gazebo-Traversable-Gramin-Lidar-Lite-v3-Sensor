#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry
from geometry_msgs.msg import Point
import tf.transformations
from rotating_lidar_sensor_position_controller.msg import lidar_sensor_data


# returns the rotation angle from a rotatin and translation matrix
def transform_angles(matrix):
    angle, direc, point = tf.transformations.rotation_from_matrix(matrix)
    return angle


#Apply a gaussian distribution to the range value from the gazebo simulation to imitate the real sensor behavior
def getGaussian(value):
    if np.isinf(value):
        return value
    #accuracy value for 2 sigma certancy
    if value < 5:
        accuracy_min = 0.025
        accuracy_current = value / 5 * accuracy_min

    else:
        accuracy_min = 0.1
        accuracy_current = value / 40 * (accuracy_min - 0.025) + 0.025
    
    gaussian_sample = np.random.default_rng().normal(value, accuracy_current/2)
    return gaussian_sample
 

class lidar_sensor:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.laserproj = laser_geometry.LaserProjection()
        self.sensor_data_pub = rospy.Publisher("/sensor/data", lidar_sensor_data, queue_size=2)
        self.sensor_sub = rospy.Subscriber("/sensor/laser/scan", LaserScan, self.sensor_data_callback)
        self.sensor_data_raw = LaserScan()


    # Look up for the transformation between target_frame and source_frame
    def update_base_link_to_laser_offset(self, target_frame, source_frame):
        self.base_link_to_laser_offset = None
        while self.base_link_to_laser_offset is None:
            try:
                self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
                trans, rot = self.tf_listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                self.base_link_to_laser_offset = (trans[0], trans[1])
                self.matrix = self.tf_listener.fromTranslationRotation(trans, rot)
            except Exception as ex:
                rospy.logerr(str(ex))
                rospy.logwarn('Retrying')
                self.base_link_to_laser_offset = None
                rospy.sleep(0.1)
        rospy.logdebug('base_link to laser offset: ' + str(self.base_link_to_laser_offset))


    #On receiving a LaserScan message on the topic "/sensor/laser/scan", update the tf tree for the sensor module and publish data on Topic "/sensor/data"
    def sensor_data_callback(self, msg):
        self.sensor_data_raw = msg
        sourceframe = self.sensor_data_raw.header.frame_id
        self.update_base_link_to_laser_offset("sensor_base_link", sourceframe)
        ranges = self.sensor_data_raw.ranges
        rotation_angle = transform_angles(self.matrix)
        new_laser_data = lidar_sensor_data()
        new_laser_data.header.frame_id = self.sensor_data_raw.header.frame_id
        new_laser_data.header.stamp = self.sensor_data_raw.header.stamp
        new_laser_data.range.data = getGaussian(ranges[0])
        new_laser_data.angle.data = rotation_angle
        self.sensor_data_pub.publish(new_laser_data)       


# Called on shut down
def shutdownhook():
    print("[lidar_sensor_node] killing on exit")


if __name__ == '__main__':
    rospy.init_node('lidar_sensor_node', anonymous=True)
    rospy.loginfo('Starting sensor node')
    rospy.on_shutdown(shutdownhook)
    ls = lidar_sensor()
    rospy.spin()