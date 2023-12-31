#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64


def deg2rad(angle_deg):
    angle_rad = angle_deg * (math.pi/180)
    return angle_rad


def rad2deg(angle_rad):
    angle_deg = angle_rad * (180/math.pi)
    return angle_deg


class actuator:
    def __init__(self):
        self.actuator_angle = float()
        self.actuator_min_angle = rospy.get_param("/sensor_actuator_node/actuator_min_angle")
        self.actuator_max_angle = rospy.get_param("/sensor_actuator_node/actuator_max_angle")
        self.actuator_angle_sub = rospy.Subscriber("/sensor/actuator/angle", Float64, self.sensor_actuator_callback)
        self.actuator_angle_pub = rospy.Publisher("/actuator_position_controller/command", Float64, queue_size=2)
                

    # On recieving a message on the '/sensor/actuator/angle' topic, check if the angle is within range of the lower and upper angle threshold.
    # calculate the angle in radian and publish it on the '/actuator_position_controller/command' topic
    def sensor_actuator_callback(self, msg):
        angle_deg = msg.data
        if angle_deg > actuator.actuator_max_angle:
            angle_deg = actuator.actuator_max_angle
            rospy.logwarn('Target angle is larger than max. angle. Setting angle to %s°.' % (actuator.actuator_max_angle))
        if angle_deg < actuator.actuator_min_angle:
            angle_deg = actuator.actuator_min_angle
            rospy.logwarn('Target angle is smaller than min. angle. Setting angle to %s°.' % (actuator.actuator_min_angle))
        self.actuator_angle = deg2rad(angle_deg)
        self.actuator_angle_pub.publish(self.actuator_angle)


#Called on shut down
def shutdownhook():
    print("[sensor_actuator_node] killing on exit")


if __name__ == '__main__':
    rospy.init_node('sensor_actuator_node', anonymous=True)
    rospy.loginfo('Starting actuator node')
    rospy.on_shutdown(shutdownhook)
    actuator = actuator()
    rospy.spin()