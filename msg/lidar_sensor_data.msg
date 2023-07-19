# Single Measurement of a range value with corresponding angular positioning of the sensor

Header header                   # timestamp is the aquisition time of the measurement in Gazebo
                                # frame_id is the identifier the ray sensor element is attached to
std_msgs/Float32 range          # range data [m]
std_msgs/Float32 angle          # angular position of the sensor [rad]