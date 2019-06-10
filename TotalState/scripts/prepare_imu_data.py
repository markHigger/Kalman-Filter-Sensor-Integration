#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu

pub=[]

def callback(data):
    global pub
    data.header.frame_id='imu_base_link'

    #Convert orientation to rpy
    quaternion_in = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion_in)

    #Transform rpy
    roll = euler[0]
    pitch = -1*euler[1]
    yaw = -1*euler[2]

    #Convert rpy back to quaternion
    quaternion_out = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    data.orientation.x = quaternion_out[0]
    data.orientation.y = quaternion_out[1]
    data.orientation.z = quaternion_out[2]
    data.orientation.w = quaternion_out[3]
    #data.orientation_covariance = [3e-9, 2.1e-9, -8.6e-9,    2.1e-9, 7.4e-9, -8.6e-9,    -8.6e-9, -8.6e-9, 2.058e-7]
    data.orientation_covariance = [3e-8, 2.1e-8, -8.6e-8,    2.1e-8, 7.4e-8, -8.6e-8,    -8.6e-8, -8.6e-8, 2.058e-7]

    #Convert angular velocity
    data.angular_velocity.y = -1*data.angular_velocity.y
    data.angular_velocity.z = -1*data.angular_velocity.z
    data.angular_velocity_covariance = [1.071e-7, 0, -1.8e-9,    0, 1e-9, 0,    -1.8e-9, 0, 1.593e-7]

    #Convert linear acceleration
    data.linear_acceleration.y = -1*data.linear_acceleration.y
    data.linear_acceleration.z = -1*data.linear_acceleration.z
    data.linear_acceleration_covariance = [1.762e-4, -1.7e-6, -2.8e-6,    -1.7e-6, 2.051e-4, -1.9e-6,    -2.8e-6, -1.9e-6, 3.255e-4]

    pub.publish(data)

def converter():
    global pub

    rospy.init_node('prepare_imu_data_node', anonymous=True)
    rospy.Subscriber("/imu/imu", Imu, callback) #Subscriber for full kalman filter
    #rospy.Subscriber("/eece5554/raw_imu", Imu, callback) #Subscriber for testing with VN-100 IMU
    pub = rospy.Publisher("/eece5554/imu", Imu, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    converter()
