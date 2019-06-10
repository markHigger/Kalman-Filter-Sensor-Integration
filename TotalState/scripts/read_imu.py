#!/usr/bin/env python
from __future__ import division
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
import time
import utm
import serial
import re
import tf
from sensor_msgs.msg import Imu


def main():

    rospy.init_node('read_imu_node', anonymous=True)

    rospy.loginfo("Read IMU Initialized")

    pub = rospy.Publisher('/eece5554/raw_imu', Imu, queue_size=10)

    time.sleep(1)
    port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.)  # 4800-N-8-1

    while not rospy.is_shutdown():
        line = port.readline()
        #print line # For debugging

        imu_msg = Imu()

        if "$VNYMR" in line:
            try:
                msg = re.split('\*',line) #Remove checksum
                msg = re.split(',',msg[0]) #Split data

                #currentDT = datetime.datetime.now()
                #hr=currentDT.hour
                #min=currentDT.minute
                #sec=currentDT.second
                #ms=currentDT.microsecond
                #self.packet.timestamp = hr*3600 + min*60 + sec + ms*(10**-len(str(ms)))

                #Attitude
                yaw = float(msg[1]) #Yaw - z
                pitch = float(msg[2]) #Pitch - y
                roll = float(msg[3]) #Roll - x
                #TODO: Convert to quaternion
                quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                #type(pose) = geometry_msgs.msg.Pose
                #imu_msg.orientation.x = quaternion[0]
                #imu_msg.orientation.y = quaternion[1]
                #imu_msg.orientation.z = quaternion[2]
                #imu_msg.orientation.w = quaternion[3]

                imu_msg.orientation.x = roll
                imu_msg.orientation.y = pitch
                imu_msg.orientation.z = yaw
                #imu_msg.orientation.w = quaternion[3]

                #Magnetic
                #self.packet.mag.x = float(msg[4])
                #self.packet.mag.y = float(msg[5])
                #self.packet.mag.z = float(msg[6])

                #Acceleration
                imu_msg.linear_acceleration.x = float(msg[7])
                imu_msg.linear_acceleration.y = float(msg[8])
                imu_msg.linear_acceleration.z = float(msg[9])

                #Gyroscope
                imu_msg.angular_velocity.x = float(msg[10])
                imu_msg.angular_velocity.y = float(msg[11])
                imu_msg.angular_velocity.z = float(msg[12])

                pub.publish(imu_msg)
            except Exception, e:
                print 'CT ERROR (' + 'error: '+ str(e)+ ' line: ' + line + ')'


if __name__ == '__main__':
        main()
