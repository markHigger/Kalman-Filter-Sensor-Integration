#!/usr/bin/env python
from __future__ import division
import roslib
#roslib.load_manifest('rosopencv')
import sys
import rospy
import math
import time
import utm
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

odomData = Odometry()
newOdomData = False

UTM_E_offset = 328282.63
UTM_N_offset = 4689641.00
ZONE_NUM = 19
ZONE_LETTER = 'T'

########################################################
#Callback functions for each subscriber
########################################################
def updateOdometry(data):
    global odomData, newOdomData
    odomData = data
    newOdomData = True


def main():
    global UTM_E_offset, UTM_N_offset, ZONE_NUM, ZONE_LETTER, odomData, newOdomData

    rospy.init_node('odom_to_gps_node', anonymous=True)

    rospy.loginfo("Odom To GPS Node Initialized")

    rospy.Subscriber("/odometry/filtered", Odometry, updateOdometry, queue_size=10)
    pub = rospy.Publisher('/eece5554/odom_filtered_gps', NavSatFix, queue_size=10)

    time.sleep(1)

    while not rospy.is_shutdown():
        while newOdomData is False:
            time.sleep(0.05)

        newOdomData = False
        satData = NavSatFix()

        #TODO: Determine proper signs for offsets
        #Offset odom position based on initial utm
        utm_x = odomData.pose.pose.position.x + UTM_E_offset
        utm_y = odomData.pose.pose.position.y + UTM_N_offset

        #convert UTM to GPS lat lon
        latlon_data = utm.to_latlon(utm_x, utm_y, ZONE_NUM, ZONE_LETTER)
        satData.latitude = latlon_data[0]
        satData.longitude = latlon_data[1]
        pub.publish(satData)

if __name__ == '__main__':
        main()
