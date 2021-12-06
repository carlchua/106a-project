#!/usr/bin/env python

import rospy
from scipy import interpolate
from msgs.msg import motorcmd # In cloned directory

def trajectory():
    # Interpolate data
    t, x, y, z = interpolateForGazebo(dataDict)

    pub  = rospy.Publisher('asd')
    rospy.init_node('trajectory', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    while not rospy.is_shutdown():
        pub.publish(motorcmd_msg)
        self.mpos_interp_func =  interpolate.interp1d(self.ref_motion['Time'].flatten(), self.ref_motion['Motor_Pos'].T, kind='linear', axis=-1)
        rate.sleep()

def interpolateForGazebo(dataDict):
