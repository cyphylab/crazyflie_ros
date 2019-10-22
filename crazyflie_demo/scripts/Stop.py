#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('test_dd_controller')

    cf = crazyflie.Crazyflie("cf1", "/tf")
    cf.setParam("controller_dd/ctrl_ddLd", 1)
    cf.stop()
