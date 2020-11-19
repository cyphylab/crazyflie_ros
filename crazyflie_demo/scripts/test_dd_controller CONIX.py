#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('test_dd_controller')

    cf = crazyflie.Crazyflie("cf1", "/tf")

 #   rospy.wait_for_service('update_params')
 #   update_params = rospy.ServiceProxy("update_params", UpdateParams)
    

    cf.setParam("controller_dd/ctrl_ddA", 0.0)
 #  time.sleep(0.5)
 #   update_params(["controller_dd/ctrl_ddA"])

    cf.setParam("controller_dd/ctrl_ddB", 1.0)
 #  time.sleep(0.5)
 #   update_params(["controller_dd/ctrl_ddB"])
    
    cf.setParam("stabilizer/dd_ctrl_active", 0)
 #   time.sleep(0.5)
 #   update_params(["controller_dd/dd_ctrl_active"])
    
    cf.setParam("controller_dd/ctrl_ddP1",-1.5) #-2
 #  time.sleep(0.5)
 #   update_params(["controller_dd/ctrl_ddP1"])
    
    cf.setParam("controller_dd/ctrl_ddP2",-1.0) #-1.5
 #  time.sleep(0.5)
 #   update_params(["controller_dd/ctrl_ddP2"])
    
    cf.setParam("controller_dd/ctrl_ddg1", 1.2) #0.002
 #  time.sleep(0.5)
 #   update_params(["controller_dd/ctrl_ddg1"])
    
    cf.setParam("controller_dd/ctrl_ddg2", 0.2)   #5e-8f
 #  time.sleep(0.5)
 #   update_params(["controller_dd/ctrl_ddg2"])
    
    cf.setParam("controller_dd/ctrl_ddTr", 0.3)
 #  time.sleep(0.5)
 #   update_params(["controller_dd/ctrl_ddTr"])
    

    cf.takeoff(targetHeight = 0.8, duration = 5.0)
    cf.takeoff(targetHeight = 0.8, duration = 5.0)
    cf.takeoff(targetHeight = 0.8, duration = 5.0)
    cf.takeoff(targetHeight = 0.8, duration = 5.0)
    time.sleep(5.0)
    
    # Hovering time
    cf.goTo(goal = [0.0, 0.0, 0.4], yaw=0.0, duration = 0.05, relative = True)
    #cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.0, duration = 0.2, relative = True)
    #cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.0, duration = 0.2, relative = True)
    #time.sleep(10.0)
    #cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.0, duration = 1, relative = True)
    #cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.0, duration = 1, relative = True)
    #cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.0, duration = 1, relative = True)
    time.sleep(5.0)
    #cf.goTo(goal = [0.0, 0.0, -1.2], yaw=0.0, duration = 0.5, relative = True)
    #cf.goTo(goal = [0.0, 0.0, 0.2], yaw=0.0, duration = 1.0, relative = True)
    #cf.goTo(goal = [0.0, 0.0, 0.2], yaw=0.0, duration = 1.0, relative = True)
    # Starting the DD Controller
    print("Activate Data Driven Controller")
    while (cf.getParam("stabilizer/dd_ctrl_active") != 1):
        cf.setParam("stabilizer/dd_ctrl_active", 1)
        time.sleep(1.0)
    print("Activate Data Driven Controller")


    # Hovering with the DD active
    #time.sleep(5.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.3)
    time.sleep(10.0)
    cf.setParam("controller_dd/ctrl_ddTr", 1.1)
    time.sleep(15.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.7)
    time.sleep(3.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.5)
    time.sleep(5.0)
    cf.setParam("controller_dd/ctrl_ddTr", 1.1)
    time.sleep(7.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.7)
    time.sleep(3.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.4)
    time.sleep(2.0)
    #cf.setParam("controller_dd/ctrl_ddTr", 1.1)
    #time.sleep(7.0)
    #cf.setParam("controller_dd/ctrl_ddTr", 0.7)
    #time.sleep(2.0)
    #cf.setParam("controller_dd/ctrl_ddTr", 0.5)
    #time.sleep(2.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.2)
    time.sleep(2.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.1)
    time.sleep(2.0)
    cf.setParam("controller_dd/ctrl_ddLd", 1)
    #while (cf.getParam("stabilizer/dd_ctrl_active") != 0):
        #cf.setParam("stabilizer/dd_ctrl_active", 0)
        #time.sleep(0.5)
        


    # Landing
    #cf.land(targetHeight = 0.0, duration = 5.0)
    #cf.land(targetHeight = 0.0, duration = 5.0)
    time.sleep(3.0)
    cf.stop()
