#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':
    rospy.init_node('test_dd_controller')

    cf = crazyflie.Crazyflie("/cf1", "/tf")
#    rospy.wait_for_service('update_params')
#    rospy.loginfo("found update_params service")
#    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    cf.setParam("stabilizer/estimator", 4)
    time.sleep(0.5)
    cf.setParam("stabilizer/controller", 1)
    time.sleep(0.5)
   
    cf.setParam("controller_dd/ctrl_ddA", -5.0)
    time.sleep(0.5)
#    update_params(["controller_dd/ctrl_ddA"])

    cf.setParam("controller_dd/ctrl_ddB", 10.0)
    time.sleep(0.5)
#    update_params(["controller_dd/ctrl_ddB"])

    cf.setParam("stabilizer/dd_ctrl_active", 0)
    time.sleep(0.5)
#    update_params(["controller_dd/dd_ctrl_active"])

    cf.setParam("controller_dd/ctrl_ddP1",-3.0) #-2
    time.sleep(0.5)
#    update_params(["controller_dd/ctrl_ddP1"])

    cf.setParam("controller_dd/ctrl_ddP2",-2.0) #-1.5
    time.sleep(0.5)
#    update_params(["controller_dd/ctrl_ddP2"])

    cf.setParam("controller_dd/ctrl_ddg1", 0.1) #0.002
    time.sleep(0.5)
#    update_params(["controller_dd/ctrl_ddg1"])

    cf.setParam("controller_dd/ctrl_ddg2", 0.3)   #5e-8f
    time.sleep(0.5)

    cf.setParam("controller_dd/ctrl_bfac", 1)

    cf.setParam("controller_dd/ctrl_ddTr", 1.6)

    cf.setParam("controller_dd/ctrl_thr", 100)

    cf.setParam("controller_dd/ctrl_Mtd", 1)
	
    cf.setParam("controller_dd/ctrl_lbb", 1.0)
	
    cf.setParam("controller_dd/ctrl_ubb", 30.0)
	
    time.sleep(0.5)
#    update_params(["controller_dd/ctrl_ddTr"])
    print("0")
    cf.takeoff(targetHeight = 1.6, duration = 5.0)
    time.sleep(5)
    while (cf.getParam("stabilizer/dd_ctrl_active") != 1):
        cf.setParam("stabilizer/dd_ctrl_active", 1)
    
    print(cf.getParam("stabilizer/dd_ctrl_active"))
    time.sleep(1.0)
		
#	time.sleep(1.0)
#	while (cf.getParam("controller_dd/ctrl_0") != 1):
#		cf.setParam("controller_dd/ctrl_0", 1)
#		print(cf.getParam("controller_dd/ctrl_0"))
#		time.sleep(1.0)
#	print("2")


# Hovering with the DD active
    time.sleep(5.0)
    Counter=0
#	while (Counter != 1):
    print("Setpoint 1.6")
    cf.setParam("controller_dd/ctrl_ddTr", 1.6)
    time.sleep(15.0)
	#cf.setParam("controller_dd/ctrl_ddTr", 0.8)
	#print("Activate Data Driven Controller 0.8")
	#time.sleep(10.0)
	#cf.setParam("controller_dd/ctrl_ddTr", 1.0)
	#print("Activate Data Driven Controller 1.0")
	#time.sleep(10.0)
	#cf.setParam("controller_dd/ctrl_ddTr", 1.3)
	#print("Activate Data Driven Controller 1.3")
	#time.sleep(10.0)
#		cf.setParam("controller_dd/ctrl_ddTr", 1.0)
#		time.sleep(10.0)
#		cf.setParam("controller_dd/ctrl_ddTr", 0.7)
#		time.sleep(10.0)
#		cf.setParam("controller_dd/ctrl_ddTr", 0.4)
#		time.sleep(10.0)
#		cf.setParam("controller_dd/ctrl_ddTr", 0.15)
#		time.sleep(0.4)
#		Counter=Counter+1
#	cf.setParam("controller_dd/ctrl_ddTr", 1.8)
#	time.sleep(10.0)
#	cf.setParam("controller_dd/ctrl_ddTr", 1.0)
#	time.sleep(5.0)
#	cf.setParam("controller_dd/ctrl_ddTr", 0.7)
#	time.sleep(5.0)
#	cf.setParam("controller_dd/ctrl_ddTr", 0.4)
#	time.sleep(5.0)
#	cf.setParam("controller_dd/ctrl_ddTr", 0.15)
#	time.sleep(5.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.7)
    time.sleep(2.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.5)
    time.sleep(2.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.2)
    time.sleep(2.0)
    cf.setParam("controller_dd/ctrl_ddTr", 0.1)
    time.sleep(2.0)
    cf.setParam("controller_dd/ctrl_ddLd", 1)
    while (cf.getParam("stabilizer/dd_ctrl_active") != 0):
        cf.setParam("stabilizer/dd_ctrl_active", 0)
        time.sleep(0.5)



# Landing
#cf.land(targetHeight = 0.0, duration = 5.0)
#cf.land(targetHeight = 0.0, duration = 5.0)
    time.sleep(1.0)
    cf.stop()
