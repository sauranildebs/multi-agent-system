#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mavros_msgs.srv import *



def setGuidedMode():
    rospy.wait_for_service('rover2/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('rover2/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException, e:
        print "GUIDED Mode could not be set."%e

def setAutoMode():
    rospy.wait_for_service('rover2/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('rover2/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='AUTO') #return true or false
    except rospy.ServiceException, e:
        print "AUTO Mode could not be set."%e


        

          
def setArm():
    rospy.wait_for_service('rover2/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('rover2/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException, e:
        print "Arming failed"%e
        
def setDisarm():
    rospy.wait_for_service('rover1/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('rover1/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException, e:
        print "Disarm failed"%e


def menu():
    print " "
    print "Press 1: Guided Mode"
    print "Press 2: AUTO Mode"
    print "Press 3: Arm"
    print "Press 4: Disarm"
    
def loop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4'])):
        menu()
        x = raw_input("Input ");
        if (x=='1'):
            setGuidedMode()
            print "Mode: Guided"
	elif(x=='2'):
            setAutoMode()
            print "Mode: Auto"
        elif(x=='3'):
            setArm()
            print "Armed"
        elif(x=='4'):
            setDisarm()
            print "Disarmed"
        else: 
            print "Exit"
        
        
    

if __name__ == '__main__':
    try:
        rospy.init_node('rover', anonymous=True)
        loop()

    except rospy.ROSInterruptException:
        pass
