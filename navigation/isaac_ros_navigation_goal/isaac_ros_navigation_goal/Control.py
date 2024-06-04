#!/usr/bin/env python
#!/usr/bin/env python
import pandas as pd
import numpy as np
import math
import rospy
import copy
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool

class Contol_module():
    
    def __init__(self):
        #retrive the status every t1 or t2 min
        self.monitor_action_pub = rospy.Publisher("/monitor_action", String, queue_size = 10) #tells monitor mod to switch to NV or IM
        rospy.Subscriber("/status",String,self.rec_status)
        self.sys_status = "NA"

        #tells monitoring module to send status
        self.send_status_pub = rospy.Publisher("/send_status", Bool, queue_size = 10)

        #define the time interval for IM and NV schemes
        self.t1 = 5
        self.t2 = 10
    
        #solution module
        self.solution_action_pub = rospy.Publisher("solution_action", String, queue_size = 10)
    
    def send_status(self):
        msg = Bool()
        msg.data = True
        self.send_status_pub.publish(msg)
    
    def get_status(self):
        return self.sys_status
    
    def rec_status(self,data):
        self.sys_status = data.data

    def pub_monitor_action(self,action):
        msg = String()
        msg.data = action
        self.monitor_action_pub.publish(msg)
    
    def pub_solution_action(self,action):
        msg = String()
        msg.data = action
        self.solution_action_pub.publish(msg)

def main():
    rospy.init_node("control_node") #create node
    rospy.loginfo("contol node active")
    control = Contol_module()

    #start time
    start_time = rospy.get_rostime()

    #wait a certian amount of time to start monitoring the system
    control.pub_monitor_action("NA") #NA message for not active
    rospy.sleep(1)
    control.pub_monitor_action("A") #A message for active
    rospy.sleep(.1) 
    control.pub_monitor_action("a_IM") #notify monitor_mod to go into IM scheme

    #data for if statements
    control.send_status()
    prevStatus = control.get_status()

    #start time
    start_time = rospy.get_rostime()

    while not rospy.is_shutdown():
        current_time = rospy.get_rostime()
        timer = (current_time.secs)-(start_time.secs) #timer is in seconds

        #if statements
        control.send_status() #signal to send status
        rospy.sleep(0.1) #wait for status to be sent and recieved
        status = control.get_status()

        if status == "balenced" and prevStatus == "balenced":
            if timer/60 > 5: #if the system has remained balenced for 5 min then tell monitoring mod to switch to NM
                control.pub_monitor_action("a_NM") 
            
        elif (status == "imbalenced" and (prevStatus == "balenced" or prevStatus == "NA")): #want this to be the first option
            start_time = rospy.get_rostime() #reset the clock
            #activate solution module
            control.pub_solution_action("A") #activate solution module
            rospy.sleep(0.1) #wait for message to be recieved
            control.pub_solution_action("a_LS") #tell solution module to start load sharing
            rospy.sleep(10)
        
        elif status == "balenced" and prevStatus == "imbalenced":
            start_time = rospy.get_rostime() #reset the clock
            control.pub_solution_action("A")
            rospy.sleep(0.1)
            control.pub_monitor_action("a_IM") #tell monitoring module to adopt IM
            control.pub_solution_action("s_LS") #tell solution module to stop load sharing
            rospy.sleep(0.1)
            control.pub_solution_action("NA") #shutdown solution module

        elif status == "imbalenced" and prevStatus == "imbalenced":
            #if timer/60 > 20: #if the system has remained unbalenced for 20min
            if timer/60 > 15: #testing 
                start_time = rospy.get_rostime() #reset the clock
                control.pub_solution_action("A")
                control.pub_solution_action("s_LS") #tell solution mod to stop load sharing
                rospy.sleep(0.1)
                control.pub_solution_action("a_ZR") #tell solution module to activate zone-Repartion
                rospy.sleep(0.1)
                control.pub_solution_action("NA")
                rospy.sleep(60) #wait a minute before chacking status
            else:
                control.pub_solution_action("a_LS") #tell solution module to start load sharing
                control.pub_monitor_action("a_IM")
                rospy.sleep(60) #wait a minute before activating load sharing again

        prevStatus = status
        #control.pub_solution_action("NA")
        rospy.sleep(0.1)
    
    
if __name__ == "__main__":
    main()
