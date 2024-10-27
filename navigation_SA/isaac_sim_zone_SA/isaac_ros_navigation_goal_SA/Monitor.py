#!/usr/bin/env python
import pandas as pd
import numpy as np
import math
import rospy
import copy
from zone_SA.Phase_2 import Phase2
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from isaac_sim_zone_SA.msg import stringarr
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
import os
import rospkg
from csv import writer, reader


class Monitoring_module(Phase2):
    
    def __init__(self):
        #get data from phase2 module
        super().__init__()
        self.current_ws = []
        self.current_cs = []

        #pub sub to talk to robot_tasks
        self.status_pub = rospy.Publisher("status", String, queue_size = 10)
        rospy.Subscriber("/rec_loads",numpy_msg(Floats),self.set_loads)
        rospy.Subscriber("/zones",stringarr,self.decode_zones)
        rospy.Subscriber("/critseg",stringarr,self.decode_critseg)

        #pub sub to talk to control
        rospy.Subscriber("/monitor_action",String,self.action) #for NA and A control
        rospy.Subscriber("/send_status",Bool,self.send_status) #signal to send status
        self.take_action = "NA"
        self.status = "NA"
        
        self.workstation_points = pd.read_csv(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data/LE','Workstation_points.csv'), sep=',', header=0, names=['workstation','critical_points'], encoding = 'utf-8')
        self.num_WS = len(self.workstation_points)
        self.rec_loads = np.zeros((self.num_WS,self.num_WS),dtype=np.float32)

        self.zonecount = 0

        self.ph2 = Phase2()
    
    def monitor_status(self):

        print(self.rec_loads)
        print("current zones")
        print(self.current_ws)
        print("current crit segments")
        print(self.current_cs)

        self.num_of_zones = len(self.current_ws)
        self.all_tip_ws = [[] for i in range(self.num_of_zones)]
        self.ph2.set_wsloads(self.rec_loads)

        self.ph2.all_path()

        for zone in range(0,self.num_of_zones):
            self.all_tip_ws[zone].append(self.ph2.finding_tip_ws(self.current_ws[zone],self.current_cs[zone]))

        #calculate how balenced the system is
        if(self.rec_loads.sum() > 0):
            if self.current_ws != []:
                SVp = self.ph2.calc_SVp(self.current_ws, self.all_tip_ws)
            else:
                SVp = 1.0
        else:
            self.status = "NA"
            return
        
        if(SVp > 0.2 or SVp == 0):
            #entering intesive monitoring scheme
            print("system is not balanced SVp:",SVp)
            self.status = "imbalenced"
        else:
            print("system is balanced SVp:",SVp)
            self.status = "balenced"
        
        #record SVp
        current_time = rospy.get_rostime()
        with open(os.path.join(rospkg.RosPack().get_path('isaac_sim_zone_SA'), 'isaac_ros_navigation_goal_SA/zone_SA/data','SVphistory.csv'), 'a') as file:
            writerobj = writer(file)
            writelist = [current_time.secs,SVp]
            writerobj.writerow(writelist)
            file.close()

    def index_to_ws(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        return ws_list[i]

    def ws_to_index(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        i_index = ws_list.index(i)
        return i_index
    
    def ws_crit_point(self,i):
        ws_list = self.workstation_points.loc[:,'workstation'].to_list()
        ws_crit_point = self.workstation_points.loc[:,'critical_points'].to_list()
        i_index = ws_list.index(i)
        return ws_crit_point[i_index]
    
    def get_status(self):
        return self.status
    
    def set_loads(self,data):
        rospy.loginfo("rec loads msg recived")
        self.rec_loads = self.decode_numpy(data.data,self.num_WS,self.num_WS)

    def decode_numpy(self,nparray,numrow,numcol):
            np_recv = np.zeros((numrow,numcol),dtype=np.float32) 
            for x in range(0,numrow):
                for y in range(0,numcol):
                    np_recv[x,y] = nparray[(y+(numrow*x))]
            return np_recv

    def decode_zones(self,data):
        #rospy.loginfo("zone msg recived")
        zone = data.data
        self.append_list(zone,"ws",0)
        
    def decode_critseg(self,data):
        #rospy.loginfo("crit msg recived")
        crit_seg = data.data
        zonei = int(crit_seg[0])
        del crit_seg[0]
        self.append_list(crit_seg,"cs",zonei)

    def send_status(self,data):
        if data.data == True:
            msg = String()
            msg.data = self.status
            self.status_pub.publish(msg)

    def action(self,data):
        self.take_action = data.data

    def get_action(self):
        return self.take_action
    
    def reset_zones(self):
        self.current_ws.clear()
        self.current_cs.clear()

    def append_list(self,list,cs_ws,zone):
        if cs_ws == "ws":
            self.current_ws.append(list)
            self.current_cs.append([])
        elif cs_ws == "cs":
            self.current_cs[zone].append(list)


def main():
    rospy.init_node("monitor_node") #create node
    rospy.loginfo("monitoring module running")
    newMonitor = Monitoring_module()
    start_time = rospy.get_rostime()
    
    #need to set up how monitoring module interacts with control module
    
    newMonitor.reset_zones()
    send_load = rospy.Publisher("send_load", Bool, queue_size = 10)
    send_zone = rospy.Publisher("send_zone", Bool, queue_size = 10)
    load_flag = Bool()
    zone_flag = Bool()
    load_flag.data = False
    zone_flag.data = True

    #wait until monitor is activated by the contol module
    while(newMonitor.get_action() == "NA"):
        continue 

    while not rospy.is_shutdown():
        newaction = newMonitor.get_action()

        #update timer
        current_time = rospy.get_rostime()
        timer = (current_time.secs)-(start_time.secs)
        
        #when control tells monitor module to go into intensive monitoring
        if newaction == "a_IM" and (timer/60) >= 1: #change to 5 later
            #update every 5 min
            print("**********adopting intesive monitoring**********")
            newMonitor.reset_zones()
            send_zone.publish(zone_flag) #get current zone configuarion
            rospy.sleep(1) 
            load_flag.data = True
            send_load.publish(load_flag) #get current recorded load information
            rospy.sleep(.1)
            load_flag.data = False
            send_load.publish(load_flag)
            rospy.sleep(.1) 
            newMonitor.monitor_status()
            start_time = rospy.get_rostime() #reset clock
            rospy.sleep(1) 

        #when control tells monitor module to go into normal monitoring
        elif(newaction == "a_NM" and (timer/60) >= 3): 
            print("*********adopting normal monitoring*************")
            newMonitor.reset_zones()
            send_zone.publish(zone_flag)
            rospy.sleep(1) 
            load_flag.data = True
            send_load.publish(load_flag)
            rospy.sleep(.1)
            load_flag.data = False
            send_load.publish(load_flag)
            rospy.sleep(.1) 
            start_time = rospy.get_rostime() #reset clock
            newMonitor.monitor_status()
            rospy.sleep(1) #only want to hit this once for timer
        
        
        rospy.sleep(1)
    
    
if __name__ == "__main__":
    main()